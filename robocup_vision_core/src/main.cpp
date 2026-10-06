#include "camera_subscriber.hpp"
#include "crop_space.hpp"
#include "voxelization.hpp"
#include "ransac_plane.hpp"
#include "dbscan_clustering.hpp"
#include "obb_estimator.hpp"
#include "plana_obb_estimator.hpp"
#include "obb_visualizer.hpp"
#include "color_utils.hpp"
#include "classifier.hpp"
#include "classification_visualizer.hpp"
#include "block_registry.hpp"
#include <robocup_vision_interfaces/srv/grasp_pose.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>
#include <tuple>
#include <cstdint>
#include <algorithm>


int main(int argc, char **argv){

    rclcpp::init(argc, argv);//initialize a ros2 node
    
    // Create an object of class CameraSubscriber (define in camera_subscriber.hpp)
    auto node = std::make_shared<robocup_vision_core::CameraSubscriber>();
    
    // Publishers for each pipeline stage (helpful for debugging in RViz)
    auto pub_roi      = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_roi", 10);// after cropped
    auto pub_objects  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_objects", 10);// after ransac
    auto pub_clusters = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_clusters", 10);// after clustering
    auto pub_obb_markers = node->create_publisher<visualization_msgs::msg::MarkerArray>("/obb_markers", 10);// after OBB computation
    
    robocup_vision_core::CropSpace cropper; // defaults from header (-0.3,-0.5,0.3) -> (0.3,0.5,5.0)
    robocup_vision_core::Voxelizer vxl(0.003f, 0.003f, 0.003f); //3mm voxels

    /* Set up the classifier with calibrated color centroids (hue values in degrees)*/
    robocup_vision_core::Classifier classifier;
    // Calibrated color centroids (hue values in degrees) for specific camera and lighting conditions
    classifier.setColorCentroids({
        { robocup_vision_core::ColorLabel::RED,    0.0f   },
        { robocup_vision_core::ColorLabel::YELLOW, 37.3f  },
        { robocup_vision_core::ColorLabel::GREEN,  141.0f },
        { robocup_vision_core::ColorLabel::BLUE,   215.3f },
    });

    /** Create a BlockRegistry instance to store the latest OBBs for each block label **/
    robocup_vision_core::BlockRegistry registry;
    // Create a ROS2 service to provide grasp poses for detected blocks
    auto grasp_service = node->create_service<robocup_vision_interfaces::srv::GraspPose>(
        "get_grasp_pose",
        [&registry, node](
            const std::shared_ptr<robocup_vision_interfaces::srv::GraspPose::Request> request,
            std::shared_ptr<robocup_vision_interfaces::srv::GraspPose::Response> response)
        { // Service callback to handle grasp pose requests
            auto entry = registry.lookup(request->target_label, node->now());
            if (!entry.has_value()) {
                response->success = false;
                response->message = "Block '" + request->target_label + "' not currently detected";
                return;
            }
            // Fill the response with the latest OBB for the requested block label
            const auto &obb = entry->obb;
            response->success = true;
            response->message = "OK";
            response->pose.header.frame_id = "camera_link";// use the same frame_id as the point cloud
            response->pose.header.stamp = entry->stamp;
            response->pose.pose.position.x = obb.position.x();
            response->pose.pose.position.y = obb.position.y();
            response->pose.pose.position.z = obb.position.z();
            response->pose.pose.orientation.w = obb.orientation.w();
            response->pose.pose.orientation.x = obb.orientation.x();
            response->pose.pose.orientation.y = obb.orientation.y();
            response->pose.pose.orientation.z = obb.orientation.z();
            response->width_m = std::min(obb.extents.x(), obb.extents.y());
        }
    );
    // Log that the grasp pose service is ready - comment if real-time
    RCLCPP_INFO(node->get_logger(), "Grasp pose service ready: /get_grasp_pose");
    
    rclcpp::Rate rate(10); // 10 Hz processing loop
    
    while (rclcpp::ok()) {

        // Process pending callbacks 
        //replace for spin() to allow loop to run  continuously without blocking on callbacks
        rclcpp::spin_some(node);

        // 1. Get the latest raw cloud
        auto raw = node -> getLatestCloud();
        if (!raw || raw->empty()) { rate.sleep(); continue; }
        //Log the number of raw points - comment if real-time
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "Raw cloud: %zu points", raw->size()); 

	    // 2. Crop Region of Interest (ROI)
        auto roi = cropper.crop(raw);
        if (!roi || roi->empty()) { rate.sleep(); continue; }
        //Log the number of cropped points - comment if real-time
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "Cropped cloud: %zu points", roi->size()); 
        
        
        // Publish ROI for visualization
        {
            sensor_msgs::msg::PointCloud2 roi_msg;
            pcl::toROSMsg(*roi, roi_msg);
            roi_msg.header.frame_id = roi->header.frame_id.empty() ? std::string("camera_link") : roi->header.frame_id;
            roi_msg.header.stamp = node->now();
            pub_roi->publish(roi_msg);
        }
        
        // 3. voxelize
        auto cloud_voxel = vxl.voxelize(roi);
        if (!cloud_voxel || cloud_voxel->empty()) { rate.sleep(); continue; }
        //Log the number of voxel points - comment if real-time
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "Voxelized cloud: %zu points", cloud_voxel->size());

        
        // 4. Run RANSAC plane removal
        pcl::ModelCoefficients::Ptr plane_coeffs(new pcl::ModelCoefficients);
        auto objects = RansacPlane::remove_plane(cloud_voxel, plane_coeffs);
        if (!objects || objects->empty()) { rate.sleep(); continue; }
        if (!plane_coeffs || plane_coeffs->values.size() != 4) { rate.sleep(); continue; }

        // coefficients of the detected plane (ax + by + cz + d = 0)
        Eigen::Vector4f plane_vec(
        plane_coeffs->values[0], plane_coeffs->values[1],
        plane_coeffs->values[2], plane_coeffs->values[3]);

        // Publish cloud after plane removal to RVIZ - comment if real-time
        sensor_msgs::msg::PointCloud2 objects_msg;// ROS2 message for point cloud
        pcl::toROSMsg(*objects, objects_msg);//Convert PCL → ROS2
        objects_msg.header.frame_id = objects->header.frame_id.empty() ? cloud_voxel->header.frame_id : objects->header.frame_id;
        objects_msg.header.stamp = node->now();// DBSCAN returning indices,avoid copying cloud
        pub_objects->publish(objects_msg);// publish to RVIZ
        // Log the number of points in the objects cloud after plane removal-comment if real-time
        RCLCPP_INFO_THROTTLE( node->get_logger(), *node->get_clock(), 2000,
                              "RANSAC cloud: %zu points", objects->size()); 

        // 5. Run DBSCAN clustering on the objects cloud
        // DBSCAN parameters
        double eps = 0.01;     // 1 cm radius
        int min_samples = 30;  // minimum neighbors to determine a core point
        auto clusters = DBSCANClustering::dbscan_indices(objects, eps, min_samples);
        // Filter out small clusters
        int min_cluster_size = 150;// min_sample to considered a valid cluster
        std::vector<pcl::PointIndices> filtered;
        for (auto &ci : clusters) {
            if ((int)ci.indices.size() >= min_cluster_size)
                filtered.push_back(ci);
        }
        clusters.swap(filtered);       
        // Log the number of clusters found - comment if real-time
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(),2000,
                             "Number of clusters: %zu", clusters.size());
        // Create colored cloud for RViz - comment if real-time
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored->resize(objects->size());
        for (size_t i = 0; i < objects->size(); ++i) {
            (*colored)[i] = (*objects)[i];
            // default color = gray
            (*colored)[i].r = 128; (*colored)[i].g = 128; (*colored)[i].b = 128;
        }
        //color palette for clusters - comment if real-time
        std::vector<std::tuple<uint8_t,uint8_t,uint8_t>> palette = {
            {255, 0, 0},     // Red
            {0, 255, 0},     // Green
            {0, 0, 255},     // Blue
            {255, 255, 0},   // Yellow
            {255, 0, 255},   // Magenta
            {0, 255, 255},   // Cyan
            {255, 128, 0},   // Orange
            {128, 0, 255},   // Purple
            {0, 255, 128},   // Spring Green
            {255, 0, 128},   // Pink
            {128, 255, 0},   // Lime
            {0, 128, 255},   // Sky Blue
            {128, 0, 0},     // Maroon
            {0, 128, 0},     // Dark Green
            {0, 0, 128}      // Navy
        };
        for (size_t cid = 0; cid < clusters.size(); ++cid) {
            auto color = palette[cid % palette.size()];
            for (int idx : clusters[cid].indices) {
                (*colored)[idx].r = std::get<0>(color);
                (*colored)[idx].g = std::get<1>(color);
                (*colored)[idx].b = std::get<2>(color);
            }
        }
        // Publish the colored clusters cloud to RVIZ - comment if real-time
        sensor_msgs::msg::PointCloud2 clusters_msg;
        pcl::toROSMsg(*colored, clusters_msg);
        //set clusters_msg header frame using the same frame_id we derived above
        clusters_msg.header.frame_id = objects_msg.header.frame_id;
	    // publish
        clusters_msg.header.stamp = node->now();
        pub_clusters->publish(clusters_msg);


        // 6. For each cluster, compute OBB and log the results       
        
        // create a MarkerArray to hold all OBB markers for visualization - comment if real-time
        visualization_msgs::msg::MarkerArray marker_array;

        // Clear the BlockRegistry at the start of each frame to remove stale entries
        registry.clear();

        // Clear all markers from the previous frame first, to avoid stale
        // "ghost" boxes/labels lingering when a cluster disappears or count
        // changes between frames (marker.lifetime = 0 means they never expire on their own).
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Loop through each cluster to compute OBB and create visualization markers
        for (size_t cid = 0; cid < clusters.size(); ++cid) {
            // Extract cloud points for this cluster
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>
            );
            
            for (int idx : clusters[cid].indices) {
                cluster_cloud->push_back((*objects)[idx]);
            }
            
            // Compute OBB for this cluster
            robocup_vision_core::OBB obb = robocup_vision_core::PlanarObbEstimator::computeOBB(cluster_cloud, plane_vec);
            
            /* only for debugging
            // Log OBB results - only for debugging, comment out if real-time
            RCLCPP_INFO(node->get_logger(),
                "Cluster %zu: points=%zu | position=(%.4f, %.4f, %.4f) | "
                "extents=(%.4f, %.4f, %.4f) | volume=%.6f",
                cid, cluster_cloud->size(),
                obb.position.x(), obb.position.y(), obb.position.z(),
                obb.extents.x(), obb.extents.y(), obb.extents.z(),
                obb.volume()
            );
            
            // Log orientation (quaternion) - only for debugging, comment out if real-time
            RCLCPP_INFO(node->get_logger(),
                "  Orientation (quat): w=%.4f, x=%.4f, y=%.4f, z=%.4f",
                obb.orientation.w(), obb.orientation.x(), 
                obb.orientation.y(), obb.orientation.z()
            );
            
            // Log corner points - only for debugging, comment out if real-time
            auto corners = obb.corners();
            RCLCPP_DEBUG(node->get_logger(),
                "  Corners (world coords):");
            for (size_t i = 0; i < corners.size(); ++i) {
                RCLCPP_DEBUG(node->get_logger(),
                    "    Corner %zu: (%.4f, %.4f, %.4f)",
                    i, corners[i].x(), corners[i].y(), corners[i].z()
                );
            }*/

            // Create marker from OBB and add to MarkerArray - comment if real-time
            auto marker = robocup_vision_core::ObbVisualizer::createBoxMarker(
                obb, cid, objects_msg.header.frame_id, node->now());
            marker_array.markers.push_back(marker);// add marker to array

            // 7. Classify each cluster by shape and color 
            // inside the same loop to avoid double compute obb for shape classification
            auto result = classifier.classify(cluster_cloud, obb);

            // Log the classification results for this cluster - comment if real-time
            /* only for light condition calibration
            RCLCPP_INFO(node->get_logger(),
            "Cluster %zu: shape=%s (ratio=%.2f) | color=%s (hue=%.1f, dist=%.1f, used_pts=%zu/%zu)",
            cid,
            robocup_vision_core::toString(result.shape).c_str(), result.shape_ratio,
            robocup_vision_core::toString(result.color).c_str(),
            result.hue_stats.median_hue_deg, result.color_match_distance,
            result.hue_stats.used_points, result.hue_stats.total_points
            );
            */

            // Create a label marker for this cluster and add to MarkerArray - comment if real-time
            // Use the ClassificationVisualizer to create a text marker above the OBB
            auto label_marker = robocup_vision_core::ClassificationVisualizer::createLabelMarker(
                obb, result, cid, objects_msg.header.frame_id, node->now());
                marker_array.markers.push_back(label_marker);

            // 8. Update the BlockRegistry with the latest OBB for this cluster
            std::string label = robocup_vision_core::ClassificationVisualizer::abbreviate(result);
            if (result.isFullyClassified()) {registry.update(label, obb, node->now());}
        }
        // Publish the MarkerArray to RVIZ for visualization
        pub_obb_markers->publish(marker_array); 


        rate.sleep();// Sleep to maintain the loop rate
    }
    
    rclcpp::shutdown();
    return 0;
}

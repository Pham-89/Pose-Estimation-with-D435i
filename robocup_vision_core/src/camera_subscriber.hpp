#pragma once

// camera_subscriber.hpp
// Subscribe a ROS2 sensor_msgs::msg::PointCloud2 and convert to pcl::PointCloud<pcl::PointXYZRGB>
// Exposes thread-safe accessor getLatestCloud() returning a shared_ptr copy.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <mutex>

namespace robocup_vision_core {

/* 
Input: ROS2 point cloud
Output: raw point cloud
*/
	class CameraSubscriber : public rclcpp::Node {
		public:
			CameraSubscriber();

    			// Return a copy of the shared_ptr to the latest raw PCL cloud.
    			// Copying shared_ptr is thread-safe; the returned pointer points to an immutable
    			// object (callback creates new cloud instances each frame).
    			 pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getLatestCloud() const;

		private:
    			void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    			// stored cloud (raw, before crop/voxel). Protected by latest_cloud_mutex_.
    			pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_raw_cloud_;
    			mutable std::mutex latest_cloud_mutex_;  // mutable so getLatestCloud() can be const

    			rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    			// optional publisher for raw cloud for visualization/debug
    			rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_raw_;
    
	};
} // namespace robocup_vision_core


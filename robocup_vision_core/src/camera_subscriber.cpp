#include "camera_subscriber.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robocup_vision_core{

// camera_subscriber.cpp
// Implementation: subscribe & convert ROS2 -> PCL, store latest cloud safely.


    CameraSubscriber::CameraSubscriber(): Node("camera_subscriber") //initialize name node with "camera_subscriber"
    { 
   
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points",
        	rclcpp::SensorDataQoS(),
        	std::bind(&CameraSubscriber::pointcloud_callback, this, std::placeholders::_1)
        );
    

        // Initialize publishers for rviz - comment for real-time
        pub_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_raw", 10);
    
        // Initialize buffer for preprocessed cloud
        latest_raw_cloud_.reset();

        RCLCPP_INFO(this->get_logger(), "Camera subscriber started");
    }


    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr CameraSubscriber::getLatestCloud() const
    {
        std::lock_guard<std::mutex> lk(latest_cloud_mutex_);
        return latest_raw_cloud_;
    }


    void CameraSubscriber::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    
        // 1. Publish raw cloud for RViz - comment if real-time
        pub_raw_->publish(*msg);
    
        // 2. Create a new PCL cloud instance per callback (avoid mutating shared buffer)
        // Use PCL Ptr (boost::shared_ptr) compatibility.
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    	pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty cloud");
            return;
        }

        // Preserve header information (frame_id, stamp) in PCL cloud so downstream modules can use it
        cloud->header.frame_id = msg->header.frame_id;

        // Store into latest_raw_cloud_ under mutex (atomic for shared_ptr refcount)
        {
            std::lock_guard<std::mutex> lk(latest_cloud_mutex_);
            latest_raw_cloud_ = cloud;
        }
    
    }
}//namespace robocup_vision_core


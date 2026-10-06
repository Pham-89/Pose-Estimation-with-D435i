#pragma once

/*
obb_visualizer.hpp

Convert OBB (Oriented Bounding Box) to RViz Marker for visualization.
Provides utilities to create visualization markers for OBB boxes.
*/

#include "obb_estimator.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robocup_vision_core {

class ObbVisualizer {
public:
    /**
     * Create a RViz Marker (box primitive) from an OBB.
     *
     * param obb         The oriented bounding box
     * param cluster_id  Unique ID for this marker (used for marker.id)
     * param frame_id    Frame ID for the marker (e.g., "camera_link")
     * param timestamp   ROS2 timestamp for the marker header
     * return            visualization_msgs::msg::Marker (box type)
     *
     * The marker will have:
     * - type = visualization_msgs::msg::Marker::CUBE
     * - position = OBB center
     * - orientation = OBB orientation (quaternion)
     * - scale = OBB extents (width, height, depth)
     * - color based on cluster_id (cycling through palette)
     */
    static visualization_msgs::msg::Marker createBoxMarker(
        const OBB &obb,
        size_t cluster_id,
        const std::string &frame_id,
        const rclcpp::Time &timestamp
    );
};

} // namespace robocup_vision_core

/*
obb_visualizer.cpp

Implementation of ObbVisualizer::createBoxMarker()
Converts an OBB (Oriented Bounding Box) into a RViz Marker (CUBE primitive)
for visualization in RViz2.

Key points:
- Marker type: visualization_msgs::msg::Marker::CUBE
- Position: OBB center (world coordinates)
- Orientation: OBB orientation (Eigen::Quaternionf converted to ROS geometry_msgs::msg::Quaternion)
- Scale: OBB extents (x, y, z lengths of the box)
- Color: RGB based on cluster_id (cycling through a color palette)
- Lifetime: 0 means the marker persists until a new one replaces it
*/

#include "obb_visualizer.hpp"
#include <cstdint>

namespace robocup_vision_core {

visualization_msgs::msg::Marker ObbVisualizer::createBoxMarker(
    const OBB &obb,
    size_t cluster_id,
    const std::string &frame_id,
    const rclcpp::Time &timestamp
)
{
    visualization_msgs::msg::Marker marker;
    
    // --- Header Information ---
    // frame_id: coordinate frame for this marker (e.g., "camera_link", "world")
    marker.header.frame_id = frame_id;
    // stamp: timestamp when marker was created (for RViz time synchronization)
    marker.header.stamp = timestamp;
    
    // --- Marker Identity ---
    // ns: namespace for grouping related markers (all OBB markers in "obb" namespace)
    marker.ns = "obb";
    // id: unique identifier within the namespace (cluster_id serves as unique ID)
    marker.id = static_cast<int32_t>(cluster_id);
    
    // --- Marker Type and Action ---
    // type: CUBE primitive (box shape)
    // visualization_msgs::msg::Marker::CUBE = 1
    marker.type = visualization_msgs::msg::Marker::CUBE;
    // action: ADD means add/update this marker (or replace if same id exists)
    // visualization_msgs::msg::Marker::ADD = 0
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // --- Position (Center of OBB in world coordinates) ---
    marker.pose.position.x = obb.position.x();
    marker.pose.position.y = obb.position.y();
    marker.pose.position.z = obb.position.z();
    
    // --- Orientation (Quaternion: rotation from box-local frame to world frame) ---
    // Convert Eigen::Quaternionf to ROS geometry_msgs::msg::Quaternion
    // Note: Eigen quaternion is (w, x, y, z) ordering
    marker.pose.orientation.w = obb.orientation.w();
    marker.pose.orientation.x = obb.orientation.x();
    marker.pose.orientation.y = obb.orientation.y();
    marker.pose.orientation.z = obb.orientation.z();
    
    // --- Scale (Extents of the Box) ---
    // RViz scale for a box is the full dimensions (width, height, depth)
    // obb.extents already stores full lengths, not half-lengths
    marker.scale.x = obb.extents.x();  // width along local x-axis
    marker.scale.y = obb.extents.y();  // depth along local y-axis
    marker.scale.z = obb.extents.z();  // height along local z-axis
    
    // --- Color (RGB + Alpha) ---
    // Define a color palette for different clusters
    // This helps distinguish between different OBBs in RViz
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> color_palette = {
        {255, 0, 0},      // Red (Cluster 0)
        {0, 255, 0},      // Green (Cluster 1)
        {0, 0, 255},      // Blue (Cluster 2)
        {255, 255, 0},    // Yellow (Cluster 3)
        {255, 0, 255},    // Magenta (Cluster 4)
        {0, 255, 255},    // Cyan (Cluster 5)
        {255, 128, 0},    // Orange (Cluster 6)
        {128, 0, 255},    // Purple (Cluster 7)
        {0, 255, 128},    // Spring Green (Cluster 8)
        {255, 0, 128},    // Pink (Cluster 9)
        {128, 255, 0},    // Lime (Cluster 10)
        {0, 128, 255},    // Sky Blue (Cluster 11)
        {128, 0, 0},      // Maroon (Cluster 12)
        {0, 128, 0},      // Dark Green (Cluster 13)
        {0, 0, 128}       // Navy (Cluster 14)
    };
    
    // Select color based on cluster_id (cycling through palette if more clusters than colors)
    auto color_rgb = color_palette[cluster_id % color_palette.size()];
    
    // Convert RGB (0-255) to ROS format (0.0-1.0)
    // ROS uses floating-point colors in range [0, 1]
    marker.color.r = static_cast<float>(std::get<0>(color_rgb)) / 255.0f;
    marker.color.g = static_cast<float>(std::get<1>(color_rgb)) / 255.0f;
    marker.color.b = static_cast<float>(std::get<2>(color_rgb)) / 255.0f;
    // alpha: transparency (1.0 = fully opaque, 0.0 = fully transparent)
    // Set to 0.7 for semi-transparent boxes so we can see through them and see other OBBs
    marker.color.a = 0.7f;
    
    // --- Lifetime ---
    // How long the marker persists before being auto-deleted by RViz
    // lifetime = 0 means the marker never auto-delete (until we publish a new one)
    // This is useful for persistent visualization
    marker.lifetime = rclcpp::Duration(0, 0);  // 0 seconds = persistent
    
    return marker;
}

} // namespace robocup_vision_core
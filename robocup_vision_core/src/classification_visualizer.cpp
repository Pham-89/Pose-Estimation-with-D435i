#include "classification_visualizer.hpp"

namespace robocup_vision_core {

// functions to convert ColorLabel and ShapeLabel to single-letter codes
namespace {
char colorLetter(ColorLabel c)
{
    switch (c) {
        case ColorLabel::RED:    return 'R';
        case ColorLabel::GREEN:  return 'G';
        case ColorLabel::BLUE:   return 'B';
        case ColorLabel::YELLOW: return 'Y';
        default:                 return '?';
    }
}

char shapeLetter(ShapeLabel s)
{
    switch (s) {
        case ShapeLabel::SQUARE:      return 'S';
        case ShapeLabel::RECTANGULAR: return 'R';
        default:                      return '?';
    }
}
} // namespace

// Builds a short 2-character code, e.g. "RS" (Red Square),
    // "BR" (Blue Rectangular). Uses '?' for the color or shape half
    // when that half was not classified (e.g. "R?" = Red, shape unknown).
std::string ClassificationVisualizer::abbreviate(const ClassificationResult &result)
{
    std::string code;
    code += colorLetter(result.color);
    code += shapeLetter(result.shape);
    return code; // e.g. "RS", "BR", "R?" (color known, shape unknown)
}

// Creates a text marker showing "<COLOR> <SHAPE>" (or a warning label
// if unclassified), floating text_offset_z meters above the OBB's top
// face (obb.position.z + obb.extents.z/2 + text_offset_z).
visualization_msgs::msg::Marker ClassificationVisualizer::createLabelMarker(
    const OBB &obb,
    const ClassificationResult &result,
    size_t cluster_id,
    const std::string &frame_id,
    const rclcpp::Time &timestamp,
    float text_offset_z,
    float text_size
)

{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = timestamp;

    // Separate namespace from "obb" boxes so RViz lets you toggle labels
    // on/off independently of the boxes.
    marker.ns = "classification_label";
    marker.id = static_cast<int32_t>(cluster_id);

    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;


    // Compute the world position for the label marker, floating above the OBB's top face.
    // The OBB's extents.z() is the block's height along the OBB's local z-axis,
    // which is aligned with the table plane normal (PlanarObbEstimator::n),
    // not necessarily the world/camera frame's Z axis. 
    // So we compute the label's world position 
    // by transforming a local offset along the OBB's local z-axis into world coordinates.      
    Eigen::Vector3f local_offset(0.0f, 0.0f, obb.extents.z() * 0.5f + text_offset_z);
    Eigen::Vector3f world_pos = obb.position + obb.orientation * local_offset;
    // Set the marker's position and orientation in world coordinates
    marker.pose.position.x = world_pos.x();
    marker.pose.position.y = world_pos.y();
    marker.pose.position.z = world_pos.z();
    marker.pose.orientation.w = 1.0; // TEXT_VIEW_FACING ignores orientation

    // Set the scale (height) of the text in meters
    marker.scale.z = text_size;

    // Short label per user request: "RS", "BR", "GS", "YR", etc.
    marker.text = abbreviate(result);

    // Set the color of the text marker based on classification status
    if (result.isFullyClassified()) {
        marker.color.r = 0.2f; marker.color.g = 1.0f; marker.color.b = 0.2f;
    } else {
        marker.color.r = 1.0f; marker.color.g = 0.4f; marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;// Set alpha to 1.0 (fully opaque)
    marker.lifetime = rclcpp::Duration(0, 0);// 0 means marker persists until deleted or replaced

    return marker;
}

} // namespace robocup_vision_core

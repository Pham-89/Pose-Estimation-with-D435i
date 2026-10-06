#pragma once
/*
classification_visualizer.hpp

Purpose
-------
Creates RViz TEXT_VIEW_FACING markers to display shape/color classification
results (from classifier.hpp) directly above each detected block, so you
can visually verify classification correctness in RViz instead of only
reading console logs. Complements ObbVisualizer::createBoxMarker() (the
box outline) with a text label floating above the box.

Kept separate from obb_visualizer.hpp/cpp intentionally: obb_visualizer only
depends on the OBB struct (geometry layer), while this file depends on
ClassificationResult (classification layer) — keeping them separate avoids
making the lower-level visualizer depend on the higher-level classifier.
*/

#include "obb_estimator.hpp"
#include "classifier.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robocup_vision_core {

class ClassificationVisualizer {
public:
    // Creates a text marker showing "<COLOR> <SHAPE>" (or a warning label
    // if unclassified), floating text_offset_z meters above the OBB's top
    // face (obb.position.z + obb.extents.z/2 + text_offset_z).
    static visualization_msgs::msg::Marker createLabelMarker(
        const OBB &obb,
        const ClassificationResult &result,
        size_t cluster_id,
        const std::string &frame_id,
        const rclcpp::Time &timestamp,
        float text_offset_z = 0.03f,
        float text_size = 0.02f
    );

    // Builds a short 2-character code, e.g. "RS" (Red Square),
    // "BR" (Blue Rectangular). Uses '?' for the color or shape half
    // when that half was not classified (e.g. "R?" = Red, shape unknown).
    static std::string abbreviate(const ClassificationResult &result);
};

} // namespace robocup_vision_core


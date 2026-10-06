#pragma once
/*
classifier.hpp

Purpose
-------
Combines shape classification (from OBB extents) and color classification
(from ColorUtils Hue statistics) into a single per-cluster classification
result. This class is pure computation with no ROS2 dependency, so it can
be reused as-is inside a future ROS2 service node without modification.

Shape classification
---------------------
Uses the ratio between the two in-plane OBB extents (extents.x / extents.y,
as produced by PlanarObbEstimator — x/y are the in-plane box dimensions,
z is block height). A square block (2x2cm) has ratio ~1, a rectangular
block (2x4cm) has ratio ~2, regardless of absolute scale errors from
voxelization/occlusion, since we only compare the two dimensions to each
other rather than to fixed absolute measurements.

Color classification
---------------------
Uses ColorUtils::computeHueStats() to get a single representative hue for
the cluster, then finds the nearest color centroid (nearest-centroid
classification on the hue circle, via ColorUtils::circularHueDistance()).
Default centroids are textbook HSV hue angles and are placeholders — they
should be recalibrated against the actual Lego blocks under your camera
and lighting using setColorCentroids() before relying on this in practice.
*/

#include "obb_estimator.hpp"
#include "color_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace robocup_vision_core {

enum class ShapeLabel {
    SQUARE,
    RECTANGULAR,
    UNKNOWN
};

enum class ColorLabel {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    UNKNOWN
};

std::string toString(ShapeLabel label);
std::string toString(ColorLabel label);

// One calibrated reference point on the hue circle for a given color label.
struct ColorCentroid {
    ColorLabel label;
    float hue_deg; // [0, 360)
};

// Full classification result for a single cluster.
struct ClassificationResult {
    ShapeLabel shape = ShapeLabel::UNKNOWN;
    ColorLabel color = ColorLabel::UNKNOWN;

    // --- Debug / diagnostic fields (useful for logging & tuning) ---
    float shape_ratio = 0.f;          // long_side / short_side of in-plane extents
    HueStats hue_stats;               // raw hue stats used for color decision
    float color_match_distance = 0.f; // circular hue distance to the chosen centroid
                                       // (only meaningful if color != UNKNOWN)

    bool isFullyClassified() const {
        return shape != ShapeLabel::UNKNOWN && color != ColorLabel::UNKNOWN;
    }
};

class Classifier {
public:
    using PointT = pcl::PointXYZRGB;
    using Cloud = pcl::PointCloud<PointT>;

    // Constructs a classifier with default (uncalibrated) color centroids.
    // Call setColorCentroids() afterwards with values measured from your
    // own setup for reliable results.
    Classifier();

    // --- Shape classification ---
    // ratio_threshold: shape_ratio below this -> SQUARE, at/above -> RECTANGULAR.
    //   Default 1.4 sits roughly at the geometric mean of the expected
    //   ratios (1.0 for square, 2.0 for rectangular: sqrt(1*2) ~= 1.41),
    //   giving roughly equal margin against noise on both sides.
    // min_extent_m: extents (in meters) below this are treated as
    //   degenerate/unreliable clusters -> UNKNOWN, to guard against noise
    //   clusters that slipped through DBSCAN's min_cluster_size filter.
    ShapeLabel classifyShape(const OBB &obb,
                              float ratio_threshold = 1.4f,
                              float min_extent_m = 0.005f,
                              float *out_ratio = nullptr) const;

    // --- Color classification ---
    // Returns ColorLabel::UNKNOWN if:
    //   - HueStats is invalid (not enough reliable pixels), or
    //   - the nearest centroid is farther than max_match_distance_deg away
    //     (i.e. the measured hue doesn't confidently match any known color).
    ColorLabel classifyColor(const HueStats &stats,
                              float max_match_distance_deg = 25.0f,
                              float *out_distance = nullptr) const;

    // --- Combined convenience API ---
    // Runs ColorUtils::computeHueStats() internally on cluster_cloud, then
    // classifies both shape and color. This is the main entry point you'll
    // call per-cluster from main.cpp (or later, from the service node).
    ClassificationResult classify(const Cloud::ConstPtr &cluster_cloud,
                                   const OBB &obb) const;

    // Replace the default centroids with calibrated ones. Must provide
    // exactly one centroid per color you want to support; colors omitted
    // from the list will never be matched (classifyColor will skip them).
    void setColorCentroids(const std::vector<ColorCentroid> &centroids);
    const std::vector<ColorCentroid> &getColorCentroids() const { return centroids_; }

    // Set the maximum allowed circular hue distance (in degrees) for a cluster to
    // be considered a match to any known color centroid. Clusters with a
    // measured hue farther than this from the nearest centroid will be classified
    // as ColorLabel::UNKNOWN. Default is 18 degrees, which is a tighter threshold
    // than the previous 25 degrees, based on real calibration data (Red-Yellow gap is only ~37 deg).
    void setMaxColorMatchDistance(float degrees) { max_color_match_distance_deg_ = degrees; }


private:
    std::vector<ColorCentroid> centroids_;

    // tightened from 25 based on real calibration data:Red-Yellow gap is only ~37 deg
    float max_color_match_distance_deg_ = 18.0f;
};

} // namespace robocup_vision_core
#include "classifier.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robocup_vision_core {

std::string toString(ShapeLabel label)
{
    switch (label) {
        case ShapeLabel::SQUARE:      return "SQUARE";
        case ShapeLabel::RECTANGULAR: return "RECTANGULAR";
        default:                      return "UNKNOWN";
    }
}

std::string toString(ColorLabel label)
{
    switch (label) {
        case ColorLabel::RED:    return "RED";
        case ColorLabel::GREEN:  return "GREEN";
        case ColorLabel::BLUE:   return "BLUE";
        case ColorLabel::YELLOW: return "YELLOW";
        default:                 return "UNKNOWN";
    }
}

Classifier::Classifier()
{
    // Placeholder centroids using textbook hue angles. These are almost
    // certainly NOT accurate for your actual Lego blocks under the D435i
    // color sensor + your lighting — recalibrate with setColorCentroids()
    // using real measured median hues before trusting classification.
    centroids_ = {
        { ColorLabel::RED,    0.0f   },
        { ColorLabel::YELLOW, 60.0f  },
        { ColorLabel::GREEN,  120.0f },
        { ColorLabel::BLUE,   240.0f },
    };
}

void Classifier::setColorCentroids(const std::vector<ColorCentroid> &centroids)
{
    centroids_ = centroids;
}

ShapeLabel Classifier::classifyShape(const OBB &obb,
                                      float ratio_threshold,
                                      float min_extent_m,
                                      float *out_ratio) const
{
    // extents.x / extents.y are the in-plane dimensions produced by
    // PlanarObbEstimator (extents.z is block height along the plane normal).
    float a = obb.extents.x();
    float b = obb.extents.y();

    float short_side = std::min(a, b);
    float long_side = std::max(a, b);

    if (short_side < min_extent_m) {
        // Degenerate box (near-zero width) — not a trustworthy measurement.
        if (out_ratio) *out_ratio = 0.f;
        return ShapeLabel::UNKNOWN;
    }

    float ratio = long_side / short_side;
    if (out_ratio) *out_ratio = ratio;

    return (ratio < ratio_threshold) ? ShapeLabel::SQUARE : ShapeLabel::RECTANGULAR;
}

ColorLabel Classifier::classifyColor(const HueStats &stats,
                                      float max_match_distance_deg,
                                      float *out_distance) const
{
    if (!stats.valid || centroids_.empty()) {
        if (out_distance) *out_distance = std::numeric_limits<float>::infinity();
        return ColorLabel::UNKNOWN;
    }

    ColorLabel best_label = ColorLabel::UNKNOWN;
    float best_distance = std::numeric_limits<float>::infinity();

    for (const auto &centroid : centroids_) {
        float d = ColorUtils::circularHueDistance(stats.median_hue_deg, centroid.hue_deg);
        if (d < best_distance) {
            best_distance = d;
            best_label = centroid.label;
        }
    }

    if (out_distance) *out_distance = best_distance;

    if (best_distance > max_match_distance_deg) {
        // Nearest centroid still too far away to trust — reject rather
        // than guess, since this result feeds grasping target selection.
        return ColorLabel::UNKNOWN;
    }

    return best_label;
}

ClassificationResult Classifier::classify(const Cloud::ConstPtr &cluster_cloud,
                                           const OBB &obb) const
{      
    ClassificationResult result;

    float ratio = 0.f;
    result.shape = classifyShape(obb, 1.4f, 0.005f, &ratio);
    result.shape_ratio = ratio;

    result.hue_stats = ColorUtils::computeHueStats(cluster_cloud);

    float distance = 0.f;
    result.color = classifyColor(result.hue_stats, max_color_match_distance_deg_, &distance);
    result.color_match_distance = distance;

    return result;
}

} // namespace robocup_vision_core
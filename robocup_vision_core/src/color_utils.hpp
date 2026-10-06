#pragma once
/*
color_utils.hpp

Purpose
-------
Low-level color computation utilities operating on PCL PointXYZRGB clusters.
This file intentionally contains NO classification logic (no color labels,
no thresholds, no calibration data) — it only converts raw RGB cluster data
into a robust, lighting-invariant representative Hue value plus supporting
statistics. The upcoming `classifier` module will consume these outputs
(e.g. via nearest-centroid matching using circularHueDistance()) to decide
the actual color label.

Why Hue instead of raw RGB
---------------------------
Lighting changes and camera auto white-balance scale R/G/B roughly
proportionally, which shifts a Lego block's exact RGB reading between
frames/sessions (e.g. (255,11,25) vs (200,5,15) for the "same" red).
Hue is largely invariant to this because it only depends on the relative
ratio between R, G, B, not their absolute magnitude. Saturation and Value
are used only to *filter out* unreliable pixels (near-white, near-black,
or over-exposed/specular highlight points) before estimating Hue.
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <vector>

namespace robocup_vision_core {

// Single pixel color in HSV space.
// hue_deg: [0, 360), 0 = red. NaN if undefined (achromatic, r=g=b).
// sat: [0, 1]
// val: [0, 1]  (max(r,g,b) normalized)
struct HSVPixel {
    float hue_deg;
    float sat;
    float val;
};

// Aggregated Hue/Saturation/Value statistics for an entire cluster,
// computed only from pixels considered "reliable" (see computeHueStats).
struct HueStats {
    bool valid = false;         // false if not enough reliable pixels were found
    float median_hue_deg = 0.f; // representative hue for the whole cluster
    float mean_saturation = 0.f;
    float mean_value = 0.f;
    size_t total_points = 0;    // points in the input cluster
    size_t used_points = 0;     // points that passed the S/V reliability filter
};


class ColorUtils {
public:
    using PointT = pcl::PointXYZRGB;
    using Cloud = pcl::PointCloud<PointT>;

    // Convert a single RGB triplet (0-255 each) to HSV.
    // hue_deg is NaN when the pixel is achromatic (r==g==b), since Hue is
    // undefined in that case; callers should treat NaN as "unreliable".
    static HSVPixel rgbToHsv(uint8_t r, uint8_t g, uint8_t b);

    // Compute representative Hue statistics for a cluster.
    //
    // sat_min:   minimum saturation [0,1] for a pixel to be considered
    //            reliable (rejects near-gray/white/black pixels whose hue
    //            is noisy or undefined). Typical: 0.25 - 0.35.
    // val_min:   minimum value [0,1] to reject near-black/shadowed pixels.
    //            Typical: 0.15 - 0.25.
    // val_max:   maximum value [0,1] to reject over-exposed/specular
    //            highlight pixels (common on glossy Lego plastic).
    //            Typical: 0.95 - 0.98.
    // min_valid_points: minimum number of reliable pixels required to trust
    //            the result; if fewer, HueStats.valid is set to false.
    //
    // The returned hue is the MEDIAN of reliable pixels' hues, computed on
    // the circle (see circularMedianHue) rather than a plain arithmetic
    // mean, since Hue is a cyclic quantity and a linear mean is invalid
    // near the 0/360 wrap-around (e.g. mixing 350 deg and 10 deg should
    // give ~0 deg, not ~180 deg).
    static HueStats computeHueStats(const Cloud::ConstPtr &cluster_cloud,
                                     float sat_min = 0.30f,
                                     float val_min = 0.20f,
                                     float val_max = 0.97f,
                                     size_t min_valid_points = 20);

    // Shortest angular distance between two hues on the circle, in degrees,
    // always in [0, 180]. E.g. circularHueDistance(350, 10) == 20, not 340.
    static float circularHueDistance(float hue_a_deg, float hue_b_deg);

    // Circular median of a set of hues (degrees). Used internally by
    // computeHueStats but exposed in case classifier.cpp needs it directly
    // (e.g. to merge stats from multiple observations of the same block).
    static float circularMedianHue(std::vector<float> hues_deg);
};

} // namespace robocup_vision_core
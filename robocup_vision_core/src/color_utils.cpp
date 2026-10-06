#include "color_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robocup_vision_core {

HSVPixel ColorUtils::rgbToHsv(uint8_t r_in, uint8_t g_in, uint8_t b_in)
{
    float r = r_in / 255.0f;
    float g = g_in / 255.0f;
    float b = b_in / 255.0f;

    float max_c = std::max({r, g, b});
    float min_c = std::min({r, g, b});
    float delta = max_c - min_c;

    HSVPixel out;
    out.val = max_c;
    out.sat = (max_c <= 1e-6f) ? 0.f : (delta / max_c);

    if (delta <= 1e-6f) {
        // r == g == b: achromatic pixel, hue undefined.
        out.hue_deg = std::numeric_limits<float>::quiet_NaN();
        return out;
    }

    float h;
    if (max_c == r) {
        h = 60.0f * std::fmod(((g - b) / delta), 6.0f);
    } else if (max_c == g) {
        h = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        h = 60.0f * (((r - g) / delta) + 4.0f);
    }
    if (h < 0.0f) h += 360.0f;

    out.hue_deg = h;
    return out;
}

float ColorUtils::circularHueDistance(float hue_a_deg, float hue_b_deg)
{
    float diff = std::fmod(std::fabs(hue_a_deg - hue_b_deg), 360.0f);
    return (diff > 180.0f) ? (360.0f - diff) : diff;
}

float ColorUtils::circularMedianHue(std::vector<float> hues_deg)
{
    if (hues_deg.empty()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    if (hues_deg.size() == 1) {
        return hues_deg.front();
    }

    // Circular median approximation:
    // 1) Convert each hue to a unit vector (cos, sin) to avoid wrap-around
    //    issues, average the vectors to get a reference direction.
    // 2) Re-express every hue as its signed angular offset from that
    //    reference direction (now a normal, non-cyclic linear quantity in
    //    [-180, 180]).
    // 3) Take the plain linear median of the offsets and add it back to
    //    the reference direction.
    // This keeps values that straddle the 0/360 boundary (e.g. 350 and 10)
    // from being wrongly treated as far apart.
    double sum_cos = 0.0, sum_sin = 0.0;
    for (float h : hues_deg) {
        double rad = h * M_PI / 180.0;
        sum_cos += std::cos(rad);
        sum_sin += std::sin(rad);
    }
    double ref_rad = std::atan2(sum_sin, sum_cos);
    float ref_deg = static_cast<float>(ref_rad * 180.0 / M_PI);
    if (ref_deg < 0.0f) ref_deg += 360.0f;

    std::vector<float> offsets;
    offsets.reserve(hues_deg.size());
    for (float h : hues_deg) {
        float diff = std::fmod(h - ref_deg + 540.0f, 360.0f) - 180.0f; // in [-180,180)
        offsets.push_back(diff);
    }

    std::sort(offsets.begin(), offsets.end());
    float median_offset;
    size_t n = offsets.size();
    if (n % 2 == 1) {
        median_offset = offsets[n / 2];
    } else {
        median_offset = 0.5f * (offsets[n / 2 - 1] + offsets[n / 2]);
    }

    float result = std::fmod(ref_deg + median_offset + 360.0f, 360.0f);
    return result;
}

HueStats ColorUtils::computeHueStats(const Cloud::ConstPtr &cluster_cloud,
                                      float sat_min,
                                      float val_min,
                                      float val_max,
                                      size_t min_valid_points)
{
    HueStats stats;
    if (!cluster_cloud || cluster_cloud->empty()) {
        return stats;
    }
    stats.total_points = cluster_cloud->size();

    std::vector<float> reliable_hues;
    reliable_hues.reserve(cluster_cloud->size());
    double sat_sum = 0.0, val_sum = 0.0;

    for (const auto &p : cluster_cloud->points) {
        HSVPixel hsv = rgbToHsv(p.r, p.g, p.b);

        if (std::isnan(hsv.hue_deg)) continue;                 // achromatic
        if (hsv.sat < sat_min) continue;                       // too washed out
        if (hsv.val < val_min || hsv.val > val_max) continue;  // too dark / blown out

        reliable_hues.push_back(hsv.hue_deg);
        sat_sum += hsv.sat;
        val_sum += hsv.val;
    }

    stats.used_points = reliable_hues.size();

    if (stats.used_points < min_valid_points) {
        stats.valid = false;
        return stats;
    }

    stats.median_hue_deg = circularMedianHue(reliable_hues);
    stats.mean_saturation = static_cast<float>(sat_sum / stats.used_points);
    stats.mean_value = static_cast<float>(val_sum / stats.used_points);
    stats.valid = true;

    return stats;
}

} // namespace robocup_vision_core
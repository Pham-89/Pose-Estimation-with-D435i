#pragma once
/*
block_registry.hpp

Purpose
-------
Thread-safe storage of the most recently classified pose for each block
label (e.g. "RS", "BR"). The main perception loop overwrites entries every
frame; the grasp-pose service reads from here on demand, decoupling the
request-response service call from the perception pipeline's own timing.

Staleness handling: since the scene is assumed static (blocks don't move
during a RoboCup run), we don't need complex tracking — just overwrite on
every frame. But if the block disappears from view (occluded, misclassified
away), lookup() will return the last-seen pose unless it's older than
max_age — this guards against returning a long-stale pose for a block
that's no longer actually visible.
*/

#include "obb_estimator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <optional>

namespace robocup_vision_core {

struct BlockEntry {
    OBB obb;
    rclcpp::Time stamp;
};

class BlockRegistry {
public:
    // Overwrite (or insert) the latest observation for this label.
    void update(const std::string &label, const OBB &obb, const rclcpp::Time &stamp);

    // Clear all entries (call at the start of a frame, before repopulating,
    // so blocks that disappeared this frame don't linger with a stale pose
    // beyond max_age anyway — but this makes "disappeared" detectable
    // immediately rather than after max_age elapses).
    void clear();

    // Look up the latest pose for a label. Returns std::nullopt if the
    // label was never seen, or if its last observation is older than
    // max_age relative to `now` (i.e. not currently visible/trustworthy).
    std::optional<BlockEntry> lookup(const std::string &label,
                                      const rclcpp::Time &now,
                                      double max_age_sec = 0.5) const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, BlockEntry> entries_;
};

} // namespace robocup_vision_core
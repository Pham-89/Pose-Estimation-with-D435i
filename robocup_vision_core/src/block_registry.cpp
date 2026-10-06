#include "block_registry.hpp"

namespace robocup_vision_core {

void BlockRegistry::update(const std::string &label, const OBB &obb, const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[label] = BlockEntry{obb, stamp};
}

void BlockRegistry::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
}

std::optional<BlockEntry> BlockRegistry::lookup(const std::string &label,
                                                 const rclcpp::Time &now,
                                                 double max_age_sec) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = entries_.find(label);
    if (it == entries_.end()) {
        return std::nullopt;
    }
    double age = (now - it->second.stamp).seconds();
    if (age > max_age_sec) {
        return std::nullopt;
    }
    return it->second;
}

} // namespace robocup_vision_core
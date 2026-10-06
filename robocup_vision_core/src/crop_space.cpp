// crop_space.cpp
// Implementation of CropSpace using pcl::CropBox.

#include "crop_space.hpp"
#include <pcl/filters/crop_box.h>

namespace robocup_vision_core
{

CropSpace::CropSpace(const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt)
: min_pt_(min_pt), max_pt_(max_pt){}

void CropSpace::setMin(const Eigen::Vector4f &min_pt) { min_pt_ = min_pt; }
void CropSpace::setMax(const Eigen::Vector4f &max_pt) { max_pt_ = max_pt; }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CropSpace::crop(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in) const
{
    if (!in || in->empty()) {
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    pcl::CropBox<pcl::PointXYZRGB> box;
    box.setInputCloud(in);
    box.setMin(min_pt_);
    box.setMax(max_pt_);

    // Use PCL's Ptr (boost::shared_ptr) for compatibility
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
    box.filter(*out);

    // Preserve header/frame
    out->header = in->header;
    return out;
}

} // namespace robocup_vision_core

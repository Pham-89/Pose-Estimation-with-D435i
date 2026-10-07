#pragma once
// crop_space.hpp
// Crop a PCL cloud to a configurable 3D box (CropBox). Stateless helper class.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace robocup_vision_core
{

class CropSpace {
public:
    // Default constructor uses example bounds (matches your previous values)
    CropSpace(
        const Eigen::Vector4f &min_pt = Eigen::Vector4f(-0.3f, -0.5f, 0.1f, 1.0f),
        const Eigen::Vector4f &max_pt = Eigen::Vector4f( 0.3f,  0.5f, 5.0f, 1.0f)
    );

    // Setters
    void setMin(const Eigen::Vector4f &min_pt);
    void setMax(const Eigen::Vector4f &max_pt);

    // Crop input cloud (ConstPtr) and return a new cloud Ptr (header is copied).
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr crop(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in) const;

private:
    Eigen::Vector4f min_pt_;
    Eigen::Vector4f max_pt_;
};

} // namespace robocup_vision_core

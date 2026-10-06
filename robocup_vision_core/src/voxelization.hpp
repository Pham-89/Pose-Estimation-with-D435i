#pragma once
// voxelization.hpp
// Voxel downsampling helper (pcl::VoxelGrid). Stateless with configurable leaf size.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace robocup_vision_core
{

class Voxelizer {
public:
    Voxelizer(float leaf_x = 0.003f, float leaf_y = 0.003f, float leaf_z = 0.003f);

    void setLeafSize(float lx, float ly, float lz);
    void getLeafSize(float &lx, float &ly, float &lz) const;

    // Voxelize input cloud and return a new cloud (header copied)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in) const;

private:
    float leaf_x_, leaf_y_, leaf_z_;
};

} // namespace robocup_vision_core
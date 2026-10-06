// voxelization.cpp
// Implementation of Voxelizer using pcl::VoxelGrid.

#include "voxelization.hpp"
#include <pcl/filters/voxel_grid.h>

namespace robocup_vision_core
{

Voxelizer::Voxelizer(float lx, float ly, float lz)
: leaf_x_(lx), leaf_y_(ly), leaf_z_(lz){}

void Voxelizer::setLeafSize(float lx, float ly, float lz) {
    leaf_x_ = lx; leaf_y_ = ly; leaf_z_ = lz;}

void Voxelizer::getLeafSize(float &lx, float &ly, float &lz) const {
    lx = leaf_x_; ly = leaf_y_; lz = leaf_z_;}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Voxelizer::voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in) const
{
    if (!in || in->empty()) {
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf_x_, leaf_y_, leaf_z_);

    // Use PCL Ptr (boost::shared_ptr) for compatibility
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
    vg.filter(*out);

    // preserve header/frame
    out->header = in->header;
    return out;
}

} // namespace robocup_vision_core

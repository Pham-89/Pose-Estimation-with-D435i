#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class RansacPlane {
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_plane(// static function
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    
    //returns the plane model [a,b,c,d] (ax+by+cz+d=0) — needed by PlanarObbEstimator.
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_plane(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::ModelCoefficients::Ptr &out_coefficients);
};

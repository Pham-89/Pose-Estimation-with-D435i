#include "ransac_plane.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*
 * remove_plane(cloud, out_coefficients)
 * Input:  PointCloud containing the full scene
 * Output: PointCloud with the dominant plane removed (e.g., table surface)
 *         out_coefficients: filled with the plane model [a,b,c,d]
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RansacPlane::remove_plane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::ModelCoefficients::Ptr &out_coefficients)
{
    // Validate input
    if (!cloud || cloud->empty()) {
        return cloud;
    }

    // Containers for the plane model coefficients and the inlier indices
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // --- RANSAC plane segmentation ---
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setProbability(0.99);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // store the coefficients for use by PlanarObbEstimator
    out_coefficients = coefficients;

    // If no plane was found, return the original cloud
    if (inliers->indices.empty()) {
        return cloud;
    }

    // --- Extract points that are NOT part of the plane ---
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.filter(*objects);

    try {
        objects->header = cloud->header;
    } catch (...) {
        // ignore if not available
    }

    return objects;
}

// Backward-compatible overload: same as above, but discards the plane
// coefficients for callers that don't need them.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr RansacPlane::remove_plane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr dummy;
    return remove_plane(cloud, dummy);
}

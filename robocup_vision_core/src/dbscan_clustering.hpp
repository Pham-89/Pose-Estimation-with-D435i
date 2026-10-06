#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>

class DBSCANClustering {
public:
    /**
     * Run DBSCAN on an input point cloud (PointXYZRGB).
     *
     * param cloud         Input point cloud (shared pointer). Must be non-null and non-empty.
     * param eps           Neighborhood radius in meters (e.g., 0.02 for 2 cm).
     * param min_samples   Minimum number of points in eps-neighborhood to form a core point.
     * return              Vector of pcl::PointIndices, each element is one cluster.
     *
     * Notes:
     * - Points labeled as noise are not returned in any cluster.
     * - Complexity depends on radius search performance (KD-tree).
     */
    static std::vector<pcl::PointIndices> dbscan_indices(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,//shared pointer "cloud" refer to invariant cloud
        double eps = 0.03,
        int min_samples = 50
    );
};


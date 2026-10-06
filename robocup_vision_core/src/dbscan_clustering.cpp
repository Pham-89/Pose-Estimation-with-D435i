#include "dbscan_clustering.hpp"
#include <pcl/search/kdtree.h>
#include <queue>
#include <limits>

/*
Implementation notes (high level):
- We follow the DBSCAN algorithm:
  1) For each unvisited point p:
     - mark visited
     - find neighbors within radius eps
     - if neighbors.size() < min_samples -> mark as noise
     - else create new cluster and expand:
         - for each neighbor q:
             - if q unvisited: mark visited, find q's neighbors; if q is core (>=min_samples) add its neighbors to expansion queue
             - if q not yet assigned to any cluster: assign to current cluster
- We use pcl::search::KdTree for efficient radius queries.
- Labels:
    - -1 : unvisited
    - -2 : noise
    - >=0 : cluster id
*/

std::vector<pcl::PointIndices> DBSCANClustering::dbscan_indices(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    double eps,
    int min_samples)
{
    //vector "clusters" store the cluster results 0, 1, 2,...
    //each cluster is a vector of indices of points in the input cloud 
    std::vector<pcl::PointIndices> clusters;

    // Basic checks
    if (!cloud || cloud->empty()) {
        return clusters; // empty result
    }
    if (eps <= 0.0) {
        return clusters;
    }
    if (min_samples <= 0) {
        return clusters;
    }

    const int N = static_cast<int>(cloud->size()); //static_cast number of cloud points to int & assign to N

    //Labels for clusters: -1 = unvisited, -2 = noise, >=0 cluster id
    std::vector<int> labels(N, -1);// vector of N elements with initial value -1 (unvisited)

    // KD-tree for radius search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<int> nn_indices; //vector of indices of neighbors found in radius search
    std::vector<float> nn_distances;//vector of distances of neighbors found in radius search

    int cluster_id = 0;

    for (int i = 0; i < N; ++i) {
        if (labels[i] != -1) {
            // already processed (visited)
            continue;
        }

        // Mark as visited (temporarily use label -3 to indicate visited but not assigned)
        // But to keep simple, we will treat visited by checking labels != -1.
        // Find neighbors within eps
        nn_indices.clear();
        nn_distances.clear();
        // radiusSearch returns number of neighbors found
        int n_found = tree->radiusSearch(i, eps, nn_indices, nn_distances);

        if (n_found < min_samples) {
            // Mark as noise
            labels[i] = -2;
            continue;
        }

        // Otherwise, create new cluster and expand
        pcl::PointIndices cluster;
        std::queue<int> expansion_queue;

        // Add seed point and its neighbors to queue
        labels[i] = cluster_id;//assign point #i to cluster_id
        cluster.indices.push_back(i);

        // Add neighbors to expansion queue (after kdtree search)
        for (int idx : nn_indices) {
            if (idx == i) continue; // skip self if present
            expansion_queue.push(idx);
        }
        
        // Expand cluster
        while (!expansion_queue.empty()) {
        	int idx = expansion_queue.front();
                expansion_queue.pop();

                if (labels[idx] == -2) {
                    // previously marked as noise -> becomes border point of this cluster
                    labels[idx] = cluster_id;
                    cluster.indices.push_back(idx);
                    continue;
                }

                if (labels[idx] != -1 && labels[idx] != -3) {
                    // already assigned to a cluster (or already processed as pushed)
                    continue;
                }

                // If we used label -3 as 'in-queue but not yet assigned', treat it appropriately
                // If it's still -1, set to cluster.
                labels[idx] = cluster_id;
                cluster.indices.push_back(idx);

                // Check if this point is a core point (has enough neighbors)
                nn_indices.clear();
                nn_distances.clear();
                int n_found2 = tree->radiusSearch(idx, eps, nn_indices, nn_distances);

                if (n_found2 >= min_samples) {
                    // If core, add its neighbors to expansion queue
                    for (int nidx : nn_indices) {
                        if (labels[nidx] == -1) {
                            expansion_queue.push(nidx);
                            // Mark as "in queue" to avoid multiple pushes (optional)
                            labels[nidx] = -3;
                        } else if (labels[nidx] == -2) {
                            // previously marked as noise -> convert to cluster border point now
                            labels[nidx] = cluster_id;
                            cluster.indices.push_back(nidx);
                        }
                    }
                }
            } // end expand

        // Save cluster
        clusters.push_back(cluster);
        ++cluster_id;
    } // end for each point

    return clusters;
}


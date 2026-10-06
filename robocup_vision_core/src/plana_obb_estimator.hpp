#pragma once
/*
planar_obb_estimator.hpp

Purpose
-------
Fixes the "flip-flopping OBB for near-square objects" problem that occurs
with pcl::MomentOfInertiaEstimation (PCA-based OBB). When two eigenvalues of
the covariance matrix are nearly equal (e.g. a 2x2cm square Lego block seen
from above), the PCA eigenvectors become numerically ill-defined and can
rotate ~45 deg between frames due to small amounts of sensor noise.

This estimator avoids PCA entirely for the in-plane (X/Y) orientation:
  1. Project the cluster's 3D points onto the supporting plane (the table,
     from RANSAC) using the plane normal -> 2D points in a stable
     plane-local (u, v) basis.
  2. Compute the 2D convex hull of the projected points.
  3. Run rotating calipers to find the minimum-area bounding rectangle of
     the hull. This is anchored to the *physical edges* of the block, not
     to an eigen-decomposition, so there is no degenerate/non-unique
     solution for square footprints.
  4. Combine the in-plane rectangle with the extent along the plane normal
     (block height) to build a full 3D OBB, using the same OBB struct as
     obb_estimator.hpp.

Use this instead of ObbEstimator::computeOBBFromMoments() whenever you need
a stable orientation for near-square (or exactly square) footprints.
*/

#include "obb_estimator.hpp"  // reuse the OBB struct
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace robocup_vision_core {

class PlanarObbEstimator {
public:
    using PointT = pcl::PointXYZRGB;
    using Cloud = pcl::PointCloud<PointT>;

    // plane_normal: normal vector of the supporting plane (table), same
    //   frame as the cloud. Does not need to be pre-normalized.
    // plane_point: any point lying on that plane (e.g. projection of the
    //   world origin onto the plane, or centroid of the RANSAC inliers).
    static OBB computeOBB(const Cloud::ConstPtr &cloud,
                           const Eigen::Vector3f &plane_normal,
                           const Eigen::Vector3f &plane_point);

    // Convenience overload: build normal/point directly from PCL plane
    // coefficients [a, b, c, d] as produced by pcl::SACSegmentation
    // (ax + by + cz + d = 0).
    static OBB computeOBB(const Cloud::ConstPtr &cloud,
                           const Eigen::Vector4f &plane_coefficients);

private:
    struct Point2D { float x; float y; };

    static float cross2D(const Point2D &O, const Point2D &A, const Point2D &B);
    static std::vector<Point2D> convexHull2D(std::vector<Point2D> pts);
};

} // namespace robocup_vision_core
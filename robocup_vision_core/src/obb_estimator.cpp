/*
** obb_estimator.cpp

** Implementation of OBB computation using PCL's MomentOfInertiaEstimation.
** This file implements ObbEstimator::computeOBBFromMoments and the OBB::corners()
   helper. The implementation uses only PCL moment-of-inertia APIs as requested.
** Important implementation notes and nuances (explained inline):
	- MomentOfInertiaEstimation computes a set of features (moment of inertia,
	eccentricity, etc.) and exposes getOBB(), which returns:
	min_point_OBB (Eigen::Vector4f)  - minimum corner of box in local box frame
	max_point_OBB (Eigen::Vector4f)  - maximum corner of box in local box frame
	position_OBB (Eigen::Vector3f)   - center of the box in world coords
	 rotational_matrix_OBB (Eigen::Matrix3f) - rotation matrix mapping box-local -> world
	 - We convert those values into an OBB consisting of:
	 position (center), orientation (quaternion), extents (max - min).
** Notes:
	- PCL's getOBB uses the point distribution (moments) and is often very close
	to PCA-based OBB; it's robust and convenient.
	- extents are positive by construction (we use abs to be defensive).
	- The quaternion is built from the rotation matrix returned by PCL.
*/

#include "obb_estimator.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>

namespace robocup_vision_core {

std::array<Eigen::Vector3f, 8> OBB::corners() const
{
    // Compute half-extents (local frame)
    Eigen::Vector3f h = extents * 0.5f;

    // Define the eight corners in local box coordinates.
    // The order is arbitrary, but typical ordering is the 8 combinations of signs.
    std::array<Eigen::Vector3f, 8> local = {
        Eigen::Vector3f( h.x(),  h.y(),  h.z()),
        Eigen::Vector3f(-h.x(),  h.y(),  h.z()),
        Eigen::Vector3f(-h.x(), -h.y(),  h.z()),
        Eigen::Vector3f( h.x(), -h.y(),  h.z()),
        Eigen::Vector3f( h.x(),  h.y(), -h.z()),
        Eigen::Vector3f(-h.x(),  h.y(), -h.z()),
        Eigen::Vector3f(-h.x(), -h.y(), -h.z()),
        Eigen::Vector3f( h.x(), -h.y(), -h.z())
    };

    // Transform local corners to world: world = orientation * local + position
    std::array<Eigen::Vector3f, 8> world;
    for (size_t i = 0; i < local.size(); ++i) {
        world[i] = orientation * local[i] + position;
    }
    return world;
}

OBB ObbEstimator::computeOBBFromMoments(const Cloud::ConstPtr &cloud)
{

    OBB obb;
    obb.position.setZero();
    obb.orientation = Eigen::Quaternionf::Identity();
    obb.extents.setZero();

    // Defensive checks
    if (!cloud || cloud->empty()) {
        // Return zero box for empty input
        return obb;
    }

    // PCL MomentOfInertiaEstimation (expects PointT types for min/max/position outputs)
    //computes moments of inertia and offers getOBB() which returns the oriented bounding box parameters.
    pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    // compute() performs all internal calculations; this must be called before getOBB()
    feature_extractor.compute();

    
    // PCL's getOBB signature expects PointT references for min/max/position and an Eigen::Matrix3f for rotation.
    // bool getOBB (PointT &min_point, PointT &max_point, PointT &position, Eigen::Matrix3f &rotational_matrix) const;
    PointT min_pt_OBB, max_pt_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    // Retrieve OBB parameters from PCL
    // Note: getOBB fills min_pt_OBB, max_pt_OBB (local frame), position_OBB (world), and rotation matrix.
    feature_extractor.getOBB(min_pt_OBB, max_pt_OBB, position_OBB, rotational_matrix_OBB);

    // Convert min/max PCL PointT -> Eigen vectors
    Eigen::Vector3f min_pt(min_pt_OBB.x, min_pt_OBB.y, min_pt_OBB.z);
    Eigen::Vector3f max_pt(max_pt_OBB.x, max_pt_OBB.y, max_pt_OBB.z);
    // Convert center PCL PointT -> Eigen vector (center of box)
    Eigen::Vector3f center(position_OBB.x, position_OBB.y, position_OBB.z);

    // extents (full lengths) are max - min in the box-local frame
    Eigen::Vector3f extents_local = (max_pt - min_pt).cwiseAbs();

    // Fill the OBB struct
    obb.position = center;
    obb.orientation = Eigen::Quaternionf(rotational_matrix_OBB);
    obb.extents = extents_local;

    // Done
    return obb;
}

         
} // namespace robocup_vision_core

#pragma once
/*
obb_estimator.hpp

** Compute an oriented bounding box (OBB) for a PCL point cloud using
** PCL's MomentOfInertiaEstimation (moment / inertia-based OBB).
** This header defines:
	- struct OBB: holds center position, orientation (quaternion), extents (full lengths) 
	and helper to compute the 8 corner points.
	
	- class ObbEstimator: single static public API function computeOBBFromMoments()
	which uses only PCL's MomentOfInertiaEstimation.
** Notes:
	- This implementation intentionally uses only PCL's moment-of-inertia API
	- The returned extents are full-lengths along the box local x/y/z axes.
	- Corners() returns the eight corners in world coordinates (orientation applied).
*/

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace robocup_vision_core {

/*
 * OBB - Oriented Bounding Box representation
 *
 * position: center of the box in world coordinates (x,y,z)
 * orientation: rotation that transforms local box coordinates into world coordinates.
 *              (i.e., world_point = orientation * local_point + position)
 * extents: full lengths of the box along local x,y,z (width, height, depth)
 */
struct OBB {
    Eigen::Vector3f position;       // world-space center
    Eigen::Quaternionf orientation; // rotation local -> world
    Eigen::Vector3f extents;        // full lengths along local x,y,z

    // Return the eight corners of the box in world coordinates.
    // Corner ordering is arbitrary but consistent.
    std::array<Eigen::Vector3f, 8> corners() const;

    // Volume (convenience)
    float volume() const { return extents.x() * extents.y() * extents.z(); }
};

/*
 * ObbEstimator
 *
 * Provides utilities to compute an OBB from a PCL point cloud using
 * PCL::MomentOfInertiaEstimation. The API is static and header-only in terms
 * of usage (implementation in .cpp).
 *
 * Template/typedef:
 *  - PointT: pcl::PointXYZRGB (adapt if you need other point types).
 */
class ObbEstimator {
public:
    using PointT = pcl::PointXYZRGB;
    using Cloud = pcl::PointCloud<PointT>;

    /*
     * computeOBBFromMoments
     *
     * Compute the oriented bounding box for `cloud` using PCL's
     * MomentOfInertiaEstimation. The function:
     *  - returns an OBB with center (position), orientation (quaternion),
     *    and extents (full lengths).
     *  - if the cloud is empty or null, returns an OBB with zero extents.
     *
     * Implementation details (in .cpp):
     *  - constructs a MomentOfInertiaEstimation<PointT> feature extractor,
     *    sets input cloud and calls compute().
     *  - then calls getOBB(min_pt, max_pt, position, rotational_matrix)
     *    to obtain the OBB in PCL's representation.
     *  - converts min/max + rotation + position into our OBB struct.
     *
     * NOTE:
     *  - PCL's getOBB returns min/max points in the box-local frame plus
     *    a position and rotation matrix that maps the box-local frame to
     *    world coordinates. We compute extents = max - min and keep the
     *    rotation as an Eigen::Quaternionf built from the rotation matrix.
     */
    static OBB computeOBBFromMoments(const Cloud::ConstPtr &cloud);
};

} // namespace robocup_vision_core

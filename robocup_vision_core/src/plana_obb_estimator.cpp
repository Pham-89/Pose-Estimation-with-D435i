#include "plana_obb_estimator.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace robocup_vision_core {

float PlanarObbEstimator::cross2D(const Point2D &O, const Point2D &A, const Point2D &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// Andrew's monotone chain convex hull. Returns hull points in
// counter-clockwise order (no duplicated first/last point).
std::vector<PlanarObbEstimator::Point2D>
PlanarObbEstimator::convexHull2D(std::vector<Point2D> pts)
{
    std::sort(pts.begin(), pts.end(), [](const Point2D &a, const Point2D &b) {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    });
    pts.erase(std::unique(pts.begin(), pts.end(),
        [](const Point2D &a, const Point2D &b) { return a.x == b.x && a.y == b.y; }),
        pts.end());

    size_t n = pts.size();
    if (n < 3) return pts;

    std::vector<Point2D> hull(2 * n);
    int k = 0;

    // lower hull
    for (size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross2D(hull[k-2], hull[k-1], pts[i]) <= 0) --k;
        hull[k++] = pts[i];
    }
    // upper hull
    for (int i = static_cast<int>(n) - 2, t = k + 1; i >= 0; --i) {
        while (k >= t && cross2D(hull[k-2], hull[k-1], pts[i]) <= 0) --k;
        hull[k++] = pts[i];
    }

    hull.resize(k - 1); // drop duplicated closing point
    return hull;
}

OBB PlanarObbEstimator::computeOBB(const Cloud::ConstPtr &cloud,
                                    const Eigen::Vector3f &plane_normal_in,
                                    const Eigen::Vector3f &plane_point)
{
    OBB obb;
    obb.position.setZero();
    obb.orientation = Eigen::Quaternionf::Identity();
    obb.extents.setZero();

    if (!cloud || cloud->size() < 3) {
        return obb;
    }

    Eigen::Vector3f n = plane_normal_in.normalized();

    // Orthonormal basis (u, v, n) spanning the plane + its normal.
    Eigen::Vector3f helper = (std::abs(n.z()) < 0.9f) ? Eigen::Vector3f::UnitZ()
                                                        : Eigen::Vector3f::UnitX();
    Eigen::Vector3f u = n.cross(helper).normalized();
    Eigen::Vector3f v = n.cross(u).normalized();

    // Project every point into (u, v); track extent along n (block height).
    std::vector<Point2D> pts2d;
    pts2d.reserve(cloud->size());
    float min_h = std::numeric_limits<float>::max();
    float max_h = std::numeric_limits<float>::lowest();

    for (const auto &p : cloud->points) {
        Eigen::Vector3f d(p.x - plane_point.x(), p.y - plane_point.y(), p.z - plane_point.z());
        pts2d.push_back({ d.dot(u), d.dot(v) });
        float h = d.dot(n);
        min_h = std::min(min_h, h);
        max_h = std::max(max_h, h);
    }

    std::vector<Point2D> hull = convexHull2D(pts2d);

    if (hull.size() < 3) {
        // Degenerate cluster (near-collinear points): fall back to an
        // axis-aligned bbox in (u,v) so we still return something usable.
        float minx = std::numeric_limits<float>::max(), maxx = std::numeric_limits<float>::lowest();
        float miny = std::numeric_limits<float>::max(), maxy = std::numeric_limits<float>::lowest();
        for (auto &p : pts2d) {
            minx = std::min(minx, p.x); maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y); maxy = std::max(maxy, p.y);
        }
        float cx = (minx + maxx) * 0.5f, cy = (miny + maxy) * 0.5f;
        float ch = (min_h + max_h) * 0.5f;
        obb.position = plane_point + cx * u + cy * v + ch * n;
        Eigen::Matrix3f R; R.col(0) = u; R.col(1) = v; R.col(2) = n;
        obb.orientation = Eigen::Quaternionf(R);
        obb.extents = Eigen::Vector3f(maxx - minx, maxy - miny, max_h - min_h);
        return obb;
    }

    // --- Rotating calipers: minimum-area bounding rectangle of the hull ---
    float best_area = std::numeric_limits<float>::max();
    float best_angle = 0.f;
    float best_minx = 0, best_maxx = 0, best_miny = 0, best_maxy = 0;

    size_t hn = hull.size();
    for (size_t i = 0; i < hn; ++i) {
        const Point2D &p1 = hull[i];
        const Point2D &p2 = hull[(i + 1) % hn];
        float edge_angle = std::atan2(p2.y - p1.y, p2.x - p1.x);

        float c = std::cos(-edge_angle);
        float s = std::sin(-edge_angle);

        float minx = std::numeric_limits<float>::max(), maxx = std::numeric_limits<float>::lowest();
        float miny = std::numeric_limits<float>::max(), maxy = std::numeric_limits<float>::lowest();
        for (const auto &p : hull) {
            float rx = p.x * c - p.y * s;
            float ry = p.x * s + p.y * c;
            minx = std::min(minx, rx); maxx = std::max(maxx, rx);
            miny = std::min(miny, ry); maxy = std::max(maxy, ry);
        }
        float area = (maxx - minx) * (maxy - miny);
        if (area < best_area) {
            best_area = area;
            best_angle = edge_angle;
            best_minx = minx; best_maxx = maxx;
            best_miny = miny; best_maxy = maxy;
        }
    }

    // Rectangle center back in (u, v).
    float rect_cx_rot = (best_minx + best_maxx) * 0.5f;
    float rect_cy_rot = (best_miny + best_maxy) * 0.5f;
    float c = std::cos(best_angle);
    float s = std::sin(best_angle);
    float cx = rect_cx_rot * c - rect_cy_rot * s;
    float cy = rect_cx_rot * s + rect_cy_rot * c;

    float width  = best_maxx - best_minx; // along rectU
    float depth  = best_maxy - best_miny; // along rectV
    float height = max_h - min_h;         // along n
    float ch = (min_h + max_h) * 0.5f;

    Eigen::Vector3f rectU = c * u + s * v;
    Eigen::Vector3f rectV = -s * u + c * v;

    obb.position = plane_point + cx * u + cy * v + ch * n;

    Eigen::Matrix3f R;
    R.col(0) = rectU;
    R.col(1) = rectV;
    R.col(2) = n;
    obb.orientation = Eigen::Quaternionf(R);
    obb.orientation.normalize();

    obb.extents = Eigen::Vector3f(width, depth, height);

    return obb;
}

OBB PlanarObbEstimator::computeOBB(const Cloud::ConstPtr &cloud,
                                    const Eigen::Vector4f &plane_coefficients)
{
    Eigen::Vector3f normal(plane_coefficients[0], plane_coefficients[1], plane_coefficients[2]);
    float norm = normal.norm();
    if (norm < 1e-8f) {
        normal = Eigen::Vector3f::UnitZ();
        norm = 1.f;
    }
    normal /= norm;
    float d = plane_coefficients[3] / norm;
    Eigen::Vector3f point_on_plane = -d * normal; // origin projected onto the plane
    return computeOBB(cloud, normal, point_on_plane);
}

} // namespace robocup_vision_core

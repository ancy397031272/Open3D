// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <cmath>

#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"

namespace open3d {
namespace geometry {

Eigen::Vector4d ToEigenVector4(const Eigen::Vector3d& in) {
    Eigen::Vector4d out = Eigen::Vector4d::Zero();
    memcpy(out.data(), in.data(), sizeof(double) * 3);
    return out;
}

void GetCoordinateSystemOnPlane(const Eigen::Vector3d& query,
                                Eigen::Vector4d& u,
                                Eigen::Vector4d& v) {
    const Eigen::Vector4d vector = ToEigenVector4(query);
    v = vector.unitOrthogonal();
    u = vector.cross3(v);
}

bool IsBoundaryPoint(const open3d::geometry::PointCloud& pc,
                     const Eigen::Vector3d& query,
                     const std::vector<int> indices,
                     const Eigen::Vector4d& u,
                     const Eigen::Vector4d& v,
                     double angle_threshold) {
    std::vector<double> angles;
    for (size_t i = 0; i < indices.size(); i++) {
        const Eigen::Vector4d delta =
                ToEigenVector4(pc.points_[indices[i]]) - ToEigenVector4(query);
        if (delta == Eigen::Vector4d::Zero()) {
            continue;
        }
        angles.push_back(atan2(v.dot(delta), u.dot(delta)));
    }

    if (angles.empty()) {
        return false;
    }

    std::sort(angles.begin(), angles.end());
    // Compute the maximal angle difference between two consecutive angles
    double dif;
    double max_dif = 0;
    for (size_t i = 0; i < angles.size() - 1; ++i) {
        dif = angles[i + 1] - angles[i];
        if (max_dif < dif) {
            max_dif = dif;
        }
    }

    // Get the angle difference between the last and the first
    dif = 2 * M_PI - angles[angles.size() - 1] + angles[0];
    if (max_dif < dif) max_dif = dif;

    // Check results
    if (max_dif > angle_threshold * M_PI / 180.0)
        return true;
    else
        return false;
}

std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
PointCloud::DetectBoundaryPoints(const KDTreeSearchParam& param,
                                 double angle_threshold) {
    std::vector<size_t> boundary_indices;
    if (!HasPoints()) {
        utility::LogError("No PointCloud data.");
    }
    if (!HasNormals()) {
        utility::LogError(
                "PointCloud must have normals attribute to compute boundary "
                "points.");
    }

    const size_t num = points_.size();
    open3d::geometry::KDTreeFlann kdtree(*this);
#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())
    for (int idx = 0; idx < static_cast<int>(num); idx++) {
        std::vector<int> ret_indices;
        std::vector<double> out_dists_sqr;
        if (kdtree.Search(points_[idx], param, ret_indices, out_dists_sqr) <
            3) {
            continue;
        }
        Eigen::Vector4d u = Eigen::Vector4d::Zero(),
                        v = Eigen::Vector4d::Zero();
        // Obtain a coordinate system on the plane
        GetCoordinateSystemOnPlane(normals_[idx], u, v);

        // Estimate whether the point is lying on a boundary surface or not
        if (IsBoundaryPoint(*this, points_[idx], ret_indices, u, v,
                            angle_threshold)) {
#pragma omp critical
            { boundary_indices.push_back(idx); }
        }
    }

    utility::LogInfo("Found {} boundary points from {} input points.",
                     boundary_indices.size(), num);

    return std::make_tuple(SelectByIndex(boundary_indices), boundary_indices);
}

}  // namespace geometry

}  // namespace open3d

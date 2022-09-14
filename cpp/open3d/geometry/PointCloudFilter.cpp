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

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <limits>

#include "open3d/geometry/BoundingVolume.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"
#include "open3d/utility/ProgressBar.h"

namespace open3d {
namespace geometry {

void getMinMax3D(const std::vector<Eigen::Vector3d>& points,
                 Eigen::Vector3d& min_pt,
                 Eigen::Vector3d& max_pt) {
    min_pt.setConstant(std::numeric_limits<float>::max());
    max_pt.setConstant(std::numeric_limits<float>::lowest());
#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())
    for (size_t i = 0; i < points.size(); ++i) {
        // Check if the point is invalid
        if (!std::isfinite(points[i][0]) || !std::isfinite(points[i][1]) ||
            !std::isfinite(points[i][2]))
            continue;
        min_pt = min_pt.cwiseMin(points[i]);
        max_pt = max_pt.cwiseMax(points[i]);
    }
}

std::vector<Eigen::Vector3d> GetPointWithinBoundingBox(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& bbox_min,
        const Eigen::Vector3d& bbox_max) {
    std::vector<Eigen::Vector3d> inside_points;
#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())
    for (size_t idx = 0; idx < points.size(); idx++) {
        const auto& point = points[idx];
        if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) &&
            point(1) >= bbox_min(1) && point(1) <= bbox_max(1) &&
            point(2) >= bbox_min(2) && point(2) <= bbox_max(2)) {
            inside_points.push_back(points[idx]);
        }
    }
    return inside_points;
}

std::vector<double> GetPointZWithinBoundingBox(
        const std::vector<Eigen::Vector3d>& points,
        const Eigen::Vector3d& bbox_min,
        const Eigen::Vector3d& bbox_max) {
    std::vector<double> inside_points;
#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())
    for (size_t idx = 0; idx < points.size(); idx++) {
        const auto& point = points[idx];
        if (point(0) >= bbox_min(0) && point(0) <= bbox_max(0) &&
            point(1) >= bbox_min(1) && point(1) <= bbox_max(1) &&
            point(2) >= bbox_min(2) && point(2) <= bbox_max(2)) {
            inside_points.push_back(points[idx][2]);
        }
    }
    return inside_points;
}

enum morph_operator { MORPH_DILATE, MORPH_ERODE };

std::shared_ptr<PointCloud> PointCloud::Morphological(
        float resolution, const int morph_operator) {
    if (points_.empty()) {
        utility::LogDebug("Morphological point cloud size is 0");
        return std::make_shared<PointCloud>(*this);
    }
    auto cloud_out = std::make_shared<PointCloud>(*this);

    float extent_res = resolution / 2.0f;

    switch (morph_operator) {
        case MORPH_DILATE:
        case MORPH_ERODE: {
#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())

            for (size_t p_idx = 0; p_idx < points_.size(); ++p_idx) {
                double minx = points_[p_idx][0] - extent_res;
                double miny = points_[p_idx][1] - extent_res;
                double minz = -std::numeric_limits<float>::max();
                double maxx = points_[p_idx][0] + extent_res;
                double maxy = points_[p_idx][1] + extent_res;
                double maxz = std::numeric_limits<float>::max();
                Eigen::Vector3d bbox_min(minx, miny, minz);
                Eigen::Vector3d bbox_max(maxx, maxy, maxz);

                auto inside_points = GetPointWithinBoundingBox(
                        (*this).points_, bbox_min, bbox_max);

                if (!inside_points.empty()) {
                    Eigen::Vector3d min_pt, max_pt;
                    getMinMax3D(inside_points, min_pt, max_pt);

                    switch (morph_operator) {
                        case MORPH_DILATE: {
                            cloud_out->points_[p_idx][2] = max_pt[2];
                            break;
                        }
                        case MORPH_ERODE: {
                            cloud_out->points_[p_idx][2] = min_pt[2];
                            break;
                        }
                    }
                }
            }
        }
            //
            //        default:
            //            utility::LogError("Morphological operator is not
            //            supported "); break;
    }

    return cloud_out;
}

std::shared_ptr<PointCloud> PointCloud::MedianFilter(float resolution) {
    if (points_.empty()) {
        utility::LogDebug("MedianFilter point cloud size is 0");
        return std::make_shared<PointCloud>(*this);
    }
    auto cloud_out = std::make_shared<PointCloud>(*this);

    double extent_res = resolution / 2.0f;

#pragma omp parallel for schedule(static) \
        num_threads(utility::EstimateMaxThreads())

    for (size_t p_idx = 0; p_idx < points_.size(); ++p_idx) {
        double minx = points_[p_idx][0] - extent_res;
        double miny = points_[p_idx][1] - extent_res;
        double minz = -std::numeric_limits<float>::max();
        double maxx = points_[p_idx][0] + extent_res;
        double maxy = points_[p_idx][1] + extent_res;
        double maxz = std::numeric_limits<float>::max();
        Eigen::Vector3d bbox_min(minx, miny, minz);
        Eigen::Vector3d bbox_max(maxx, maxy, maxz);

        auto vals =
                GetPointZWithinBoundingBox((*this).points_, bbox_min, bbox_max);
        if (vals.empty()) continue;

        auto middle_it = vals.begin() + vals.size() / 2;
        std::nth_element(vals.begin(), middle_it, vals.end());
        float new_depth = *middle_it;

        cloud_out->points_[p_idx][2] = new_depth;
    }
    return cloud_out;
}
}  // namespace geometry
}  // namespace open3d

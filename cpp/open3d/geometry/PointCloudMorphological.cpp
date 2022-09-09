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

#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/BoundingVolume.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"
#include "open3d/utility/ProgressBar.h"

namespace open3d {
namespace geometry {


void getMinMax3D(const PointCloud &cloud,
                 Eigen::Vector3d &min_pt,
                 Eigen::Vector3d &max_pt)
{
    min_pt.setConstant(std::numeric_limits<float>::max());
    max_pt.setConstant(std::numeric_limits<float>::lowest());

    for (const auto &point : cloud.points_) {
        // Check if the point is invalid
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2]))
            continue;
        min_pt = min_pt.cwiseMin(point);
        max_pt = max_pt.cwiseMax(point);
    }

}

enum  morph_operator {
    MORPH_OPEN,
    MORPH_CLOSE,
    MORPH_DILATE,
    MORPH_ERODE
};

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
            for (size_t p_idx = 0; p_idx < points_.size(); ++p_idx) {

                double minx = points_[p_idx][0] - extent_res;
                double miny = points_[p_idx][1] - extent_res;
                double minz = -std::numeric_limits<float>::max();
                double maxx = points_[p_idx][0] + extent_res;
                double maxy = points_[p_idx][1] + extent_res;
                double maxz = std::numeric_limits<float>::max();
                Eigen::Vector3d bbox_min(minx, miny, minz);
                Eigen::Vector3d bbox_max(maxx, maxy, maxz);
                AxisAlignedBoundingBox aabb(bbox_min, bbox_max);

                auto pcd_in_aabb = (*this).Crop(aabb);

                if (!pcd_in_aabb->points_.empty()) {
                    Eigen::Vector3d min_pt, max_pt;
                    getMinMax3D((*pcd_in_aabb), min_pt, max_pt);

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
    }

    return cloud_out;
}
}  // namespace geometry
}  // namespace open3d

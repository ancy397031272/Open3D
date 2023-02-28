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

#include <open3d/Open3D.h>
#include <open3d/pipelines/pose_estimation/DataStructure.h>

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace open3d;
int main() {

    pipelines::pose_estimation::PPFEstimatorConfig config;
    config.training_param_.invert_model_normal = true;
    config.training_param_.rel_sample_dist = 0.03;
    config.training_param_.calc_normal_relative = 0.04;
    config.training_param_.rel_dense_sample_dist = 0.02;

    config.voting_param_.min_angle_thresh = 0.52;
    config.voting_param_.min_dist_thresh = 0.1;
    config.score_thresh_ = 0.01;
    config.num_result_ = 1;

    config.refine_param_.method = pipelines::pose_estimation::
            PPFEstimatorConfig::RefineMethod::PointToPlane;

    pipelines::pose_estimation::PPFEstimator ppf(config);

    ppf.LoadModel(
            "/home/rvbust/Tmp/machined_parts_pose_estimation/data/model.p3m");

    geometry::PointCloud scene_pc;
    io::ReadPointCloud(
            "/home/rvbust/Tmp/machined_parts_pose_estimation/output/"
            "sampled_pcd.ply",
            scene_pc);
    std::vector<pipelines::pose_estimation::Pose6D> results;
    //
    std::shared_ptr<geometry::PointCloud> scene_pc_ptr =
            std::make_shared<geometry::PointCloud>(scene_pc);
    ppf.Estimate(scene_pc_ptr, results);


    return 0;
}
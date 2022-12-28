# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

import numpy as np
import open3d as o3d


if __name__ == '__main__':
    config = o3d.pipelines.pose_estimation.PPFEstimatorConfig()
    # init training param
    config.training_param.rel_sample_dist = 0.03
    config.training_param.calc_normal_relative = 0.02
    config.voting_param.min_angle_thresh = 20
    config.voting_param.min_dist_thresh = 0.1
    config.score_thresh = 0.01
    config.refine_param.method = o3d.pipelines.pose_estimation.PPFEstimatorConfig.PointToPlane
    # init ppf detector
    ppf = o3d.pipelines.pose_estimation.PPFEstimator(config)

    model = o3d.io.read_point_cloud('/home/rvbust/Rvbust/Data/XT/鞋面/0926/三维重建、视觉精度数据/22/pts_model_2.ply')

    # train ppf detector
    ret = ppf.train(model)

    if ret is False:
        print('train fail')
        exit()

    scene = o3d.io.read_point_cloud('/home/rvbust/Rvbust/Data/XT/鞋面/0926/三维重建、视觉精度数据/22/pts_2_bk.ply')

    # mathch scene points
    ret, results = ppf.estimate(scene)
    print(len(results))
    if ret is False:
        print('No matched')
    else:
        pose = results[0].pose
        sampled_model = ppf.get_sampled_model()
        reg_result = o3d.pipelines.registration.registration_icp(
            sampled_model, scene, 0.01, pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        pose = reg_result.transformation

        transformed_model = model.transform(pose)
        transformed_model.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([transformed_model, scene])


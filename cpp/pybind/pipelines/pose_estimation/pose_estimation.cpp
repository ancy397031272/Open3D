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

#include "open3d/pipelines/pose_estimation/PPFEstimation.h"

#include "pybind/docstring.h"
#include "pybind/pipelines/pose_estimation/pose_estimation.h"

namespace open3d {
namespace pipelines {
namespace pose_estimation {


void pybind_pose_estimation_classes(py::module &m) {
    py::class_<PPFEstimator>(m, "PPFEstimator")
            .def(py::init<>())
            .def(py::init<const PPFEstimatorConfig &>())
            .def("set_config", &PPFEstimator::SetConfig)
            .def("train", &PPFEstimator::Train)
            .def("estimate",
                 [](PPFEstimator &self, const PointCloudPtr &pc) {
                   std::vector<Pose6D> results;
                   bool ret = self.Estimate(pc, results);
                   return std::make_tuple(ret, results);
                 })
            .def("get_model_diameter", &PPFEstimator::GetModelDiameter)
            .def("get_pose", &PPFEstimator::GetPose)
            .def("get_sampled_model", &PPFEstimator::GetSampledModel)
            .def("get_sampled_scene", &PPFEstimator::GetSampledScene)
            .def("get_model_edges", &PPFEstimator::GetModelEdges)
            .def("load_model", &PPFEstimator::LoadModel)
            .def("save_model", &PPFEstimator::SaveModel)
            .def("get_scene_edges", &PPFEstimator::GetSceneEdges);

    py::class_<Pose6D>(m, "Pose6D")
            .def(py::init<>())
            .def_readwrite("pose", &Pose6D::pose_)
            .def_readwrite("q", &Pose6D::q_)
            .def_readwrite("t", &Pose6D::t_)
            .def_readwrite("num_votes", &Pose6D::num_votes_)
            .def_readwrite("score", &Pose6D::score_)
            .def_readwrite("object_id", &Pose6D::object_id_)
            .def_readwrite("corr_mi", &Pose6D::corr_mi_);

    py::class_<PPFEstimatorConfig> config(m, "PPFEstimatorConfig");
    config.def(py::init<>());

    py::class_<PPFEstimatorConfig::TrainingParam>(config, "TrainingParam")
            .def(py::init<>())
            .def_readwrite("invert_model_normal",
                           &PPFEstimatorConfig::TrainingParam::invert_model_normal)
            .def_readwrite("use_external_normal",
                           &PPFEstimatorConfig::TrainingParam::use_external_normal)
            .def_readwrite("rel_sample_dist",
                           &PPFEstimatorConfig::TrainingParam::rel_sample_dist)
            .def_readwrite("calc_normal_relative",
                           &PPFEstimatorConfig::TrainingParam::calc_normal_relative)
            .def_readwrite(
                    "rel_dense_sample_dist",
                    &PPFEstimatorConfig::TrainingParam::rel_dense_sample_dist);

    py::enum_<PPFEstimatorConfig::ReferencePointSelection>(
            config, "ReferencePointSelection")
            .value("Random", PPFEstimatorConfig::ReferencePointSelection::Random)
            .export_values();

    py::enum_<PPFEstimatorConfig::RefineMethod>(config, "RefineMethod")
            .value("NoRefine", PPFEstimatorConfig::RefineMethod::NoRefine)
            .value("PointToPlane", PPFEstimatorConfig::RefineMethod::PointToPlane)
            .value("PointToPoint", PPFEstimatorConfig::RefineMethod::PointToPoint)
            .export_values();

    py::enum_<PPFEstimatorConfig::VotingMode>(config, "VotingMode")
            .value("SampledPoints", PPFEstimatorConfig::VotingMode::SampledPoints)
            .value("EdgePoints", PPFEstimatorConfig::VotingMode::EdgePoints)
            .export_values();

    py::class_<PPFEstimatorConfig::EdgeParam>(config, "EdgeParam")
            .def(py::init<>())
            .def_readwrite("pts_num", &PPFEstimatorConfig::EdgeParam::pts_num)
            .def_readwrite("angle_threshold", &PPFEstimatorConfig::EdgeParam::angle_threshold);

    py::class_<PPFEstimatorConfig::ReferenceParam>(config, "ReferenceParam")
            .def(py::init<>())
            .def_readwrite("method", &PPFEstimatorConfig::ReferenceParam::method)
            .def_readwrite("ratio", &PPFEstimatorConfig::ReferenceParam::ratio);

    py::class_<PPFEstimatorConfig::VotingParam>(config, "VotingParam")
            .def(py::init<>())
            .def_readwrite("method", &PPFEstimatorConfig::VotingParam::method)
            .def_readwrite("faster_mode",
                           &PPFEstimatorConfig::VotingParam::faster_mode)
            .def_readwrite("angle_step",
                           &PPFEstimatorConfig::VotingParam::angle_step)
            .def_readwrite("min_dist_thresh",
                           &PPFEstimatorConfig::VotingParam::min_dist_thresh)
            .def_readwrite("min_angle_thresh",
                           &PPFEstimatorConfig::VotingParam::min_angle_thresh);

    py::class_<PPFEstimatorConfig::RefineParam>(config, "RefineParam")
            .def(py::init<>())
            .def_readwrite("method", &PPFEstimatorConfig::RefineParam::method)
            .def_readwrite(
                    "rel_dist_sparse_thresh",
                    &PPFEstimatorConfig::RefineParam::rel_dist_sparse_thresh);

    config.def_readwrite("training_param",
                         &PPFEstimatorConfig::training_param_);
    config.def_readwrite("ref_param", &PPFEstimatorConfig::ref_param_);
    config.def_readwrite("voting_param", &PPFEstimatorConfig::voting_param_);
    config.def_readwrite("edge_param", &PPFEstimatorConfig::edge_param_);
    config.def_readwrite("refine_param", &PPFEstimatorConfig::refine_param_);
    config.def_readwrite("score_thresh", &PPFEstimatorConfig::score_thresh_);
    config.def_readwrite("num_result", &PPFEstimatorConfig::num_result_);
    config.def_readwrite("object_id", &PPFEstimatorConfig::object_id_);
}


void pybind_pose_estimation(py::module &m) {
    py::module m_submodule = m.def_submodule("pose_estimation", "pose_estimation pipeline.");
    pybind_pose_estimation_classes(m_submodule);
}

}  // namespace pose_estimation
}  // namespace pipelines
}  // namespace open3d

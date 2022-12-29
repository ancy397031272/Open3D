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
#include <teaser/registration.h>

#include "open3d/pipelines/registration/Registration.h"

#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/registration/Feature.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"
#include "open3d/utility/Random.h"

namespace open3d {
namespace pipelines {
namespace registration {

static RegistrationResult GetRegistrationResultAndCorrespondences(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const geometry::KDTreeFlann &target_kdtree,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);
    if (max_correspondence_distance <= 0.0) {
        return result;
    }

    double error2 = 0.0;

#pragma omp parallel
    {
        double error2_private = 0.0;
        CorrespondenceSet correspondence_set_private;
#pragma omp for nowait
        for (int i = 0; i < (int)source.points_.size(); i++) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            const auto &point = source.points_[i];
            if (target_kdtree.SearchHybrid(point, max_correspondence_distance,
                                           1, indices, dists) > 0) {
                error2_private += dists[0];
                correspondence_set_private.push_back(
                        Eigen::Vector2i(i, indices[0]));
            }
        }
#pragma omp critical(GetRegistrationResultAndCorrespondences)
        {
            for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                result.correspondence_set_.push_back(
                        correspondence_set_private[i]);
            }
            error2 += error2_private;
        }
    }

    if (result.correspondence_set_.empty()) {
        result.fitness_ = 0.0;
        result.inlier_rmse_ = 0.0;
    } else {
        size_t corres_number = result.correspondence_set_.size();
        result.fitness_ = (double)corres_number / (double)source.points_.size();
        result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
    }
    return result;
}

static double EvaluateInlierCorrespondenceRatio(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);

    int inlier_corres = 0;
    double max_dis2 = max_correspondence_distance * max_correspondence_distance;
    for (const auto &c : corres) {
        double dis2 =
                (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
        if (dis2 < max_dis2) {
            inlier_corres++;
        }
    }

    return double(inlier_corres) / double(corres.size());
}

RegistrationResult EvaluateRegistration(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d
                &transformation /* = Eigen::Matrix4d::Identity()*/) {
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloud pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    return GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
}

RegistrationResult RegistrationICP(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/) {
    if (max_correspondence_distance <= 0.0) {
        utility::LogError("Invalid max_correspondence_distance.");
    }
    if ((estimation.GetTransformationEstimationType() ==
                 TransformationEstimationType::PointToPlane ||
         estimation.GetTransformationEstimationType() ==
                 TransformationEstimationType::ColoredICP) &&
        (!target.HasNormals())) {
        utility::LogError(
                "TransformationEstimationPointToPlane and "
                "TransformationEstimationColoredICP "
                "require pre-computed normal vectors for target PointCloud.");
    }
    if ((estimation.GetTransformationEstimationType() ==
         TransformationEstimationType::GeneralizedICP) &&
        (!target.HasCovariances() || !source.HasCovariances())) {
        utility::LogError(
                "TransformationEstimationForGeneralizedICP require "
                "pre-computed per point covariances matrices for source and "
                "target PointCloud.");
    }

    Eigen::Matrix4d transformation = init;
    geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
    geometry::PointCloud pcd = source;
    if (!init.isIdentity()) {
        pcd.Transform(init);
    }
    RegistrationResult result;
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, kdtree, max_correspondence_distance, transformation);
    for (int i = 0; i < criteria.max_iteration_; i++) {
        utility::LogDebug("ICP Iteration #{:d}: Fitness {:.4f}, RMSE {:.4f}", i,
                          result.fitness_, result.inlier_rmse_);
        Eigen::Matrix4d update = estimation.ComputeTransformation(
                pcd, target, result.correspondence_set_);
        transformation = update * transformation;
        pcd.Transform(update);
        RegistrationResult backup = result;
        result = GetRegistrationResultAndCorrespondences(
                pcd, target, kdtree, max_correspondence_distance,
                transformation);
        if (std::abs(backup.fitness_ - result.fitness_) <
                    criteria.relative_fitness_ &&
            std::abs(backup.inlier_rmse_ - result.inlier_rmse_) <
                    criteria.relative_rmse_) {
            break;
        }
    }
    return result;
}

RegistrationResult RegistrationRANSACBasedOnCorrespondence(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres,
        double max_correspondence_distance,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers /* = {}*/,
        const RANSACConvergenceCriteria &criteria
        /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || (int)corres.size() < ransac_n ||
        max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    RegistrationResult best_result;
    geometry::KDTreeFlann kdtree(target);
    int est_k_global = criteria.max_iteration_;
    int total_validation = 0;

#pragma omp parallel
    {
        CorrespondenceSet ransac_corres(ransac_n);
        RegistrationResult best_result_local;
        int est_k_local = criteria.max_iteration_;
        utility::random::UniformIntGenerator<int> rand_gen(0,
                                                           corres.size() - 1);

#pragma omp for nowait
        for (int itr = 0; itr < criteria.max_iteration_; itr++) {
            if (itr < est_k_global) {
                for (int j = 0; j < ransac_n; j++) {
                    ransac_corres[j] = corres[rand_gen()];
                }

                Eigen::Matrix4d transformation =
                        estimation.ComputeTransformation(source, target,
                                                         ransac_corres);

                // Check transformation: inexpensive
                bool check = true;
                for (const auto &checker : checkers) {
                    if (!checker.get().Check(source, target, ransac_corres,
                                             transformation)) {
                        check = false;
                        break;
                    }
                }
                if (!check) continue;

                // Expensive validation
                geometry::PointCloud pcd = source;
                pcd.Transform(transformation);
                auto result = GetRegistrationResultAndCorrespondences(
                        pcd, target, kdtree, max_correspondence_distance,
                        transformation);

                if (result.IsBetterRANSACThan(best_result_local)) {
                    best_result_local = result;

                    double corres_inlier_ratio =
                            EvaluateInlierCorrespondenceRatio(
                                    pcd, target, corres,
                                    max_correspondence_distance,
                                    transformation);

                    // Update exit condition if necessary.
                    // If confidence is 1.0, then it is safely inf, we always
                    // consume all the iterations.
                    double est_k_local_d =
                            std::log(1.0 - criteria.confidence_) /
                            std::log(1.0 -
                                     std::pow(corres_inlier_ratio, ransac_n));
                    est_k_local =
                            est_k_local_d < est_k_global
                                    ? static_cast<int>(std::ceil(est_k_local_d))
                                    : est_k_local;
                    utility::LogDebug(
                            "Thread {:06d}: registration fitness={:.3f}, "
                            "corres inlier ratio={:.3f}, "
                            "Est. max k = {}",
                            itr, result.fitness_, corres_inlier_ratio,
                            est_k_local_d);
                }
#pragma omp critical
                {
                    total_validation += 1;
                    if (est_k_local < est_k_global) {
                        est_k_global = est_k_local;
                    }
                }
            }  // if
        }      // for loop

#pragma omp critical(RegistrationRANSACBasedOnCorrespondence)
        {
            if (best_result_local.IsBetterRANSACThan(best_result)) {
                best_result = best_result_local;
            }
        }
    }
    utility::LogDebug(
            "RANSAC exits after {:d} validations. Best inlier ratio {:e}, "
            "RMSE {:e}",
            total_validation, best_result.fitness_, best_result.inlier_rmse_);
    return best_result;
}

RegistrationResult RegistrationRANSACBasedOnFeatureMatching(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        bool mutual_filter,
        double max_correspondence_distance,
        const TransformationEstimation
                &estimation /* = TransformationEstimationPointToPoint(false)*/,
        int ransac_n /* = 3*/,
        const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
                &checkers /* = {}*/,
        const RANSACConvergenceCriteria
                &criteria /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    int num_src_pts = int(source.points_.size());
    int num_tgt_pts = int(target.points_.size());

    geometry::KDTreeFlann kdtree_target(target_feature);
    pipelines::registration::CorrespondenceSet corres_ij(num_src_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
    for (int i = 0; i < num_src_pts; i++) {
        std::vector<int> corres_tmp(1);
        std::vector<double> dist_tmp(1);

        kdtree_target.SearchKNN(Eigen::VectorXd(source_feature.data_.col(i)), 1,
                                corres_tmp, dist_tmp);
        int j = corres_tmp[0];
        corres_ij[i] = Eigen::Vector2i(i, j);
    }

    // Do reverse check if mutual_filter is enabled
    if (mutual_filter) {
        geometry::KDTreeFlann kdtree_source(source_feature);
        pipelines::registration::CorrespondenceSet corres_ji(num_tgt_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
        for (int j = 0; j < num_tgt_pts; ++j) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);
            kdtree_source.SearchKNN(
                    Eigen::VectorXd(target_feature.data_.col(j)), 1, corres_tmp,
                    dist_tmp);
            int i = corres_tmp[0];
            corres_ji[j] = Eigen::Vector2i(i, j);
        }

        pipelines::registration::CorrespondenceSet corres_mutual;
        for (int i = 0; i < num_src_pts; ++i) {
            int j = corres_ij[i](1);
            if (corres_ji[j](0) == i) {
                corres_mutual.emplace_back(i, j);
            }
        }

        // Empirically mutual correspondence set should not be too small
        if (int(corres_mutual.size()) >= ransac_n * 3) {
            utility::LogDebug("{:d} correspondences remain after mutual filter",
                              corres_mutual.size());
            return RegistrationRANSACBasedOnCorrespondence(
                    source, target, corres_mutual, max_correspondence_distance,
                    estimation, ransac_n, checkers, criteria);
        }
        utility::LogDebug(
                "Too few correspondences after mutual filter, fall back to "
                "original correspondences.");
    }

    return RegistrationRANSACBasedOnCorrespondence(
            source, target, corres_ij, max_correspondence_distance, estimation,
            ransac_n, checkers, criteria);
}

RegistrationResult RegistrationTeaserBasedOnCorrespondence(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres,
        double noise_bound,
        double cbar2 /*= 1*/,
        bool estimate_scaling /* = false*/,
        double rotation_gnc_factor /* = 1.4*/,
        size_t rotation_max_iterations /*= 10000*/,
        double rotation_cost_threshold /*= 1e-16*/){
    if ((int)corres.size() < 3 || noise_bound <= 0.0 || cbar2<=0.0
        || rotation_gnc_factor <= 0.0 || rotation_cost_threshold <= 0.0) {
        return RegistrationResult();
    }
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = noise_bound;
    params.cbar2 = cbar2;
    params.estimate_scaling = estimate_scaling;
    params.rotation_gnc_factor = rotation_gnc_factor;
    params.rotation_max_iterations = rotation_max_iterations;
    params.rotation_cost_threshold = rotation_cost_threshold;
    params.inlier_selection_mode = teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_EXACT;
    params.rotation_tim_graph = teaser::RobustRegistrationSolver::INLIER_GRAPH_FORMULATION::CHAIN;
    params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;

    teaser::RobustRegistrationSolver solver(params);
    auto func_to_teaser_point_cloud = [](const open3d::geometry::PointCloud& source){
      teaser::PointCloud src;
      src.reserve(source.points_.size());
      for (size_t i = 0; i < source.points_.size(); ++i) {
          const auto& point = source.points_[i];
          src.push_back({(float )point.x(), (float )point.y(), (float )point.z()});
      }
      return src;
    };

    auto func_to_teaser_correspondences = [](const CorrespondenceSet& corres){
      std::vector<std::pair<int, int>> correspondences;
      correspondences.reserve(corres.size());
      for (size_t i = 0; i < corres.size(); ++i) {
          correspondences.push_back({corres[i].x(), corres[i].y()});
      }
      return correspondences;
    };

    teaser::PointCloud src = func_to_teaser_point_cloud(source);
    teaser::PointCloud dst = func_to_teaser_point_cloud(target);
    std::vector<std::pair<int, int>> correspondences = func_to_teaser_correspondences(corres);
    solver.solve(src, dst, correspondences);

    auto solution = solver.getSolution();
    RegistrationResult result;
    if (solution.valid){
        utility::LogDebug(
                "Teaser find solution success.");
        result.transformation_.topLeftCorner(3, 3) = solution.rotation;
        result.transformation_.topRightCorner(3, 1) = solution.translation;
        // calculate correspondence set
        auto source_transformed = std::make_shared<open3d::geometry::PointCloud>(source);
        source_transformed->Transform(result.transformation_);
        double error = 0;
        int count = 0;
        for (size_t i = 0; i < corres.size(); ++i) {
            auto p1 = source_transformed->points_[corres[i].x()];
            auto p2 = target.points_[corres[i].y()];
            auto dist = (p1 - p2).norm();
            if (dist <= noise_bound){
                result.correspondence_set_.push_back(corres[i]);
                error += dist;
                count++;
            }
        }
        result.inlier_rmse_ = error/count;
    }
    return result;
}

RegistrationResult RegistrationTeaserBasedOnFeatureMatching(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const Feature &source_feature,
        const Feature &target_feature,
        bool mutual_filter,
        double noise_bound,
        double cbar2 /*= 1*/,
        bool estimate_scaling /* = false*/,
        double rotation_gnc_factor /* = 1.4*/,
        size_t rotation_max_iterations /*= 10000*/,
        double rotation_cost_threshold /*= 1e-16*/){
    if (noise_bound <= 0.0 || cbar2<=0.0
        || rotation_gnc_factor <= 0.0 || rotation_cost_threshold <= 0.0) {
        return RegistrationResult();
    }

    int num_src_pts = int(source.points_.size());
    int num_tgt_pts = int(target.points_.size());

    geometry::KDTreeFlann kdtree_target(target_feature);
    pipelines::registration::CorrespondenceSet corres_ij(num_src_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
    for (int i = 0; i < num_src_pts; i++) {
        std::vector<int> corres_tmp(1);
        std::vector<double> dist_tmp(1);

        kdtree_target.SearchKNN(Eigen::VectorXd(source_feature.data_.col(i)), 1,
                                corres_tmp, dist_tmp);
        int j = corres_tmp[0];
        corres_ij[i] = Eigen::Vector2i(i, j);
    }

    // Do reverse check if mutual_filter is enabled
    if (mutual_filter) {
        geometry::KDTreeFlann kdtree_source(source_feature);
        pipelines::registration::CorrespondenceSet corres_ji(num_tgt_pts);

#pragma omp parallel for num_threads(utility::EstimateMaxThreads())
        for (int j = 0; j < num_tgt_pts; ++j) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);
            kdtree_source.SearchKNN(
                    Eigen::VectorXd(target_feature.data_.col(j)), 1, corres_tmp,
                    dist_tmp);
            int i = corres_tmp[0];
            corres_ji[j] = Eigen::Vector2i(i, j);
        }

        pipelines::registration::CorrespondenceSet corres_mutual;
        for (int i = 0; i < num_src_pts; ++i) {
            int j = corres_ij[i](1);
            if (corres_ji[j](0) == i) {
                corres_mutual.emplace_back(i, j);
            }
        }

        // Empirically mutual correspondence set should not be too small
        if (int(corres_mutual.size()) >= 3 * 3) {
            utility::LogDebug("{:d} correspondences remain after mutual filter",
                              corres_mutual.size());
            return RegistrationTeaserBasedOnCorrespondence(
                    source, target, corres_mutual, noise_bound,
                    cbar2, estimate_scaling, rotation_gnc_factor, rotation_max_iterations, rotation_cost_threshold);
        }
        utility::LogDebug(
                "Too few correspondences after mutual filter, fall back to "
                "original correspondences.");
    }
    return RegistrationTeaserBasedOnCorrespondence(
            source, target, corres_ij, noise_bound,
            cbar2, estimate_scaling, rotation_gnc_factor, rotation_max_iterations, rotation_cost_threshold);
}


Eigen::Matrix6d GetInformationMatrixFromPointClouds(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_correspondence_distance,
        const Eigen::Matrix4d &transformation) {
    geometry::PointCloud pcd = source;
    if (!transformation.isIdentity()) {
        pcd.Transform(transformation);
    }
    RegistrationResult result;
    geometry::KDTreeFlann target_kdtree(target);
    result = GetRegistrationResultAndCorrespondences(
            pcd, target, target_kdtree, max_correspondence_distance,
            transformation);

    // write q^*
    // see http://redwood-data.org/indoor/registration.html
    // note: I comes first in this implementation
    Eigen::Matrix6d GTG = Eigen::Matrix6d::Zero();
#pragma omp parallel
    {
        Eigen::Matrix6d GTG_private = Eigen::Matrix6d::Zero();
        Eigen::Vector6d G_r_private = Eigen::Vector6d::Zero();
#pragma omp for nowait
        for (int c = 0; c < int(result.correspondence_set_.size()); c++) {
            int t = result.correspondence_set_[c](1);
            double x = target.points_[t](0);
            double y = target.points_[t](1);
            double z = target.points_[t](2);
            G_r_private.setZero();
            G_r_private(1) = z;
            G_r_private(2) = -y;
            G_r_private(3) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = -z;
            G_r_private(2) = x;
            G_r_private(4) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
            G_r_private.setZero();
            G_r_private(0) = y;
            G_r_private(1) = -x;
            G_r_private(5) = 1.0;
            GTG_private.noalias() += G_r_private * G_r_private.transpose();
        }
#pragma omp critical(GetInformationMatrixFromPointClouds)
        { GTG += GTG_private; }
    }
    return GTG;
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d

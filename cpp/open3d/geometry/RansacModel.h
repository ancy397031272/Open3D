#pragma once

#include <omp.h>

#include <Eigen/Core>
#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>

#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Logging.h"
#include "open3d/utility/Parallel.h"
#include "open3d/utility/Random.h"

const double EPS = 1.0e-8;

namespace open3d {
namespace geometry {

void VectorToEigenMatrix(const std::vector<Eigen::Vector3d> &pc,
                         Eigen::Matrix3Xd &new_pc);

double CalcPoint2LineDistance(const Eigen::Vector3d &query,
                              const Eigen::Vector3d &point1,
                              const Eigen::Vector3d &point2);

Eigen::Matrix4d CalcAxisTransform(const Eigen::Vector3d &x_head,
                                  const Eigen::Vector3d &origin,
                                  const Eigen::Vector3d &ref);

Eigen::MatrixX3d Transform(const Eigen::MatrixX3d &points,
                           const Eigen::Matrix4d &transform);
std::tuple<Eigen::RowVector3d, Eigen::Matrix3d> CalcCovarianceMatrix(
        const Eigen::Matrix3d &src);

/// \brief base primitives model
class Model {
public:
    Eigen::VectorXd parameters_;  // The parameters of the current model

    explicit Model(Eigen::VectorXd parameters)
        : parameters_(std::move(parameters)) {}
    Model &operator=(const Model &model) = default;

    Model() = default;
};

/// \brief the plane model is described as [a, b, c, d] => ax + by + cz + d = 0

class Plane : public Model {
public:
    Plane() : Model(Eigen::VectorXd(4)){};
    Plane(const Plane &model) : Model(model) {
        parameters_ = model.parameters_;
    }
    Plane &operator=(const Plane &model) {
        parameters_ = model.parameters_;
        return *this;
    }
};

/// \brief the sphere is describe as [x, y, z, r], where the first three are
/// center and the last is radius

class Sphere : public Model {
public:
    Sphere() : Model(Eigen::VectorXd(4)){};
    Sphere(const Sphere &model) : Model(model) {
        parameters_ = model.parameters_;
    }
    Sphere &operator=(const Sphere &model) {
        parameters_ = model.parameters_;
        return *this;
    }
};

/// \brief the cylinder is describe as [x, y, z, dx, dy, dz, r], where the first
/// three is a point on the cylinder axis and the second three are  direction
/// vector, the last one is radius

class Cylinder : public Model {
public:
    Cylinder() : Model(Eigen::VectorXd(7)){};
    Cylinder(const Cylinder &model) : Model(model) {
        parameters_ = model.parameters_;
    }
    Cylinder &operator=(const Cylinder &model) {
        parameters_ = model.parameters_;
        return *this;
    }
};

/// \brief the cirle3d is describe as [x, y, z, nx, ny, nz, r]
/// Compute circle equation given three 3D points
/// (The three points cannot be colinear)

class Circle3D : public Model {
public:
    Circle3D() : Model(Eigen::VectorXd(7)){};
    Circle3D(const Circle3D &model) : Model(model) {
        parameters_ = model.parameters_;
    }
    Circle3D &operator=(const Circle3D &model) {
        parameters_ = model.parameters_;
        return *this;
    }
};

/// \brief base class of model estimator
class ModelEstimator {
protected:
    explicit ModelEstimator(int minimal_sample)
        : minimal_sample_(minimal_sample) {}

    /// \brief check whether number of input points meet the minimal requirement
    ///
    /// \param num

    bool MinimalCheck(int num) const { return num >= minimal_sample_; }

public:
    /// \brief fit model using least sample points
    ///
    /// \param pc
    /// \param model

    virtual bool MinimalFit(const open3d::geometry::PointCloud &pc,
                            Model &model) const = 0;

    ///\ brief fit model using least square method
    ///
    /// \param pc
    /// \param model

    virtual bool GeneralFit(const open3d::geometry::PointCloud &pc,
                            Model &model) const = 0;

    /// \brief evaluate point distance to model to determine inlier&outlier
    ///
    /// \param query
    /// \param model

    virtual double CalcPointToModelDistance(const Eigen::Vector3d &query,
                                            const Model &model) const = 0;

public:
    int minimal_sample_;
};

class PlaneEstimator : public ModelEstimator {
public:
    PlaneEstimator() : ModelEstimator(3) {}

    bool MinimalFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    bool GeneralFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    double CalcPointToModelDistance(const Eigen::Vector3d &query,
                                    const Model &model) const override;
};

class SphereEstimator : public ModelEstimator {
public:
    SphereEstimator() : ModelEstimator(4) {}

    bool MinimalFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    bool GeneralFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    double CalcPointToModelDistance(const Eigen::Vector3d &query,
                                    const Model &model) const override;

private:
    inline bool ValidationCheck(const open3d::geometry::PointCloud &pc) const;
};

/// \brief Cylinder estimation reference from PCL implementation.

class CylinderEstimator : public ModelEstimator {
public:
    CylinderEstimator() : ModelEstimator(2) {}

    bool MinimalFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    /**
     * @brief the general fit of clinder model is not implemented yet
     * TODO: 1. linear least square method. 2. nonlinear least square method.
     *
     * @param pc
     * @return true
     * @return false
     */
    bool GeneralFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    double CalcPointToModelDistance(const Eigen::Vector3d &query,
                                    const Model &model) const override;
};

/// \brief Circle3D estimation reference from PCL implementation.

class Circle3DEstimator : public ModelEstimator {
public:
    Circle3DEstimator() : ModelEstimator(3) {}

    bool MinimalFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    bool GeneralFit(const open3d::geometry::PointCloud &pc,
                    Model &model) const override;

    double CalcPointToModelDistance(const Eigen::Vector3d &query,
                                    const Model &model) const override;
};

/// \class RANSAC
/// \brief a RANSAC class for model fitting.

/// \tparam ModelEstimator
/// \tparam Model
/// \tparam Sampler

template <typename ModelEstimator, typename Model, typename Sampler>
class RANSAC {
public:
    RANSAC()
        : fitness_(0),
          inlier_rmse_(0),
          max_iteration_(1000),
          probability_(0.9999) {}

    /// \brief Set Point Cloud to be used for RANSAC
    /// \param points
    ///
    void SetPointCloud(const open3d::geometry::PointCloud &pc) {
        if (!pc_.HasPoints()) {
            pc_.Clear();
        }

        pc_ = pc;
    }

    /// \brief set probability to find the best model
    ///
    /// \param probability
    ///
    void SetProbability(double probability) {
        if (probability <= 0 || probability > 1) {
            open3d::utility::LogError("Probability must be > 0 or <= 1.0");
        }
        probability_ = probability;
    }

    /// \brief set maximum iteration, usually used if using parallel ransac
    /// fitting
    ///
    /// ///\param num
    void SetMaxIteration(size_t num) { max_iteration_ = num; }

    /// \brief fit model with given parameters
    ///
    /// \param threshold
    /// \param model
    /// \param inlier_indices

    bool FitModel(double threshold,
                  Model &model,
                  std::vector<size_t> &inlier_indices) {
        Clear();
        const int num_points = pc_.points_.size();
        if (num_points < estimator_.minimal_sample_) {
            open3d::utility::LogError(
                    "Can not fit model due to lack of points");
        }

        return FitModelParallel(threshold, model, inlier_indices);
    }

private:
    void Clear() {
        fitness_ = 0;
        inlier_rmse_ = 0;
    }

    /// \brief refine model using general fitting of estimator, usually is least
    ///  square method.
    ///
    /// \param threshold
    /// \param model
    /// \param inlier_indices

    bool RefineModel(double threshold,
                     Model &model,
                     std::vector<size_t> &inlier_indices) {
        inlier_indices.clear();
        for (size_t i = 0; i < pc_.points_.size(); ++i) {
            const double d =
                    estimator_.CalcPointToModelDistance(pc_.points_[i], model);
            if (d < threshold) {
                inlier_indices.emplace_back(i);
            }
        }

        // improve best model using general fitting
        const auto inliers_pc = pc_.SelectByIndex(inlier_indices);

        return estimator_.GeneralFit(*inliers_pc, model);
    }

    /// \brief Ransac fitting method, the iteration number is varying
    /// with inlier number in each iteration using multithreading.
    ///
    /// \param threshold
    /// \param model
    /// \param inlier_indices

    bool FitModelParallel(double threshold,
                          Model &model,
                          std::vector<size_t> &inlier_indices) {
        const size_t num_points = pc_.points_.size();
        std::vector<size_t> indices_list(num_points);
        std::iota(std::begin(indices_list), std::end(indices_list), 0);

        Model best_model;
        size_t count = 0;
        size_t current_iteration = std::numeric_limits<size_t>::max();
        open3d::utility::random::RandomSampler<size_t> sampler(num_points);
        //#pragma omp parallel for schedule(static)
        // num_threads(utility::EstimateMaxThreads())
        for (size_t i = 0; i < max_iteration_; ++i) {
            if (count > current_iteration) {
                continue;
            }
            const std::vector<size_t> sample_indices =
                    sampler(estimator_.minimal_sample_);
            const auto sample = pc_.SelectByIndex(sample_indices);

            Model model_trial;
            bool ret;
            ret = estimator_.MinimalFit(*sample, model_trial);

            if (!ret) {
                continue;
            }

            const auto result =
                    EvaluateModel(pc_.points_, threshold, model_trial);
            double fitness = std::get<0>(result);
            double inlier_rmse = std::get<1>(result);
            //#pragma omp critical
            {
                // update model if satisfy both fitness and rmse check
                if (fitness > fitness_ ||
                    (fitness == fitness_ && inlier_rmse < inlier_rmse_)) {
                    fitness_ = fitness;
                    inlier_rmse_ = inlier_rmse;
                    best_model = model_trial;

                    if (fitness_ < 1.0) {
                        current_iteration = std::min(
                                log(1 - probability_) /
                                        log(1 -
                                            pow(fitness_,
                                                estimator_.minimal_sample_)),
                                (double)max_iteration_);
                    } else {
                        // Set break_iteration to 0 to force to break the loop.
                        current_iteration = 0;
                    }
                }
                count++;
            }
        }

        open3d::utility::LogInfo(
                "Find best model with {}% inliers and run {} "
                "iterations",
                fitness_ * 100, count);

        const bool ret = RefineModel(threshold, best_model, inlier_indices);
        model = best_model;
        return ret;
    }

    std::tuple<double, double> EvaluateModel(
            const std::vector<Eigen::Vector3d> &points,
            double threshold,
            const Model &model) {
        size_t inlier_num = 0;
        double error = 0;

        for (size_t idx = 0; idx < points.size(); ++idx) {
            const double distance =
                    estimator_.CalcPointToModelDistance(points[idx], model);

            if (distance < threshold) {
                error += distance;
                inlier_num++;
            }
        }

        double fitness;
        double inlier_rmse;

        if (inlier_num == 0) {
            fitness = 0;
            inlier_rmse = 1e+10;
        } else {
            fitness = (double)inlier_num / (double)points.size();
            inlier_rmse = error / std::sqrt((double)inlier_num);
        }

        return std::make_tuple(fitness, inlier_rmse);
    }

private:
    ModelEstimator estimator_;

    open3d::geometry::PointCloud pc_;

    double fitness_;
    double inlier_rmse_;
    size_t max_iteration_;
    double probability_;
};

using RandomIndexSampler = open3d::utility::random::RandomSampler<size_t>;
using RANSACPlane = RANSAC<PlaneEstimator, Plane, RandomIndexSampler>;
using RANSACSphere = RANSAC<SphereEstimator, Sphere, RandomIndexSampler>;
using RANSACCylinder = RANSAC<CylinderEstimator, Cylinder, RandomIndexSampler>;
using RANSACCircle3D = RANSAC<Circle3DEstimator, Circle3D, RandomIndexSampler>;

}  // namespace geometry
}  // namespace open3d

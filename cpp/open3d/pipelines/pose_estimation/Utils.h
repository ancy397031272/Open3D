#pragma once

#include <cmath>
#include <float.h>
#include <omp.h>
#include <string.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "Eigen/Dense"
#include "open3d/geometry/Geometry.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/registration/Feature.h"
#include "open3d/utility/Parallel.h"

namespace open3d {
namespace pipelines {
namespace pose_estimation {
typedef std::shared_ptr<open3d::geometry::Geometry> GeometryPtr;
typedef std::shared_ptr<open3d::geometry::PointCloud> PointCloudPtr;
typedef std::shared_ptr<open3d::geometry::TriangleMesh> TriangleMeshPtr;
typedef std::shared_ptr<open3d::pipelines::registration::Feature> FeaturePtr;

/**
 * @brief Timer for duration measurement.
 *
 */
class Timer {
public:
    void Start() { t0_ = std::chrono::high_resolution_clock::now(); }
    double Stop() {
        const double timestamp =
                std::chrono::duration<double>(
                        std::chrono::high_resolution_clock::now() - t0_)
                        .count();
        return timestamp;
    }

private:
    std::chrono::high_resolution_clock::time_point t0_;
};

/**
 * @brief base sampler class
 *
 * @tparam T
 */
template <typename T>
class Sampler {
public:
    /**
     * @brief pure virtual operator, which define the I/O of this sampler
     *
     * @param sample_size
     * @return std::vector<T>
     */
    virtual std::vector<T> operator()(int sample_size) = 0;
};

/**
 * @brief Extract a random sample of given sample_size from the input indices
 *
 * @tparam T
 */
template <typename T>
class RandomSampler : public Sampler<T> {
public:
    explicit RandomSampler(const int size) : Sampler<T>(), size_(size) {
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    // This operator is usually used in for loop and sample a small subset from
    // original indices
    std::vector<T> operator()(int sample_size) override {
        // Lock this operation when using OpenMP to ensure synchronization
        std::lock_guard<std::mutex> guard(mutex_);

        std::vector<T> sample;
        sample.reserve(sample_size);
        int valid_sample = 0;
        while (valid_sample < sample_size) {
            int idx = rng_() % size_;
            if (std::find(sample.begin(), sample.end(), idx) == sample.end()) {
                sample.push_back(idx);
                valid_sample++;
            }
        }

        return sample;
    }

    // This function is usually called once to sample more than half of original
    // indices
    std::vector<T> SampleWithoutDuplicate(T sample_size) {
        std::vector<T> indices(size_);
        std::iota(indices.begin(), indices.end(), 0);

        for (T i = 0; i < sample_size; ++i) {
            std::swap(indices[i], indices[rng_() % size_]);
        }

        std::vector<T> sample;
        sample.reserve(sample_size);
        for (T idx = 0; idx < sample_size; ++idx) {
            sample.push_back(indices[idx]);
        }

        return sample;
    }

private:
    int size_;
    std::mt19937 rng_;
    std::mutex mutex_;
};

/**
 * @brief perfoem normal consistent, here we have assumption that the point
 * clouds are all in camera coordinate
 *
 * @param pc
 */
inline void NormalConsistent(open3d::geometry::PointCloud &pc) {
    if (!pc.HasNormals()) {
        return;
    } else {
        const int size = pc.points_.size();

//#pragma omp parallel for
        for (int i = 0; i < size; i++) {
            if (pc.points_[i].dot(pc.normals_[i]) > 0) {
                pc.normals_[i] *= -1;
            }
            pc.normals_[i].normalize();
        }
    }
}


/**
 * @brief compute point to line distance
 *
 * @tparam T
 * @param query
 * @param point1
 * @param point2
 * @return T
 */
template <typename T>
inline T CalcPoint2LineDistance(const Eigen::Matrix<T, 3, 1> &query,
                                const Eigen::Matrix<T, 3, 1> &point1,
                                const Eigen::Matrix<T, 3, 1> &point2) {
    const Eigen::Matrix<T, 3, 1> a = query - point1;
    const Eigen::Matrix<T, 3, 1> b = query - point2;
    const Eigen::Matrix<T, 3, 1> c = point2 - point1;

    return a.cross(b).norm() / c.norm();
}

/**
 * @brief convert degree to radian
 *
 * @param angle_deg
 * @return double
 */
template <typename T>
inline T Deg2Rad(const T angle_deg) {
    return angle_deg / 180 * M_PI;
}

/**
 * @brief convert radian to degree
 *
 * @param angle_rad
 * @return double
 */
template <typename T>
inline T Rad2Deg(const T angle_rad) {
    return angle_rad / M_PI * 180;
}



}  // namespace pose_estimation
}  // namespace pipelines
}  // namespace open3d
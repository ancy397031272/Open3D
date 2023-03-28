// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <mutex>
#include <random>

#include "open3d/utility/Logging.h"

namespace open3d {
namespace utility {
namespace random {

/// Set Open3D global random seed.
void Seed(const int seed);

/// Get global singleton random engine.
/// You must also lock the global mutex before calling the engine.
///
/// Example:
/// ```cpp
/// #include "open3d/utility/Random.h"
///
/// {
///     // Put the lock and the call to the engine in the same scope.
///     std::lock_guard<std::mutex> lock(*utility::random::GetMutex());
///     std::shuffle(vals.begin(), vals.end(), *utility::random::GetEngine());
/// }
/// ```
std::mt19937* GetEngine();

/// Get global singleton mutex to protect the engine call. Also see
/// random::GetEngine().
std::mutex* GetMutex();

/// Generate a random uint32.
/// This function is globally seeded by utility::random::Seed().
/// This function is automatically protected by the global random mutex.
uint32_t RandUint32();

/// Generate uniformly distributed random integers in [low, high).
/// This class is globally seeded by utility::random::Seed().
/// This class is a wrapper around std::uniform_int_distribution.
///
/// Example:
/// ```cpp
/// #include "open3d/utility/Random.h"
///
/// // Globally seed Open3D. This will affect all random functions.
/// utility::random::Seed(0);
///
/// // Generate a random int in [0, 100).
/// utility::random::UniformIntGenerator<int> gen(0, 100);
/// for (size_t i = 0; i < 10; i++) {
///     std::cout << gen() << std::endl;
/// }
/// ```
template <typename T>
class UniformIntGenerator {
public:
    /// Generate uniformly distributed random integer from
    /// [low, low + 1, ... high - 1].
    ///
    /// \param low The lower bound (inclusive).
    /// \param high The upper bound (exclusive). \p high must be > \p low.
    UniformIntGenerator(const T low, const T high) : distribution_(low, high) {
        if (low < 0) {
            utility::LogError("low must be > 0, but got {}.", low);
        }
        if (low >= high) {
            utility::LogError("low must be < high, but got low={} and high={}.",
                              low, high);
        }
    }

    /// Call this to generate a uniformly distributed integer.
    T operator()() {
        std::lock_guard<std::mutex> lock(*GetMutex());
        return distribution_(*GetEngine());
    }

protected:
    std::uniform_int_distribution<T> distribution_;
};

/// Generate uniformly distributed floating point values in [low, high).
/// This class is globally seeded by utility::random::Seed().
/// This class is a wrapper around std::uniform_real_distribution.
///
/// Example:
/// ```cpp
/// #include "open3d/utility/Random.h"
///
/// // Globally seed Open3D. This will affect all random functions.
/// utility::random::Seed(0);
///
/// // Generate a random double in [0, 1).
/// utility::random::UniformRealGenerator<double> gen(0, 1);
/// for (size_t i = 0; i < 10; i++) {
///    std::cout << gen() << std::endl;
/// }
/// ```
template <typename T>
class UniformRealGenerator {
public:
    /// Generate uniformly distributed floating point values in [low, high).
    ///
    /// \param low The lower bound (inclusive).
    /// \param high The upper bound (exclusive).
    UniformRealGenerator(const T low = 0.0, const T high = 1.0)
        : distribution_(low, high) {
        if (low >= high) {
            utility::LogError("low must be < high, but got low={} and high={}.",
                              low, high);
        }
    }

    /// Call this to generate a uniformly distributed floating point value.
    T operator()() {
        std::lock_guard<std::mutex> lock(*GetMutex());
        return distribution_(*GetEngine());
    }

protected:
    std::uniform_real_distribution<T> distribution_;
};

/// Generate normally distributed floating point values with mean and std.
/// This class is globally seeded by utility::random::Seed().
/// This class is a wrapper around std::normal_distribution.
///
/// Example:
/// ```cpp
/// #include "open3d/utility/Random.h"
///
/// // Globally seed Open3D. This will affect all random functions.
/// utility::random::Seed(0);
///
/// // Generate a random double with mean 0 and std 1.
/// utility::random::NormalGenerator<double> gen(0, 1);
/// for (size_t i = 0; i < 10; i++) {
///     std::cout << gen() << std::endl;
/// }
/// ```
template <typename T>
class NormalGenerator {
public:
    /// Generate normally distributed floating point value with mean and std.
    ///
    /// \param mean The mean of the distribution.
    /// \param stddev The standard deviation of the distribution.
    NormalGenerator(const T mean = 0.0, const T stddev = 1.0)
        : distribution_(mean, stddev) {
        if (stddev <= 0) {
            utility::LogError("stddev must be > 0, but got {}.", stddev);
        }
    }

    /// Call this to generate a normally distributed floating point value.
    T operator()() {
        std::lock_guard<std::mutex> lock(*GetMutex());
        return distribution_(*GetEngine());
    }

protected:
    std::normal_distribution<T> distribution_;
};

/// \brief Extract a random sample of given sample_size from the input indices
///
/// \tparam T

template <typename T>
class RandomSampler {
public:
    explicit RandomSampler(const size_t size) : size_(size) {
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    // This operator is usually used in for loop and sample a small subset from
    // original indices
    std::vector<T> operator()(size_t sample_size) {
        // Lock this operation when using OpenMP to ensure synchronization
        std::lock_guard<std::mutex> guard(mutex_);

        std::vector<T> sample;
        sample.reserve(sample_size);
        size_t valid_sample = 0;
        while (valid_sample < sample_size) {
            size_t idx = rng_() % size_;
            if (std::find(sample.begin(), sample.end(), idx) == sample.end()) {
                sample.push_back(idx);
                valid_sample++;
            }
        }

        return sample;
    }

    // This function is usually called once to sample more than half of original
    // indices
    std::vector<T> SampleWithoutDuplicate(size_t sample_size) {
        std::vector<T> indices(size_);
        std::iota(indices.begin(), indices.end(), 0);

        for (size_t i = 0; i < sample_size; ++i) {
            std::swap(indices[i], indices[rng_() % size_]);
        }

        std::vector<T> sample;
        sample.reserve(sample_size);
        for (int idx = 0; idx < sample_size; ++idx) {
            sample.push_back(indices[idx]);
        }

        return sample;
    }

private:
    size_t size_;
    std::mt19937 rng_;
    std::mutex mutex_;
};
}  // namespace random
}  // namespace utility
}  // namespace open3d

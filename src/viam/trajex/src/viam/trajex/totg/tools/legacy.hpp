#pragma once

#include <cmath>
#include <cstddef>
#include <list>
#include <stdexcept>

#include <Eigen/Dense>

namespace viam::trajex::totg::legacy {

///
/// Calls a function at each quantized sample point over a duration.
///
/// Computes evenly-spaced sample times from duration and frequency, skipping
/// the initial t=0 sample. The callback receives the sample time and the
/// uniform timestep between samples.
///
/// @param duration_sec Total duration to sample over (must be positive)
/// @param sampling_frequency_hz Number of samples per second (must be positive)
/// @param f Callable invoked as f(double time, double timestep) for each sample
/// @throws std::invalid_argument if inputs are non-positive or produce too many samples
///
template <typename Func>
void for_each_sample(double duration_sec, double sampling_frequency_hz, Func&& f) {
    if (duration_sec <= 0.0 || sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz are not both positive");
    }
    static constexpr std::size_t k_max_samples = 1000000;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz exceed the maximum allowable samples");
    }

    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);
    const double step = duration_sec / static_cast<double>(num_samples - 1);

    for (std::size_t i = 1; i < num_samples - 1; ++i) {
        f(static_cast<double>(i) * step, step);
    }

    // Last sample uses exactly the duration to avoid accumulated floating point drift
    f(duration_sec, step);
}

///
/// Check if point is within tolerance cylinder around line segment.
///
/// Returns false if the segment is degenerate (start == end) or if the
/// point projects outside the segment bounds (non-monotonic movement).
/// Tolerance is interpreted as cylinder diameter (radius = tolerance/2).
///
/// @param point Point to test
/// @param line_start Start of line segment
/// @param line_end End of line segment
/// @param tolerance Diameter of tolerance cylinder
/// @return true if point is within tolerance, false otherwise
///
bool within_colinearization_tolerance(const Eigen::VectorXd& point,
                                      const Eigen::VectorXd& line_start,
                                      const Eigen::VectorXd& line_end,
                                      double tolerance);

///
/// Apply colinearization to waypoint list, removing redundant waypoints.
///
/// Implements waypoint coalescing by extending a tolerance cylinder from each
/// anchor waypoint, removing intermediate waypoints that remain within
/// tolerance of the straight-line segment. When extending the cylinder, all
/// previously skipped waypoints are revalidated to prevent drift.
///
/// @param waypoints List of waypoints to coalesce (modified in-place)
/// @param tolerance Diameter of tolerance cylinder (in radians)
///
void apply_colinearization(std::list<Eigen::VectorXd>& waypoints, double tolerance);

}  // namespace viam::trajex::totg::legacy

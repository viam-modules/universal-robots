// Test utility functions shared across multiple test files
#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg::test {

/// Check if two configurations are close within tolerance
bool configs_close(const xt::xarray<double>& a, const xt::xarray<double>& b, double tolerance = 1e-6);

/// Verify path visits all waypoints within max_deviation
void verify_path_visits_waypoints(const path& p, const xt::xarray<double>& waypoints, double max_deviation);

/// Build a segment-type string: 'L' per linear segment, 'C' per circular, left to right.
/// Example: a 3-waypoint path with a single circular blend yields "LCL".
std::string path_type_sequence(const path& p);

}  // namespace viam::trajex::totg::test

// Equality operators for trajectory event types. Placed in viam::trajex::totg so
// ADL finds them for types nested in trajectory, which lives in that namespace.
// They live in test_utils.hpp rather than the library headers to avoid committing
// to them as part of the public ABI.

namespace viam::trajex::totg {

inline bool operator==(const trajectory::integration_observer::started_forward_event& a,
                       const trajectory::integration_observer::started_forward_event& b) noexcept {
    return a.start == b.start;
}

inline bool operator==(const trajectory::integration_observer::limit_hit_event& a,
                       const trajectory::integration_observer::limit_hit_event& b) noexcept {
    return a.breach == b.breach && a.s_dot_max_acc == b.s_dot_max_acc && a.s_dot_max_vel == b.s_dot_max_vel;
}

inline bool operator==(const trajectory::integration_observer::started_backward_event& a,
                       const trajectory::integration_observer::started_backward_event& b) noexcept {
    return a.start == b.start && a.kind == b.kind;
}

inline bool operator==(const trajectory::integration_observer::splice_event& a,
                       const trajectory::integration_observer::splice_event& b) noexcept {
    // We need a custom comparator here, because for splice events, we can have s_ddot = NaN for the last element.
    return std::ranges::equal(a.pruned, b.pruned, [](const auto& lhs, const auto& rhs) {
        if (lhs.time != rhs.time) {
            return false;
        }
        if (lhs.s != rhs.s) {
            return false;
        }
        if (lhs.s_dot != rhs.s_dot) {
            return false;
        }
        if (!std::isnan(static_cast<double>(lhs.s_ddot)) && !std::isnan(static_cast<double>(lhs.s_ddot)) && (lhs.s_ddot != rhs.s_ddot)) {
            return false;
        }
        return true;
    });
}

}  // namespace viam::trajex::totg

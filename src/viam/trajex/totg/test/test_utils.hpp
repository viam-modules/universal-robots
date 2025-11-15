// Test utility functions shared across multiple test files
#pragma once

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

namespace viam::trajex::totg {
class path;  // Forward declaration
}

namespace viam::trajex::totg::test {

/// Check if two configurations are close within tolerance
bool configs_close(const xt::xarray<double>& a, const xt::xarray<double>& b, double tolerance = 1e-6);

/// Verify path visits all waypoints within max_deviation
void verify_path_visits_waypoints(const path& p, const xt::xarray<double>& waypoints, double max_deviation);

}  // namespace viam::trajex::totg::test

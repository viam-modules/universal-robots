// Test utility function implementations
#include "test_utils.hpp"

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_length.hpp>

#if __has_include(<xtensor/reducers/xnorm.hpp>)
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xnorm.hpp>
#endif

#include <boost/test/unit_test.hpp>

namespace viam::trajex::totg::test {

bool configs_close(const xt::xarray<double>& a, const xt::xarray<double>& b, double tolerance) {
    if (a.shape(0) != b.shape(0)) {
        return false;
    }
    for (size_t i = 0; i < a.shape(0); ++i) {
        if (std::abs(a(i) - b(i)) > tolerance) {
            return false;
        }
    }
    return true;
}

void verify_path_visits_waypoints(const path& p, const xt::xarray<double>& waypoints, double max_deviation) {
    using namespace viam::trajex;

    for (size_t i = 0; i < waypoints.shape(0); ++i) {
        const xt::xarray<double> waypoint = xt::view(waypoints, i, xt::all());

        // Find minimum distance from waypoint to any point on path
        double min_distance = std::numeric_limits<double>::max();
        // Use dense sampling: 1000 samples to ensure we don't miss waypoints
        const size_t num_samples = 1000;

        for (size_t j = 0; j <= num_samples; ++j) {
            const double fraction = static_cast<double>(j) / static_cast<double>(num_samples);
            const arc_length s{static_cast<double>(p.length()) * fraction};
            const auto config = p.configuration(s);

            // Compute distance
            const auto diff = waypoint - config;
            const double distance = xt::norm_l2(diff)();
            min_distance = std::min(min_distance, distance);
        }

        // Waypoint should be within max_deviation of some point on path
        // Allow small numerical tolerance for floating point and sampling error
        BOOST_CHECK_LE(min_distance, max_deviation + 1e-4);
    }
}

}  // namespace viam::trajex::totg::test

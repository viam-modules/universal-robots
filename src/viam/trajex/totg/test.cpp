#define BOOST_TEST_MODULE trajex_test

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>

#if __has_include(<xtensor/reducers/xnorm.hpp>)
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xnorm.hpp>
#endif

#include <numbers>

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(waypoint_accumulator_tests)

BOOST_AUTO_TEST_CASE(construct_with_single_waypoint) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    BOOST_CHECK_NO_THROW(waypoint_accumulator{waypoints});
}

BOOST_AUTO_TEST_CASE(construct_with_multiple_waypoints) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    BOOST_CHECK_NO_THROW(waypoint_accumulator{waypoints});
}

BOOST_AUTO_TEST_CASE(add_waypoints) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints1 = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints1};
    BOOST_CHECK_EQUAL(acc.size(), 1);
    BOOST_CHECK_EQUAL(acc.dof(), 3);

    xt::xarray<double> waypoints2 = {{4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    BOOST_CHECK_NO_THROW(acc.add_waypoints(waypoints2));
    BOOST_CHECK_EQUAL(acc.size(), 3);
}

BOOST_AUTO_TEST_CASE(validates_dimension) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints_1d = {1.0, 2.0, 3.0};
    BOOST_CHECK_THROW(waypoint_accumulator{waypoints_1d}, std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_dof_consistency) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints};

    xt::xarray<double> waypoints_wrong_dof = {{4.0, 5.0}};
    BOOST_CHECK_THROW(acc.add_waypoints(waypoints_wrong_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(iterator_interface) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    // Member function calls
    BOOST_CHECK_NO_THROW(acc.begin());
    BOOST_CHECK_NO_THROW(acc.end());
    BOOST_CHECK_NO_THROW(acc.cbegin());
    BOOST_CHECK_NO_THROW(acc.cend());

    // Verify they return the right type and work
    auto it1 = acc.begin();
    auto it2 = acc.end();
    BOOST_CHECK(it1 != it2);

    // ADL calls (unqualified)
    using std::begin;
    using std::end;
    BOOST_CHECK_NO_THROW(begin(acc));
    BOOST_CHECK_NO_THROW(end(acc));

    // std::cbegin/cend (qualified)
    BOOST_CHECK_NO_THROW(std::cbegin(acc));
    BOOST_CHECK_NO_THROW(std::cend(acc));

    // Verify range-based for works
    int count = 0;
    for (const auto& wp : acc) {
        (void)wp;  // Suppress unused warning
        ++count;
    }
    BOOST_CHECK_EQUAL(count, 2);
}

BOOST_AUTO_TEST_CASE(bounds_checked_access) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    // Valid access should work
    BOOST_CHECK_NO_THROW(acc.at(0));
    BOOST_CHECK_NO_THROW(acc.at(1));

    // Out of bounds should throw
    BOOST_CHECK_THROW(acc.at(2), std::out_of_range);
    BOOST_CHECK_THROW(acc.at(100), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(unchecked_access) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    // Unchecked access should work
    BOOST_CHECK_NO_THROW(acc[0]);
    BOOST_CHECK_NO_THROW(acc[1]);
}

BOOST_AUTO_TEST_CASE(empty_check) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints};

    BOOST_CHECK(!acc.empty());
    BOOST_CHECK_EQUAL(acc.size(), 1);
}

BOOST_AUTO_TEST_CASE(dof_consistency_after_add) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints1 = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints1};
    BOOST_CHECK_EQUAL(acc.dof(), 3);

    xt::xarray<double> waypoints2 = {{4.0, 5.0, 6.0}};
    acc.add_waypoints(waypoints2);
    BOOST_CHECK_EQUAL(acc.dof(), 3);  // DOF should remain consistent
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(path_tests)

// Helper function to check if two configurations are close
bool configs_close(const xt::xarray<double>& a, const xt::xarray<double>& b, double tolerance = 1e-6) {
    if (a.shape(0) != b.shape(0)) return false;
    for (size_t i = 0; i < a.shape(0); ++i) {
        if (std::abs(a(i) - b(i)) > tolerance) return false;
    }
    return true;
}

// Helper to verify path visits all waypoints within max_deviation
void verify_path_visits_waypoints(const viam::trajex::totg::path& p, const xt::xarray<double>& waypoints, double max_deviation) {
    using namespace viam::trajex;
    using namespace viam::trajex::totg;

    for (size_t i = 0; i < waypoints.shape(0); ++i) {
        xt::xarray<double> waypoint = xt::view(waypoints, i, xt::all());

        // Find minimum distance from waypoint to any point on path
        double min_distance = std::numeric_limits<double>::max();
        // Use dense sampling: 1000 samples to ensure we don't miss waypoints
        const size_t num_samples = 1000;

        for (size_t j = 0; j <= num_samples; ++j) {
            const double fraction = static_cast<double>(j) / static_cast<double>(num_samples);
            const arc_length s{static_cast<double>(p.length()) * fraction};
            auto config = p.configuration(s);

            // Compute distance
            auto diff = waypoint - config;
            double distance = xt::norm_l2(diff)();
            min_distance = std::min(min_distance, distance);
        }

        // Waypoint should be within max_deviation of some point on path
        // Allow small numerical tolerance for floating point and sampling error
        BOOST_CHECK_LE(min_distance, max_deviation + 1e-4);
    }
}

BOOST_AUTO_TEST_CASE(create_path) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    BOOST_CHECK_NO_THROW(static_cast<void>(path::create(acc)));
}

BOOST_AUTO_TEST_CASE(validates_max_deviation) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    // Negative blend deviation should throw
    BOOST_CHECK_THROW(static_cast<void>(path::create(acc, path::options{}.set_max_blend_deviation(-0.1))), std::invalid_argument);

    // Negative linear deviation should throw
    BOOST_CHECK_THROW(static_cast<void>(path::create(acc, path::options{}.set_max_linear_deviation(-0.1))), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(requires_minimum_two_waypoints) {
    using namespace viam::trajex::totg;

    // Single waypoint should throw
    xt::xarray<double> single = {{1.0, 2.0, 3.0}};
    BOOST_CHECK_THROW(static_cast<void>(path::create(single)), std::invalid_argument);

    // Two waypoints should work
    xt::xarray<double> two = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    BOOST_CHECK_NO_THROW(static_cast<void>(path::create(two)));
}

BOOST_AUTO_TEST_CASE(create_from_array_vs_accumulator) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};

    // Create from accumulator
    waypoint_accumulator acc{waypoints};
    path p1 = path::create(acc);

    // Create from array directly
    path p2 = path::create(waypoints);

    // Should produce equivalent paths
    BOOST_CHECK_EQUAL(p1.dof(), p2.dof());
    BOOST_CHECK_EQUAL(p1.size(), p2.size());
    BOOST_CHECK_EQUAL(static_cast<double>(p1.length()), static_cast<double>(p2.length()));
}

BOOST_AUTO_TEST_CASE(path_dof_matches_waypoints) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}};
    path p = path::create(waypoints);

    BOOST_CHECK_EQUAL(p.dof(), 4);
}

BOOST_AUTO_TEST_CASE(linear_path_length) {
    using namespace viam::trajex::totg;

    // Simple 3-4-5 triangle in 2D
    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints);

    // Two segments: 3.0 + 4.0 = 7.0
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 7.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_path_segment_count) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    path p = path::create(waypoints);

    // 3 waypoints => 2 segments
    BOOST_CHECK_EQUAL(p.size(), 2);
}

BOOST_AUTO_TEST_CASE(segment_lookup_at_start) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints);

    // Query at start of path
    auto view = p(arc_length{0.0});

    BOOST_CHECK_EQUAL(static_cast<double>(view.start()), 0.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.end()), 3.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.length()), 3.0);
    BOOST_CHECK(view.is<path::segment::linear>());
}

BOOST_AUTO_TEST_CASE(segment_lookup_at_boundary) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints);

    // Query at boundary between segments (exactly at arc_length 3.0)
    auto view = p(arc_length{3.0});

    // Should return second segment
    BOOST_CHECK_EQUAL(static_cast<double>(view.start()), 3.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.end()), 7.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.length()), 4.0);
}

BOOST_AUTO_TEST_CASE(segment_lookup_mid_segment) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints);

    // Query in middle of first segment
    auto view = p(arc_length{1.5});

    BOOST_CHECK_EQUAL(static_cast<double>(view.start()), 0.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.end()), 3.0);
    BOOST_CHECK(view.is<path::segment::linear>());
}

BOOST_AUTO_TEST_CASE(segment_lookup_at_end) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints);

    // Query at end of path
    auto view = p(arc_length{7.0});

    // Should return last segment
    BOOST_CHECK_EQUAL(static_cast<double>(view.start()), 3.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view.end()), 7.0);
}

BOOST_AUTO_TEST_CASE(segment_lookup_out_of_range) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}};
    path p = path::create(waypoints);

    // Query beyond path length should throw
    BOOST_CHECK_THROW(p(arc_length{10.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(multiple_segments_lookup) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Four waypoints => three segments (1D path)
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({4, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 1.0;
    waypoints(2, 0) = 3.0;
    waypoints(3, 0) = 6.0;
    path p = path::create(waypoints);

    BOOST_CHECK_EQUAL(p.size(), 3);
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 6.0, 1e-6);

    // First segment [0, 1)
    auto v1 = p(arc_length{0.5});
    BOOST_CHECK_EQUAL(static_cast<double>(v1.start()), 0.0);
    BOOST_CHECK_EQUAL(static_cast<double>(v1.end()), 1.0);

    // Second segment [1, 3)
    auto v2 = p(arc_length{2.0});
    BOOST_CHECK_EQUAL(static_cast<double>(v2.start()), 1.0);
    BOOST_CHECK_EQUAL(static_cast<double>(v2.end()), 3.0);

    // Third segment [3, 6)
    auto v3 = p(arc_length{5.0});
    BOOST_CHECK_EQUAL(static_cast<double>(v3.start()), 3.0);
    BOOST_CHECK_EQUAL(static_cast<double>(v3.end()), 6.0);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_collinear_points) {
    using namespace viam::trajex::totg;

    // Waypoints that form a perfect line - should coalesce fully
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.001));  // Tight tolerance

    // All waypoints are collinear, all within tolerance of anchor→last line
    // Should coalesce to just first and last
    BOOST_CHECK_EQUAL(p.size(), 1U);  // Single segment from (0,0) to (3,0)
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 3.0, 0.001);

    // Verify endpoints
    using viam::trajex::arc_length;
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{3.0, 0.0}));

    // Verify all waypoints visited
    verify_path_visits_waypoints(p, waypoints, 0.001);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_no_reduction) {
    using namespace viam::trajex::totg;

    // Waypoints where each is outside tolerance - no reduction should occur
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.5},  // Far from line (0,0) to (2,0): deviation = 0.5
        {2.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));  // Tolerance smaller than deviations

    // Middle waypoint is 0.5 units from line, outside 0.1 tolerance
    // Should keep all waypoints: 2 segments
    BOOST_CHECK_EQUAL(p.size(), 2U);

    // Verify segment endpoints correspond to actual waypoints
    auto seg1 = *p.begin();
    auto seg1_start = seg1.configuration(seg1.start());
    auto seg1_end = seg1.configuration(seg1.end());
    BOOST_CHECK(configs_close(seg1_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(seg1_end, xt::xarray<double>{1.0, 0.5}));

    auto it = p.begin();
    ++it;
    auto seg2 = *it;
    auto seg2_start = seg2.configuration(seg2.start());
    auto seg2_end = seg2.configuration(seg2.end());
    BOOST_CHECK(configs_close(seg2_start, xt::xarray<double>{1.0, 0.5}));
    BOOST_CHECK(configs_close(seg2_end, xt::xarray<double>{2.0, 0.0}));

    // Verify all waypoints visited
    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_with_reduction) {
    using namespace viam::trajex::totg;

    // Waypoints with some within tolerance, some outside
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},   // On the line from (0,0) to (3,0), within tolerance
        {2.0, 0.05},  // Very close to line, within tolerance
        {3.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));  // 0.1 unit tolerance

    // Intermediate waypoints are within tube from (0,0) to (3,0), should be skipped
    // Should coalesce to single segment: (0,0)→(3,0)
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_all_within_tolerance) {
    using namespace viam::trajex::totg;

    // All intermediate waypoints within tolerance tube
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.05},
        {2.0, -0.05},
        {3.0, 0.03},
        {4.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));  // All deviations < 0.1

    // All intermediate points within tube, should coalesce to single segment
    BOOST_CHECK_EQUAL(p.size(), 1U);
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 4.0, 0.1);  // Approximately straight line
}

BOOST_AUTO_TEST_CASE(tube_coalescing_preserves_endpoints) {
    using namespace viam::trajex::totg;

    // Verify first and last waypoints always preserved
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.5, 0.0},  // All within tolerance
        {1.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(1.0));  // Large tolerance

    // Should still have at least 1 segment (first to last)
    BOOST_REQUIRE_GE(p.size(), 1U);

    // Check path starts at first waypoint and ends at last
    auto first_seg = *p.begin();
    auto config_start = first_seg.configuration(first_seg.start());
    BOOST_CHECK_CLOSE(config_start(0), 0.0, 0.001);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 0.001);

    auto config_end = first_seg.configuration(first_seg.end());
    BOOST_CHECK_CLOSE(config_end(0), 1.0, 0.001);
    BOOST_CHECK_CLOSE(config_end(1), 0.0, 0.001);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_zero_deviation_no_coalescing) {
    using namespace viam::trajex::totg;

    // With max_deviation = 0, no coalescing should occur
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {2.0, 0.0}
    };

    path p = path::create(waypoints);

    // Should have segment between each consecutive pair
    BOOST_CHECK_EQUAL(p.size(), 2U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_two_waypoints_only) {
    using namespace viam::trajex::totg;

    // With only 2 waypoints, no intermediate points to coalesce
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(1.0));  // Large tolerance shouldn't matter

    // Should have single segment regardless of tolerance
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_three_waypoints) {
    using namespace viam::trajex::totg;

    // Minimal case for actual coalescing: 3 waypoints
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.01},  // Slightly off line
        {2.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));  // Large enough tolerance

    // Middle point should be coalesced
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_duplicate_waypoints) {
    using namespace viam::trajex::totg;

    // Consecutive duplicate waypoints
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 0.0},  // Duplicate
        {2.0, 0.0}
    };

    // Should handle gracefully (duplicates should be coalesced away)
    BOOST_CHECK_NO_THROW(path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.01)));
}

BOOST_AUTO_TEST_CASE(tube_coalescing_zigzag_pattern) {
    using namespace viam::trajex::totg;

    // Zigzag pattern: alternating above/below centerline
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.05},   // Above
        {2.0, -0.05},  // Below
        {3.0, 0.05},   // Above
        {4.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));  // All deviations within tolerance

    // All intermediate points within tube, should coalesce to single segment
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_higher_dimensions) {
    using namespace viam::trajex::totg;

    // Test in 6D (typical robot arm)
    xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.01, 0.01, 0.01, 0.01, 0.01},  // Small deviation in all dims
        {2.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // Should handle multi-dimensional case
    BOOST_CHECK_NO_THROW(path p2 = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1)));
}

BOOST_AUTO_TEST_CASE(tube_coalescing_projection_beyond_segment) {
    using namespace viam::trajex::totg;

    // Current waypoint projects beyond the next waypoint
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.5, 0.0},  // Projects to middle of (0,0)→(1,0)
        {1.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.01));

    // Middle point on the line, should coalesce
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_multiple_consecutive_within_tolerance) {
    using namespace viam::trajex::totg;

    // Multiple consecutive points within tolerance
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.02},
        {2.0, 0.03},
        {3.0, 0.01},
        {4.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // All within tolerance, should coalesce to single segment
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_direction_reversal) {
    using namespace viam::trajex::totg;

    // Waypoints that reverse direction
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {2.0, 0.0},  // Move forward
        {1.0, 0.0}   // Reverse back (projection < 0)
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(1.0));  // Large tolerance

    // Middle waypoint reverses direction, must be kept despite being on line
    // Should have 2 segments: (0,0)→(2,0) and (2,0)→(1,0)
    BOOST_CHECK_EQUAL(p.size(), 2U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_projection_beyond_next) {
    using namespace viam::trajex::totg;

    // Current waypoint projects beyond next waypoint
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {2.0, 0.0},  // Projects beyond (0,0)→(1,0)
        {1.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(1.0));  // Large tolerance

    // Middle waypoint projects beyond segment, must be kept
    BOOST_CHECK_EQUAL(p.size(), 2U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_backward_then_forward) {
    using namespace viam::trajex::totg;

    // Path that goes forward, back, then forward again
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {0.5, 0.0},  // Backward
        {2.0, 0.0}   // Forward again
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // All direction changes must be preserved
    // Should have 3 segments
    BOOST_CHECK_EQUAL(p.size(), 3U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_right_angle_turn) {
    using namespace viam::trajex::totg;

    // 90-degree turn with intermediate point near corner
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.05},  // Slightly off x-axis
        {1.0, 1.0}    // Turn up
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // Middle point is within tolerance of line from (0,0) to (1,1)
    // Should check if it gets coalesced correctly
    BOOST_CHECK_GE(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_projection_at_boundaries) {
    using namespace viam::trajex::totg;

    // Test projection exactly at anchor (projection ≈ 0)
    xt::xarray<double> waypoints1 = {
        {0.0, 0.0},
        {0.0, 0.05},  // Perpendicular to line (0,0)→(1,0), projects at 0
        {1.0, 0.0}
    };

    path p1 = path::create(waypoints1, path::options{}.set_max_linear_deviation(0.1));
    // Should keep middle point (projection at boundary)
    BOOST_CHECK_GE(p1.size(), 1U);

    // Test projection exactly at next (projection ≈ segment_length)
    xt::xarray<double> waypoints2 = {
        {0.0, 0.0},
        {1.0, 0.05},  // Near end of segment, projects ≈ 1.0
        {1.0, 0.0}
    };

    path p2 = path::create(waypoints2, path::options{}.set_max_linear_deviation(0.1));
    // Should handle gracefully
    BOOST_CHECK_GE(p2.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_all_waypoints_identical) {
    using namespace viam::trajex::totg;

    // Degenerate case: all waypoints at same location
    xt::xarray<double> waypoints = {
        {1.0, 1.0},
        {1.0, 1.0},
        {1.0, 1.0}
    };

    // Should handle gracefully (segment_length check should catch)
    // Will try to create segments but they'll have zero length
    BOOST_CHECK_THROW(static_cast<void>(path::create(waypoints, path::options{}.set_max_linear_deviation(0.1))), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_complex_pattern) {
    using namespace viam::trajex::totg;

    // Complex: multiple regions with different coalescing behavior
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.5, 0.01},  // Within tolerance of (0,0)→(1,0)
        {1.0, 0.0},
        {1.0, 0.5},   // Sharp turn up (outside tolerance)
        {1.5, 0.51},  // Within tolerance of (1,0.5)→(2,0.5)
        {2.0, 0.5}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.05));

    // (0.5,0.01) coalesces: (0,0)→(1,0)
    // Sharp turn at (1,0.5) prevents coalescing
    // (1.5,0.51) coalesces: (1,0.5)→(2,0.5)
    // Expect: (0,0)→(1,0)→(1,0.5)→(2,0.5) = 3 segments
    BOOST_CHECK_EQUAL(p.size(), 3U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_first_waypoint_duplicate) {
    using namespace viam::trajex::totg;

    // First two waypoints identical
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.0, 0.0},  // Duplicate start
        {1.0, 0.0}
    };

    // Should handle - duplicate should be coalesced
    BOOST_CHECK_NO_THROW(static_cast<void>(path::create(waypoints, path::options{}.set_max_linear_deviation(0.1))));
}

BOOST_AUTO_TEST_CASE(tube_coalescing_last_waypoint_duplicate) {
    using namespace viam::trajex::totg;

    // Last two waypoints identical
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 0.0}  // Duplicate end
    };

    // Tube coalescing handles duplicate: segment_length < 1e-10 causes continue
    // After coalescing: (0,0), (1,0) which creates valid segment
    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p.size(), 1U);  // Single segment (0,0)→(1,0)
}

BOOST_AUTO_TEST_CASE(tube_coalescing_without_blends) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};

    // Non-zero max_deviation now works (coalescing without blends)
    // Creates path with sharp corners instead of blended curves
    BOOST_CHECK_NO_THROW(path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1)));

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p.size(), 1U);  // Single segment (2 waypoints)
}

BOOST_AUTO_TEST_CASE(tube_coalescing_very_small_tolerance) {
    using namespace viam::trajex::totg;

    // Test with tolerance near machine epsilon
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 1e-15},  // Tiny deviation (near machine epsilon)
        {2.0, 0.0}
    };

    // With tiny tolerance, middle point should be kept
    path p1 = path::create(waypoints, path::options{}.set_max_linear_deviation(1e-16));
    BOOST_CHECK_EQUAL(p1.size(), 2U);

    // With slightly larger tolerance, middle point should coalesce
    path p2 = path::create(waypoints, path::options{}.set_max_linear_deviation(1e-14));
    BOOST_CHECK_EQUAL(p2.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_multiple_consecutive_duplicates) {
    using namespace viam::trajex::totg;

    // Multiple duplicates in a row
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 0.0},  // Duplicate 1
        {1.0, 0.0},  // Duplicate 2
        {1.0, 0.0},  // Duplicate 3
        {2.0, 0.0}
    };

    // All duplicates get skipped (segment_length < 1e-10)
    // After removing duplicates: (0,0), (1,0), (2,0)
    // Then (1,0) is on the line from (0,0)→(2,0), so it coalesces
    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p.size(), 1U);  // Fully coalesced to single segment
}

BOOST_AUTO_TEST_CASE(tube_coalescing_alternating_duplicates) {
    using namespace viam::trajex::totg;

    // Pattern: unique, duplicate, unique, duplicate
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 1.0},
        {1.0, 1.0},  // Duplicate of previous
        {2.0, 2.0},
        {2.0, 2.0}   // Duplicate of previous
    };

    // Duplicates get skipped, leaving (0,0), (1,1), (2,2)
    // These are collinear, so they coalesce to single segment
    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p.size(), 1U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_near_collinear_accumulating_error) {
    using namespace viam::trajex::totg;

    // Many points almost on a line with small perpendicular deviations
    // that alternate above/below the line
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.01},   // Slightly above
        {2.0, -0.01},  // Slightly below
        {3.0, 0.01},   // Slightly above
        {4.0, -0.01},  // Slightly below
        {5.0, 0.0}
    };

    // With tolerance 0.02, all should coalesce to single segment
    path p1 = path::create(waypoints, path::options{}.set_max_linear_deviation(0.02));
    BOOST_CHECK_EQUAL(p1.size(), 1U);

    // With tight tolerance 0.005, none should coalesce
    path p2 = path::create(waypoints, path::options{}.set_max_linear_deviation(0.005));
    BOOST_CHECK_EQUAL(p2.size(), 5U);

    // With medium tolerance 0.015, some points coalesce but not all
    // (1,0.01) is within 0.015 of (0,0)→(2,-0.01) line: distance ≈ 0.01
    // (2,-0.01) becomes new anchor
    // (3,0.01) vs (2,-0.01)→(4,-0.01): perpendicular distance ≈ 0.02 > 0.015
    // So (3,0.01) is kept
    path p3 = path::create(waypoints, path::options{}.set_max_linear_deviation(0.015));
    BOOST_CHECK_EQUAL(p3.size(), 3U);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_projection_exact_boundaries) {
    using namespace viam::trajex::totg;

    // Test projection exactly at 0.0 (perpendicular to anchor)
    xt::xarray<double> waypoints1 = {
        {0.0, 0.0},
        {0.0, 0.01},  // Projection exactly at anchor (projection = 0)
        {1.0, 0.0}
    };

    // Point perpendicular to anchor has projection = 0, which is NOT < 0
    // So it passes the bounds check and is evaluated for perpendicular distance
    // Distance is 0.01 < 0.1, so it coalesces
    path p1 = path::create(waypoints1, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p1.size(), 1U);

    // Test with tolerance too small
    path p1b = path::create(waypoints1, path::options{}.set_max_linear_deviation(0.001));
    BOOST_CHECK_EQUAL(p1b.size(), 2U);

    // Test projection exactly at next waypoint
    xt::xarray<double> waypoints2 = {
        {0.0, 0.0},
        {1.0, 0.01},  // Projection at next waypoint (projection ≈ segment_length)
        {1.0, 0.0}
    };

    // Point near next should coalesce if within tolerance
    // Projection ≈ 1.0, segment_length ≈ 1.0005, perpendicular distance ≈ 0.01
    path p2 = path::create(waypoints2, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p2.size(), 1U);

    // Test projection clearly beyond next waypoint
    xt::xarray<double> waypoints3 = {
        {0.0, 0.0},
        {1.5, 0.0},  // Clearly beyond next waypoint (projection > segment_length)
        {1.0, 0.0}
    };

    // Point beyond next should be kept
    path p3 = path::create(waypoints3, path::options{}.set_max_linear_deviation(0.1));
    BOOST_CHECK_EQUAL(p3.size(), 2U);
}

// Adversarial geometry tests

BOOST_AUTO_TEST_CASE(tube_coalescing_narrow_zigzag_at_boundary) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Zigzag pattern with deviations exactly at tolerance boundary
    // Tests numerical stability when deviation ≈ max_deviation
    const double tol = 0.1;
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, tol * 0.99},      // Just inside tolerance
        {2.0, -tol * 0.99},     // Just inside tolerance
        {3.0, tol * 1.01},      // Just outside tolerance
        {4.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(tol));

    // First two should coalesce, third is outside tolerance
    // Verify first/last waypoints preserved
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{4.0, 0.0}));

    verify_path_visits_waypoints(p, waypoints, tol);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_projection_exactly_at_segment_length) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Construct waypoint where projection exactly equals segment_length
    // This tests the boundary condition: projection <= segment_length vs projection < segment_length
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 1e-10},  // Essentially at (1,0) with tiny perpendicular offset
        {1.0, 0.0}     // Next is essentially same x-coordinate
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // Should coalesce since perpendicular distance is tiny
    BOOST_CHECK_EQUAL(p.size(), 1U);

    // Verify endpoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}, 1e-6));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{1.0, 0.0}, 1e-6));
}

BOOST_AUTO_TEST_CASE(tube_coalescing_long_chain_mixed_pattern) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Long chain with complex pattern: some coalesce, some don't
    // Tests that anchor updates correctly through many iterations
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.01},   // Within tolerance of (0,0)→(2,0)
        {2.0, 0.0},
        {3.0, 0.5},    // Large deviation, forces new segment
        {4.0, 0.51},   // Within tolerance of (3,0.5)→(5,0.5)
        {5.0, 0.5},
        {6.0, 0.0},    // Another large deviation
        {7.0, 0.01},   // Within tolerance of (6,0)→(8,0)
        {8.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // Expected segments:
    // (0,0)→(2,0): coalesces waypoint 1
    // (2,0)→(3,0.5): large turn
    // (3,0.5)→(5,0.5): coalesces waypoint 4
    // (5,0.5)→(6,0): large turn
    // (6,0)→(8,0): coalesces waypoint 7
    BOOST_CHECK_EQUAL(p.size(), 5U);

    // Verify first/last
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{8.0, 0.0}));

    // Verify all waypoints visited
    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_rapidly_changing_direction) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Rapid direction changes that should prevent coalescing
    // Tests that projection bounds check correctly identifies non-monotonic motion
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},   // Move right
        {0.5, 0.0},   // Reverse left (projection < 0 from waypoint 1)
        {1.5, 0.0},   // Jump ahead right (projection > segment_length from 0.5)
        {1.0, 0.0}    // Back to middle
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // Each direction change should be preserved (no coalescing due to reversals/jumps)
    BOOST_CHECK_EQUAL(p.size(), 4U);

    // Verify first/last
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{1.0, 0.0}));

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(tube_coalescing_many_near_duplicates) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Many points clustered tightly together (near-duplicates but not exact)
    // Tests handling of very short segments
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.0, 1e-11},  // Essentially duplicate (< 1e-10 threshold)
        {1e-11, 0.0},  // Essentially duplicate
        {1e-11, 1e-11},  // Essentially duplicate
        {1.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // All near-duplicates should be skipped via segment_length < 1e-10 check
    // Should end up with single segment from first to last
    BOOST_CHECK_EQUAL(p.size(), 1U);

    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}, 1e-9));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{1.0, 0.0}, 1e-9));
}

BOOST_AUTO_TEST_CASE(tube_coalescing_high_dimension_corner_cases) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // High-dimensional path with complex geometry
    // Tests that algorithm works correctly in higher dimensions
    xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.01, 0.0, 0.0, 0.0, 0.0},  // Small deviation in one dimension
        {2.0, 0.0, 0.01, 0.0, 0.0, 0.0},  // Small deviation in different dimension
        {3.0, 0.0, 0.0, 0.5, 0.0, 0.0},   // Large deviation
        {4.0, 0.0, 0.0, 0.5, 0.01, 0.0},  // Continue from new anchor
        {5.0, 0.0, 0.0, 0.5, 0.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1));

    // First two coalesce, large deviation at waypoint 3, last two coalesce
    // Verify endpoints in 6D space
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK_EQUAL(config_start.shape(0), 6U);
    BOOST_CHECK_EQUAL(config_end.shape(0), 6U);
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{5.0, 0.0, 0.0, 0.5, 0.0, 0.0}));

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

// Circular blend tests
// These tests verify circular arc blending at corners that cannot be coalesced

BOOST_AUTO_TEST_CASE(circular_blend_right_angle_basic) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Simple 90-degree corner with blend
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // Corner at 90 degrees
        {1.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should have 3 segments: trimmed linear, circular blend, trimmed linear
    BOOST_CHECK_EQUAL(p.size(), 3U);

    // Verify path starts and ends at exact waypoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{1.0, 1.0}));

    // Verify corner waypoint is visited within blend deviation
    verify_path_visits_waypoints(p, waypoints, 0.1);

    // Check that middle segment is circular
    auto it = p.begin();
    BOOST_CHECK((*it).is<path::segment::linear>());  // First segment
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());  // Blend segment
    ++it;
    BOOST_CHECK((*it).is<path::segment::linear>());  // Last segment
}

BOOST_AUTO_TEST_CASE(circular_blend_obtuse_angle) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // 135-degree turn (obtuse angle)
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // Corner at 135 degrees
        {0.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should create blend at obtuse corner
    BOOST_CHECK_EQUAL(p.size(), 3U);

    // Verify endpoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{0.0, 1.0}));

    // Verify corner is visited within tolerance
    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_acute_angle) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // 45-degree turn (acute angle)
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // Corner at 45 degrees
        {2.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should create blend at acute corner
    BOOST_CHECK_EQUAL(p.size(), 3U);

    // Verify endpoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{2.0, 1.0}));

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_multiple_corners) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Path with two 90-degree corners
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // First corner
        {1.0, 1.0},  // Second corner
        {0.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should have: linear, blend, linear, blend, linear = 5 segments
    BOOST_CHECK_EQUAL(p.size(), 5U);

    // Verify endpoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{0.0, 1.0}));

    // All corners should be visited within tolerance
    verify_path_visits_waypoints(p, waypoints, 0.1);

    // Verify blend segments are circular
    auto it = p.begin();
    BOOST_CHECK((*it).is<path::segment::linear>());  // Start segment
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());  // First blend
    ++it;
    BOOST_CHECK((*it).is<path::segment::linear>());  // Middle segment
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());  // Second blend
    ++it;
    BOOST_CHECK((*it).is<path::segment::linear>());  // End segment
}

BOOST_AUTO_TEST_CASE(circular_blend_zero_deviation_no_blends) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // With zero blend deviation, should get sharp corners (no blends)
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.0));

    // Should have 2 linear segments, no blends
    BOOST_CHECK_EQUAL(p.size(), 2U);

    // Both segments should be linear
    for (auto seg : p) {
        BOOST_CHECK(seg.is<path::segment::linear>());
    }

    // Path goes exactly through corner
    auto corner_config = p.configuration(arc_length{1.0});
    BOOST_CHECK(configs_close(corner_config, xt::xarray<double>{1.0, 0.0}));
}

BOOST_AUTO_TEST_CASE(circular_blend_preserves_coalescing) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Mix of coalesceable points and blend corners
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.5, 0.0},  // Should coalesce
        {1.0, 0.0},  // Corner requiring blend
        {1.0, 1.0}
    };

    path p = path::create(waypoints,
                         path::options{}
                             .set_max_linear_deviation(0.1)
                             .set_max_blend_deviation(0.1));

    // Should coalesce first segment, then blend at corner: linear, blend, linear = 3 segments
    BOOST_CHECK_EQUAL(p.size(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_insufficient_segment_length) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Corner with segments too short for requested blend
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.1, 0.0},  // Very short segment
        {0.1, 0.1}   // Very short segment
    };

    // Request large blend that won't fit fully
    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.5));

    // Should create a blend limited by segment length (ℓ = min(0.05, 0.05, ...))
    // Result: trimmed linear, circular blend, trimmed linear = 3 segments
    BOOST_CHECK_EQUAL(p.size(), 3U);

    // Verify endpoints
    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());
    BOOST_CHECK(configs_close(config_start, xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(config_end, xt::xarray<double>{0.1, 0.1}));
}

BOOST_AUTO_TEST_CASE(circular_blend_collinear_no_blend) {
    using namespace viam::trajex::totg;

    // Collinear points should not create blend (angle is 180 degrees)
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {2.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should have 2 linear segments, no blend (points are collinear)
    BOOST_CHECK_EQUAL(p.size(), 2U);

    for (auto seg : p) {
        BOOST_CHECK(seg.is<path::segment::linear>());
    }
}

BOOST_AUTO_TEST_CASE(circular_blend_very_sharp_angle) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Nearly 180-degree turn (very sharp bend back)
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {0.1, 0.1}  // Nearly reverses direction
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should create blend even for sharp angle
    BOOST_CHECK_GE(p.size(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_higher_dimensions) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // 3D path with corner
    xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},  // Corner in XY plane
        {1.0, 1.0, 0.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    BOOST_CHECK_EQUAL(p.size(), 3U);
    BOOST_CHECK_EQUAL(p.dof(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_blend_radius_geometry) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Verify blend geometry: 90-degree corner with known max_deviation
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0}
    };

    const double max_dev = 0.1;
    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(max_dev));

    // For 90-degree turn using Kunz & Stilman formulas (eq. 3-4):
    // Trim: ℓ = δ * sin(α/2) / (1 - cos(α/2))
    // Radius: r = ℓ / tan(α/2)
    const double half_angle = M_PI / 4.0;  // 45 degrees
    const double expected_trim = max_dev * std::sin(half_angle) / (1.0 - std::cos(half_angle));
    const double expected_radius = expected_trim / std::tan(half_angle);

    // Path length should be: (1 - trim) + (radius * π/2) + (1 - trim)
    const double expected_length = 2.0 * (1.0 - expected_trim) + expected_radius * M_PI / 2.0;
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), expected_length, 0.1);  // 0.1% tolerance
}

BOOST_AUTO_TEST_CASE(circular_blend_start_end_waypoints_exact) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Start and end waypoints must be exact even with blends
    xt::xarray<double> waypoints = {
        {0.5, 0.3},  // Arbitrary start
        {1.7, 0.8},  // Corner
        {2.1, 1.9}   // Arbitrary end
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.2));

    auto config_start = p.configuration(arc_length{0.0});
    auto config_end = p.configuration(p.length());

    // Start and end must be exact (machine precision)
    BOOST_CHECK_CLOSE(config_start(0), 0.5, 1e-10);
    BOOST_CHECK_CLOSE(config_start(1), 0.3, 1e-10);
    BOOST_CHECK_CLOSE(config_end(0), 2.1, 1e-10);
    BOOST_CHECK_CLOSE(config_end(1), 1.9, 1e-10);
}

BOOST_AUTO_TEST_CASE(circular_blend_symmetric_corners) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Symmetric path - blends should be symmetric
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0},
        {0.0, 1.0},
        {0.0, 0.0}  // Square path
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should have 4 corners with 4 blends: 4 linear + 4 circular = 8 segments
    // (last segment closes back to start, but no blend at start/end)
    BOOST_CHECK_GE(p.size(), 7U);  // At least 4 corners blended
}

BOOST_AUTO_TEST_CASE(circular_blend_different_blend_and_linear_tolerances) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Use different tolerances for coalescing and blending
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.5, 0.05},  // Slightly off line
        {1.0, 0.0},   // Corner
        {1.0, 1.0}
    };

    // Large linear deviation to coalesce, small blend deviation
    path p = path::create(waypoints,
                         path::options{}
                             .set_max_linear_deviation(0.1)   // Coalesce middle point
                             .set_max_blend_deviation(0.05)); // Small blend

    // Middle point should coalesce, corner should blend
    BOOST_CHECK_EQUAL(p.size(), 3U);
    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_no_coalescing_only_blending) {
    using namespace viam::trajex::totg;

    // Test blending without coalescing enabled
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0}
    };

    path p = path::create(waypoints,
                         path::options{}
                             .set_max_linear_deviation(0.0)  // No coalescing
                             .set_max_blend_deviation(0.1)); // Only blending

    // Should have blend at corner
    BOOST_CHECK_EQUAL(p.size(), 3U);

    // Verify second segment is circular
    auto it = p.begin();
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());
}

BOOST_AUTO_TEST_CASE(circular_blend_consecutive_corners) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Two corners very close together
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // First corner
        {1.1, 0.1},  // Second corner very close
        {1.1, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.05));

    // Should handle consecutive corners appropriately
    BOOST_CHECK_GE(p.size(), 3U);
    verify_path_visits_waypoints(p, waypoints, 0.05);
}

BOOST_AUTO_TEST_CASE(circular_blend_path_continuity) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Verify path is continuous across blend boundaries
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Sample at segment boundaries - should be continuous
    auto it = p.begin();
    auto seg1_end = (*it).end();
    ++it;
    auto seg2_start = (*it).start();
    auto seg2_end = (*it).end();
    ++it;
    auto seg3_start = (*it).start();

    // Boundaries should match
    BOOST_CHECK_CLOSE(static_cast<double>(seg1_end), static_cast<double>(seg2_start), 1e-10);
    BOOST_CHECK_CLOSE(static_cast<double>(seg2_end), static_cast<double>(seg3_start), 1e-10);

    // Configurations at boundaries should match
    auto config_boundary1 = p.configuration(seg1_end);
    auto config_boundary1_next = p.configuration(seg2_start);
    BOOST_CHECK(configs_close(config_boundary1, config_boundary1_next));
}

BOOST_AUTO_TEST_CASE(circular_blend_tangent_continuity) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Verify tangent vectors are continuous at blend boundaries
    xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0}
    };

    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Get tangents just before and just after segment boundaries
    auto it = p.begin();
    auto seg1_end = (*it).end();
    ++it;
    auto seg2_start = (*it).start();

    // Tangent at end of linear segment
    auto tangent1 = p.tangent(arc_length{static_cast<double>(seg1_end) - 1e-6});
    // Tangent at start of blend
    auto tangent2 = p.tangent(arc_length{static_cast<double>(seg2_start) + 1e-6});

    // Tangents should be close (C1 continuity)
    double dot_product = xt::sum(tangent1 * tangent2)();
    BOOST_CHECK_CLOSE(dot_product, 1.0, 1.0);  // Within 1% (small angle tolerance)
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajectory_generation_tests)

BOOST_AUTO_TEST_CASE(generate_trajectory) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(std::move(p), options)));
}

BOOST_AUTO_TEST_CASE(validates_velocity_dof) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0},  // Wrong DOF
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_acceleration_dof) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5}  // Wrong DOF
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(custom_integration_parameters) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
                                .delta = 0.0005,
                                .epsilon = 1e-9};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(std::move(p), options)));
}

BOOST_AUTO_TEST_CASE(validates_delta_positive) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
        .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
        .delta = 0.0  // Invalid
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_epsilon_positive) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
        .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
        .epsilon = -1e-6  // Invalid
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajectory_tests)

BOOST_AUTO_TEST_CASE(trajectory_has_path_reference) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Should be able to get path reference (trajectory now owns the path)
    BOOST_CHECK_EQUAL(traj.path().dof(), 3);
}

BOOST_AUTO_TEST_CASE(trajectory_dof_accessor) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0, 1.0},
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_EQUAL(traj.dof(), 4);
}

BOOST_AUTO_TEST_CASE(trajectory_duration_is_valid) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Duration should be non-negative (currently 0.0 until implementation)
    BOOST_CHECK(traj.duration().count() >= 0.0);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_negative_time) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_THROW(traj.sample(trajectory::seconds{-1.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_time_beyond_duration) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Beyond duration should throw
    const auto beyond_duration = traj.duration() + trajectory::seconds{1.0};
    BOOST_CHECK_THROW(traj.sample(beyond_duration), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_returns_valid_structure) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // At time 0.0 should return a valid sample structure
    auto sample = traj.sample(trajectory::seconds{0.0});
    BOOST_CHECK_EQUAL(sample.time.count(), 0.0);
    // Position, velocity, acceleration arrays exist (placeholder values for now)
    BOOST_CHECK_NO_THROW(sample.configuration.size());
    BOOST_CHECK_NO_THROW(sample.velocity.size());
    BOOST_CHECK_NO_THROW(sample.acceleration.size());
}

BOOST_AUTO_TEST_CASE(trajectory_dof_matches_path_dof) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const size_t expected_dof = p.dof();
    trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_EQUAL(traj.dof(), expected_dof);
    BOOST_CHECK_EQUAL(traj.path().dof(), traj.dof());
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(segment_tests)

BOOST_AUTO_TEST_CASE(linear_constructor) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Should be able to create linear segments
    xt::xarray<double> start = {1.0, 2.0, 3.0};
    xt::xarray<double> end = {4.0, 5.0, 6.0};
    BOOST_CHECK_NO_THROW((path::segment::linear{start, end}));
}

BOOST_AUTO_TEST_CASE(circular_constructor) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Should be able to create circular segments
    xt::xarray<double> center = {0.0, 0.0, 0.0};
    xt::xarray<double> x = {1.0, 0.0, 0.0};
    xt::xarray<double> y = {0.0, 1.0, 0.0};
    BOOST_CHECK_NO_THROW((path::segment::circular{center, x, y, 1.0, 1.57}));
}

BOOST_AUTO_TEST_CASE(segment_view_type_check) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create linear segment
    path::segment::linear linear_data{xt::xarray<double>{1.0, 2.0, 3.0}, xt::xarray<double>{4.0, 5.0, 6.0}};
    path::segment linear_seg{linear_data};
    path::segment::view linear_view{linear_seg, arc_length{0.0}, linear_data.length};

    BOOST_CHECK(linear_view.is<path::segment::linear>());
    BOOST_CHECK(!linear_view.is<path::segment::circular>());

    // Create circular segment
    path::segment::circular circular_data{
        xt::xarray<double>{0.0, 0.0, 0.0}, xt::xarray<double>{1.0, 0.0, 0.0}, xt::xarray<double>{0.0, 1.0, 0.0}, 1.0, 1.57};
    path::segment circular_seg{circular_data};
    path::segment::view circular_view{circular_seg, arc_length{0.0}, arc_length{1.57}};

    BOOST_CHECK(circular_view.is<path::segment::circular>());
    BOOST_CHECK(!circular_view.is<path::segment::linear>());
}

BOOST_AUTO_TEST_CASE(segment_view_visit) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create linear segment
    path::segment::linear linear_data{xt::xarray<double>{1.0, 2.0, 3.0}, xt::xarray<double>{4.0, 5.0, 6.0}};
    path::segment linear_seg{linear_data};
    path::segment::view linear_view{linear_seg, arc_length{0.0}, linear_data.length};

    // Visit linear segment - direction (3,3,3) normalizes to (1/√3, 1/√3, 1/√3)
    bool visited_linear = false;
    linear_view.visit([&visited_linear](const auto& seg_data) {
        using T = std::decay_t<decltype(seg_data)>;
        if constexpr (std::is_same_v<T, path::segment::linear>) {
            visited_linear = true;
            BOOST_CHECK_CLOSE(seg_data.unit_direction(0), 1.0 / std::sqrt(3.0), 1e-6);
            BOOST_CHECK_CLOSE(seg_data.unit_direction(1), 1.0 / std::sqrt(3.0), 1e-6);
            BOOST_CHECK_CLOSE(seg_data.unit_direction(2), 1.0 / std::sqrt(3.0), 1e-6);
        }
    });
    BOOST_CHECK(visited_linear);

    // Create circular segment
    path::segment::circular circular_data{
        xt::xarray<double>{0.0, 0.0, 0.0}, xt::xarray<double>{1.0, 0.0, 0.0}, xt::xarray<double>{0.0, 1.0, 0.0}, 2.5, 1.57};
    path::segment circular_seg{circular_data};
    path::segment::view circular_view{circular_seg, arc_length{0.0}, arc_length{1.57}};

    // Visit circular segment
    bool visited_circular = false;
    circular_view.visit([&visited_circular](const auto& seg_data) {
        using T = std::decay_t<decltype(seg_data)>;
        if constexpr (std::is_same_v<T, path::segment::circular>) {
            visited_circular = true;
            BOOST_CHECK_CLOSE(seg_data.radius, 2.5, 1e-6);
        }
    });
    BOOST_CHECK(visited_circular);
}

BOOST_AUTO_TEST_CASE(linear_segment_configuration) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segment from (0,0) to (3,4) - length = 5, direction = (0.6, 0.8)
    path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, data.length};

    // At start (s=0): should be (0,0)
    auto config_start = view.configuration(arc_length{0.0});
    BOOST_CHECK_CLOSE(config_start(0), 0.0, 1e-6);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 1e-6);

    // At middle (s=2.5): should be (1.5, 2.0)
    auto config_mid = view.configuration(arc_length{2.5});
    BOOST_CHECK_CLOSE(config_mid(0), 1.5, 1e-6);
    BOOST_CHECK_CLOSE(config_mid(1), 2.0, 1e-6);

    // At end (s=5): should be (3,4)
    auto config_end = view.configuration(arc_length{5.0});
    BOOST_CHECK_CLOSE(config_end(0), 3.0, 1e-6);
    BOOST_CHECK_CLOSE(config_end(1), 4.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_segment_tangent) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segment from (0,0) to (3,4) - length = 5
    // Unit tangent should be (0.6, 0.8)
    path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, data.length};

    // Tangent should be constant along the segment
    auto tangent_start = view.tangent(arc_length{0.0});
    BOOST_CHECK_CLOSE(tangent_start(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_start(1), 0.8, 1e-6);

    auto tangent_mid = view.tangent(arc_length{2.5});
    BOOST_CHECK_CLOSE(tangent_mid(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_mid(1), 0.8, 1e-6);

    auto tangent_end = view.tangent(arc_length{5.0});
    BOOST_CHECK_CLOSE(tangent_end(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_end(1), 0.8, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_segment_curvature) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segments have zero curvature
    path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, data.length};

    auto curvature = view.curvature(arc_length{2.5});
    BOOST_CHECK_SMALL(curvature(0), 1e-10);
    BOOST_CHECK_SMALL(curvature(1), 1e-10);
}

BOOST_AUTO_TEST_CASE(circular_segment_configuration) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Quarter circle in xy plane, radius 1, centered at origin
    // x = (1, 0), y = (0, 1), arc from (1,0) to (0,1)
    path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        1.0,                           // radius
        std::numbers::pi / 2.0         // angle_rads (90 degrees)
    };
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, arc_length{std::numbers::pi / 2.0}};

    // At start (angle=0): should be (1, 0)
    auto config_start = view.configuration(arc_length{0.0});
    BOOST_CHECK_CLOSE(config_start(0), 1.0, 1e-6);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 1e-6);

    // At middle (angle=π/4): should be (√2/2, √2/2)
    auto config_mid = view.configuration(arc_length{std::numbers::pi / 4.0});
    BOOST_CHECK_CLOSE(config_mid(0), std::sqrt(2.0) / 2.0, 1e-6);
    BOOST_CHECK_CLOSE(config_mid(1), std::sqrt(2.0) / 2.0, 1e-6);

    // At end (angle=π/2): should be (0, 1)
    auto config_end = view.configuration(arc_length{std::numbers::pi / 2.0});
    BOOST_CHECK_SMALL(config_end(0), 1e-10);  // Near zero, use SMALL not CLOSE
    BOOST_CHECK_CLOSE(config_end(1), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(circular_segment_tangent) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Quarter circle, tangent should be perpendicular to radius
    path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        1.0,                           // radius
        std::numbers::pi / 2.0         // angle_rads
    };
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, arc_length{std::numbers::pi / 2.0}};

    // At start (angle=0): tangent should be (0, 1)
    auto tangent_start = view.tangent(arc_length{0.0});
    BOOST_CHECK_SMALL(tangent_start(0), 1e-10);  // Near zero
    BOOST_CHECK_CLOSE(tangent_start(1), 1.0, 1e-6);

    // At end (angle=π/2): tangent should be (-1, 0)
    auto tangent_end = view.tangent(arc_length{std::numbers::pi / 2.0});
    BOOST_CHECK_CLOSE(tangent_end(0), -1.0, 1e-6);
    BOOST_CHECK_SMALL(tangent_end(1), 1e-10);  // Near zero
}

BOOST_AUTO_TEST_CASE(circular_segment_curvature) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Curvature vector points toward center with magnitude 1/radius
    path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        2.0,                           // radius = 2
        std::numbers::pi / 2.0         // angle_rads
    };
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, arc_length{std::numbers::pi}};

    // At start (angle=0): curvature should point from (2,0) to (0,0), magnitude 1/2
    auto curvature_start = view.curvature(arc_length{0.0});
    BOOST_CHECK_CLOSE(curvature_start(0), -0.5, 1e-6);  // -1/radius
    BOOST_CHECK_CLOSE(curvature_start(1), 0.0, 1e-6);

    // Magnitude should be 1/radius = 0.5
    double mag = xt::norm_l2(curvature_start)();
    BOOST_CHECK_CLOSE(mag, 0.5, 1e-6);
}

BOOST_AUTO_TEST_CASE(segment_view_bounds_checking) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    path::segment seg{data};
    path::segment::view view{seg, arc_length{0.0}, data.length};

    // Negative arc_length should throw
    BOOST_CHECK_THROW(view.configuration(arc_length{-1.0}), std::out_of_range);
    BOOST_CHECK_THROW(view.tangent(arc_length{-0.5}), std::out_of_range);
    BOOST_CHECK_THROW(view.curvature(arc_length{-0.1}), std::out_of_range);

    // Arc_length beyond segment end should throw
    BOOST_CHECK_THROW(view.configuration(arc_length{5.1}), std::out_of_range);
    BOOST_CHECK_THROW(view.tangent(arc_length{6.0}), std::out_of_range);
    BOOST_CHECK_THROW(view.curvature(arc_length{10.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(path_container_semantics) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create a path with 3 waypoints -> 2 linear segments
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    auto p = path::create(waypoints);

    // Test size and empty
    BOOST_CHECK_EQUAL(p.size(), 2U);
    BOOST_CHECK(!p.empty());

    // Test iteration - iterator yields segment::view
    size_t count = 0;
    for (const auto& view : p) {
        // Each iteration yields a segment::view
        BOOST_CHECK(view.is<path::segment::linear>());
        BOOST_CHECK(view.start() >= arc_length{0.0});
        BOOST_CHECK(view.length() > arc_length{0.0});
        ++count;
    }
    BOOST_CHECK_EQUAL(count, 2U);

    // Test iterator accessors
    auto it = p.begin();
    BOOST_CHECK(it != p.end());

    auto view1 = *it;
    BOOST_CHECK(view1.is<path::segment::linear>());
    BOOST_CHECK_EQUAL(static_cast<double>(view1.start()), 0.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view1.end()), 1.0);

    // Can query the view immediately
    auto config = view1.configuration(arc_length{0.5});
    BOOST_CHECK_EQUAL(config.shape(0), 2U);

    ++it;
    BOOST_CHECK(it != p.end());

    auto view2 = *it;
    BOOST_CHECK_EQUAL(static_cast<double>(view2.start()), 1.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view2.end()), 2.0);  // second segment length is 1.0

    ++it;
    BOOST_CHECK(it == p.end());

    // Test const_iterator
    auto cit = p.cbegin();
    BOOST_CHECK(cit != p.cend());
    ++cit;
    BOOST_CHECK(cit != p.cend());
    ++cit;
    BOOST_CHECK(cit == p.cend());
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(uniform_sampler_tests)

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_basic) {
    using namespace viam::trajex::totg;

    // 1.0 second @ 100 Hz should give 101 samples
    // dt = 1.0 / (101 - 1) = 1.0 / 100 = 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(1.0, 100.0);
    BOOST_CHECK_CLOSE(dt, 0.01, 0.001);  // Within 0.001% tolerance
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_ensures_endpoint_hit) {
    using namespace viam::trajex::totg;

    // 1.01 seconds @ 100 Hz
    // putative_samples = 1.01 * 100 = 101.0
    // num_samples = ceil(101.0) + 1 = 102
    // dt = 1.01 / (102 - 1) = 1.01 / 101 ≈ 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(1.01, 100.0);

    // Verify we can reach exactly 1.01 with 101 steps
    const double endpoint = 101 * dt;
    BOOST_CHECK_CLOSE(endpoint, 1.01, 0.001);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_oversamples) {
    using namespace viam::trajex::totg;

    // 0.99 seconds @ 100 Hz
    // putative_samples = 0.99 * 100 = 99.0
    // num_samples = ceil(99.0) + 1 = 100
    // dt = 0.99 / (100 - 1) = 0.99 / 99 = 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(0.99, 100.0);

    // Should slightly oversample (100 samples instead of 99)
    const int num_samples = static_cast<int>(std::ceil(0.99 / dt)) + 1;
    BOOST_CHECK_EQUAL(num_samples, 100);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_invalid_duration) {
    using namespace viam::trajex::totg;

    // Zero duration
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(0.0, 100.0), std::invalid_argument);

    // Negative duration
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(-1.0, 100.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_invalid_frequency) {
    using namespace viam::trajex::totg;

    // Zero frequency
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(1.0, 0.0), std::invalid_argument);

    // Negative frequency
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(1.0, -100.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_exceeds_max_samples) {
    using namespace viam::trajex::totg;

    // Duration * frequency > 1000000
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(10000.0, 1000.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_at_least_two_samples) {
    using namespace viam::trajex::totg;

    // Very small duration and frequency
    const double dt = uniform_sampler::calculate_quantized_dt(0.001, 1.0);

    // Should still get at least 2 samples (start and end)
    // putative = 0.001 * 1 = 0.001
    // num_samples = ceil(0.001) + 1 = 2
    // dt = 0.001 / (2 - 1) = 0.001
    BOOST_CHECK_CLOSE(dt, 0.001, 0.001);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(end_to_end_tests)

BOOST_AUTO_TEST_CASE(waypoints_to_samples_smoke_test) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // 1. Create waypoints
    xt::xarray<double> waypoints = {
        {0.0, 0.0},  // Start
        {1.0, 0.0},  // Move right
        {1.0, 1.0}   // Move up
    };

    // 2. Create path
    path p = path::create(waypoints);  // Linear segments only
    BOOST_CHECK_EQUAL(p.size(), 2U);
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 2.0, 0.001);

    // 3. Create trajectory
    trajectory::options opts{.max_velocity = xt::xarray<double>{1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5}};
    trajectory traj = trajectory::create(std::move(p), opts);

    // 4. Verify trajectory has non-zero duration (stub sets fake duration)
    BOOST_CHECK(traj.duration().count() > 0.0);

    // 5. Create uniform sampler
    uniform_sampler sampler{trajectory::seconds{0.1}};

    // 6. Sample trajectory using cursor + sampler
    int sample_count = 0;
    trajectory::seconds last_time{-1.0};

    for (const auto& s : traj.samples(sampler)) {
        // Verify sample structure
        BOOST_CHECK_NO_THROW(s.time.count());
        BOOST_CHECK_NO_THROW(s.configuration.size());
        BOOST_CHECK_NO_THROW(s.velocity.size());
        BOOST_CHECK_NO_THROW(s.acceleration.size());

        // Verify time is monotonically increasing
        BOOST_CHECK(s.time > last_time);
        last_time = s.time;

        // Verify configuration has correct DOF
        BOOST_CHECK_EQUAL(s.configuration.shape(0), 2U);
        BOOST_CHECK_EQUAL(s.velocity.shape(0), 2U);
        BOOST_CHECK_EQUAL(s.acceleration.shape(0), 2U);

        ++sample_count;
    }

    // Should have gotten multiple samples
    BOOST_CHECK(sample_count > 1);
}

BOOST_AUTO_TEST_CASE(cursor_manual_sampling) {
    using namespace viam::trajex::totg;

    // Create simple trajectory
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    trajectory::options opts{.max_velocity = xt::xarray<double>{1.0, 1.0}, .max_acceleration = xt::xarray<double>{1.0, 1.0}};
    trajectory traj = trajectory::create(std::move(p), opts);

    // Create cursor
    auto cursor = traj.create_cursor();

    // Verify initial state
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.0);
    BOOST_CHECK_EQUAL(&cursor.trajectory(), &traj);

    // Advance and sample
    cursor.advance_to(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.5);

    auto sample = cursor.sample();
    BOOST_CHECK_EQUAL(sample.time.count(), 0.5);
    BOOST_CHECK_EQUAL(sample.configuration.shape(0), 2U);
}

BOOST_AUTO_TEST_CASE(quantized_sampler_end_to_end) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Create trajectory with path length that works out to power-of-2 fraction
    // Path length = 1.0, max_vel = 1.0 → duration = 1.0s
    // At 8 Hz: putative = 8, num_samples = 9, dt = 1.0/8 = 0.125 (exact in binary!)
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 1.0;
    path p = path::create(waypoints);

    trajectory::options opts{.max_velocity = xt::xarray<double>{1.0}, .max_acceleration = xt::xarray<double>{1.0}};
    trajectory traj = trajectory::create(std::move(p), opts);

    BOOST_REQUIRE_EQUAL(traj.duration().count(), 1.0);  // Verify stub gives expected duration

    // Use quantized sampler with 8 Hz (dt = 0.125, exact in binary)
    auto sampler = uniform_sampler::quantized_for_trajectory(traj, hertz{8.0});

    std::optional<trajectory::seconds> last_time;
    int sample_count = 0;
    for (const auto& s : traj.samples(sampler)) {
        last_time = s.time;
        ++sample_count;
    }

    // Should get exactly 9 samples: 0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0
    BOOST_REQUIRE(last_time.has_value());
    BOOST_CHECK_EQUAL(sample_count, 9);
    BOOST_CHECK_EQUAL(last_time->count(), 1.0);  // Should hit endpoint exactly
}

BOOST_AUTO_TEST_SUITE_END()

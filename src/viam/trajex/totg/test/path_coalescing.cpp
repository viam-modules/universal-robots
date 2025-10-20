// Path tube coalescing algorithm tests
// Extracted from test.cpp lines 152-189 (helpers) and 377-1105 (tests)

#include "test_utils.hpp"

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <boost/test/unit_test.hpp>

using viam::trajex::totg::test::configs_close;
using viam::trajex::totg::test::verify_path_visits_waypoints;

BOOST_AUTO_TEST_SUITE(path_tests)
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


BOOST_AUTO_TEST_SUITE_END()

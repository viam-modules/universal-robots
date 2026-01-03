// Path circular blend tests
// Extracted from test.cpp lines 152-189 (helpers) and 1106-1560 (tests)

#include "test_utils.hpp"

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/types/angles.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <numbers>

#include <boost/test/unit_test.hpp>

using viam::trajex::degrees_to_radians;
using viam::trajex::totg::test::configs_close;
using viam::trajex::totg::test::verify_path_visits_waypoints;

BOOST_AUTO_TEST_SUITE(path_tests)
// Circular blend tests
// These tests verify circular arc blending at corners that cannot be coalesced

BOOST_AUTO_TEST_CASE(circular_blend_right_angle_basic) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Simple 90-degree corner with blend
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {1.0, 0.0},  // Corner at 90 degrees
                                          {1.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {1.0, 0.0},  // Corner at 135 degrees
                                          {0.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {1.0, 0.0},  // Corner at 45 degrees
                                          {2.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {1.0, 0.0},  // First corner
                                          {1.0, 1.0},  // Second corner
                                          {0.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.0));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {0.5, 0.0},  // Should coalesce
                                          {1.0, 0.0},  // Corner requiring blend
                                          {1.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_linear_deviation(0.1).set_max_blend_deviation(0.1));

    // Should coalesce first segment, then blend at corner: linear, blend, linear = 3 segments
    BOOST_CHECK_EQUAL(p.size(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_insufficient_segment_length) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Corner with segments too short for requested blend
    const xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {0.1, 0.0},  // Very short segment
        {0.1, 0.1}   // Very short segment
    };

    // Request large blend that won't fit fully
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.5));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {
        {0.0, 0.0}, {1.0, 0.0}, {0.1, 0.1}  // Nearly reverses direction
    };

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should create blend even for sharp angle
    BOOST_CHECK_GE(p.size(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_higher_dimensions) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // 3D path with corner
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0},
                                          {1.0, 0.0, 0.0},  // Corner in XY plane
                                          {1.0, 1.0, 0.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    BOOST_CHECK_EQUAL(p.size(), 3U);
    BOOST_CHECK_EQUAL(p.dof(), 3U);

    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_blend_radius_geometry) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Verify blend geometry: 90-degree corner with known max_deviation
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

    const double max_dev = 0.1;
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(max_dev));

    // For 90-degree turn using Kunz & Stilman formulas (eq. 3-4):
    // Trim: ℓ = δ * sin(α/2) / (1 - cos(α/2))
    // Radius: r = ℓ / tan(α/2)
    const double half_angle = degrees_to_radians(45.0);
    const double expected_trim = max_dev * std::sin(half_angle) / (1.0 - std::cos(half_angle));
    const double expected_radius = expected_trim / std::tan(half_angle);

    // Path length should be: (1 - trim) + (radius * π/2) + (1 - trim)
    const double expected_length = (2.0 * (1.0 - expected_trim)) + ((expected_radius * std::numbers::pi) / 2.0);
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), expected_length, 0.1);  // 0.1% tolerance
}

BOOST_AUTO_TEST_CASE(circular_blend_start_end_waypoints_exact) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Start and end waypoints must be exact even with blends
    const xt::xarray<double> waypoints = {
        {0.5, 0.3},  // Arbitrary start
        {1.7, 0.8},  // Corner
        {2.1, 1.9}   // Arbitrary end
    };

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.2));

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
    const xt::xarray<double> waypoints = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}  // Square path
    };

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Should have 4 corners with 4 blends: 4 linear + 4 circular = 8 segments
    // (last segment closes back to start, but no blend at start/end)
    BOOST_CHECK_GE(p.size(), 7U);  // At least 4 corners blended
}

BOOST_AUTO_TEST_CASE(circular_blend_different_blend_and_linear_tolerances) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Use different tolerances for coalescing and blending
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {0.5, 0.05},  // Slightly off line
                                          {1.0, 0.0},   // Corner
                                          {1.0, 1.0}};

    // Large linear deviation to coalesce, small blend deviation
    const path p = path::create(waypoints,
                                path::options{}
                                    .set_max_linear_deviation(0.1)    // Coalesce middle point
                                    .set_max_blend_deviation(0.05));  // Small blend

    // Middle point should coalesce, corner should blend
    BOOST_CHECK_EQUAL(p.size(), 3U);
    verify_path_visits_waypoints(p, waypoints, 0.1);
}

BOOST_AUTO_TEST_CASE(circular_blend_no_coalescing_only_blending) {
    using namespace viam::trajex::totg;

    // Test blending without coalescing enabled
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

    const path p = path::create(waypoints,
                                path::options{}
                                    .set_max_linear_deviation(0.0)   // No coalescing
                                    .set_max_blend_deviation(0.1));  // Only blending

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
    const xt::xarray<double> waypoints = {{0.0, 0.0},
                                          {1.0, 0.0},  // First corner
                                          {1.1, 0.1},  // Second corner very close
                                          {1.1, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.05));

    // Should handle consecutive corners appropriately
    BOOST_CHECK_GE(p.size(), 3U);
    verify_path_visits_waypoints(p, waypoints, 0.05);
}

BOOST_AUTO_TEST_CASE(circular_blend_path_continuity) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Verify path is continuous across blend boundaries
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

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
    const double dot_product = xt::sum(tangent1 * tangent2)();
    BOOST_CHECK_CLOSE(dot_product, 1.0, 1.0);  // Within 1% (small angle tolerance)
}

BOOST_AUTO_TEST_SUITE_END()

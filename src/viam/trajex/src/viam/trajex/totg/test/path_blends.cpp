// Path circular blend tests
// Extracted from test.cpp lines 152-189 (helpers) and 1106-1560 (tests)

#include "test_utils.hpp"

#include <cmath>
#include <numbers>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/types/angles.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <boost/test/unit_test.hpp>

using viam::trajex::degrees_to_radians;
using viam::trajex::totg::test::configs_close;
using viam::trajex::totg::test::path_type_sequence;
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

// Regression: blend trim must cap against the original (waypoint-to-waypoint) segment length,
// not the remaining length after a previous blend consumed its share. Kunz & Stilman Section IV:
// "We also need to ensure that the circular segment does not replace more than half of each of
// the neighboring linear segments." Neighboring means the original path segments.
//
// Z-staircase: (0,0) -> (0,1) -> (1,1) -> (1,2). Three unit segments, two 90-degree corners.
// With geometry-limited deviation (100.0 >> needed), each blend trims min(L1/2, L2/2) = 0.5 from
// the shared middle segment. Together they consume it entirely, so no linear segment appears
// between the two circular blends -- the path is LCCL.
BOOST_AUTO_TEST_CASE(circular_blend_trim_respects_original_segment_length) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    const xt::xarray<double> waypoints = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 2.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(100.0));

    // The two blends together consume the full middle segment: LCCL, no linear between them.
    BOOST_REQUIRE_EQUAL(p.size(), 4U);

    auto it = p.begin();
    BOOST_CHECK((*it).is<path::segment::linear>());
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());
    ++it;
    BOOST_CHECK((*it).is<path::segment::circular>());
    ++it;
    BOOST_CHECK((*it).is<path::segment::linear>());

    BOOST_CHECK(configs_close(p.configuration(arc_length{0.0}), xt::xarray<double>{0.0, 0.0}));
    BOOST_CHECK(configs_close(p.configuration(p.length()), xt::xarray<double>{1.0, 2.0}));
}

// Regression (RSDK-12771): blend trim can leave an incoming linear whose arc-length
// contribution is representable relative to dist_to_locus (passing the existing guard)
// but not relative to cumulative_length, producing a zero-width segment view that traps
// path::cursor in an infinite loop during forward integration.
//
// Waypoints extracted from a real 7000-waypoint dense trajectory capture that triggered
// the hang. The blend at waypoint [20] trims the [19]->[20] incoming segment down to
// ~6.9e-18 radians, which is below ULP of the cumulative arc length (~0.118 rad) at
// that point.
BOOST_AUTO_TEST_CASE(blend_trim_zero_width_segment) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {
        {0.82739107946303503, 0.88282048800603485, 0.3039375847975771, -3.136198473590786, 0.98778065081918609, 0.74039620121758376},
        {0.82629088242947146, 0.88015395319311995, 0.29929061711574401, -3.1361612990406651, 0.98576635111271682, 0.74146624876782663},
        {0.82518902514739223, 0.87750093630902959, 0.29467042227130935, -3.1361212806604346, 0.98376585935290217, 0.7425369173880072},
        {0.82408439141808032, 0.87486007035523905, 0.29007430703974835, -3.136083340709396, 0.98177575096874348, 0.74361269537785513},
        {0.82297766704596664, 0.87223282218850162, 0.28550514470579086, -3.1360432784270205, 0.97980134663434992, 0.7446863591792573},
        {0.82175093188676929, 0.86934079066277903, 0.28047842559826797, -3.1359989040577294, 0.97762778247494508, 0.74587804588963502},
        {0.82052122478846157, 0.86646396202641407, 0.27548181447209474, -3.1359544227250566, 0.97546946137410784, 0.74707256825793555},
        {0.81928851589217333, 0.86360202999044611, 0.27051468583278532, -3.1359098886631349, 0.97332587508569313, 0.74827007078641894},
        {0.8180527967645177, 0.86075441708409506, 0.26557607257769117, -3.135865526464737, 0.97119590782824416, 0.74947043923950929},
        {0.81691547078297266, 0.85815210566197009, 0.26106578102812716, -3.1358240998866274, 0.96925181642027836, 0.75057491150423716},
        {0.81577544695593207, 0.85556188147923184, 0.25657946439116519, -3.1357834811994563, 0.9673198495138623, 0.75168229175030732},
        {0.81463322511546443, 0.85298334430198652, 0.25211648865248959, -3.135741171041222, 0.96539921603621814, 0.75279123497030176},
        {0.8134882050117741, 0.8504162266389127, 0.24767611227211728, -3.1356999702528503, 0.96348915314636985, 0.75390332968205465},
        {0.81239977300000277, 0.8479915334672864, 0.24348418047826675, -3.1356649955963296, 0.96168408895129498, 0.75496095169515587},
        {0.81130993554969921, 0.84557935592151812, 0.23931712067998823, -3.135625154121938, 0.95989451529514969, 0.75601903359253686},
        {0.81021853068193705, 0.84317907848638818, 0.23517379397324567, -3.135581307674193, 0.95811895749286435, 0.75707826471125217},
        {0.80912400325497646, 0.8407869536014575, 0.23104650713052508, -3.1355416437658326, 0.95634888710197274, 0.75814095295035044},
        {0.8079804871517845, 0.8383033321852208, 0.2267640376073147, -3.135500013460045, 0.95451298609643331, 0.75925112318759469},
        {0.80683444819414352, 0.83583023284298352, 0.22250259711775061, -3.1354584901952416, 0.95268732223556085, 0.76036387690474327},
        {0.80568580309839632, 0.83336896720205433, 0.2182642674204768, -3.1354171697807547, 0.95087545998514833, 0.76147925931929172},
        {0.80453379071711051, 0.8309134890629436, 0.21403804528617396, -3.1353803509698968, 0.94906219693260463, 0.76259848675258046},
        {0.80348035729964573, 0.82868511774530973, 0.21020648152308605, -3.1353351970286405, 0.94743226524784308, 0.76362086319744726},
        {0.80242316594544671, 0.82645762028543435, 0.20637754406775707, -3.1352985688288966, 0.94579144498181067, 0.76464635430345096},
        {0.80136455609434587, 0.82424371145004272, 0.20257472793898343, -3.1352581155276646, 0.94417228036033596, 0.76567419014892324},
        {0.80030343340426746, 0.82203557017046813, 0.19878386991668273, -3.1352196593643331, 0.94255472609856017, 0.7667040771997794},
        {0.79914710824348445, 0.8196442013430919, 0.19468049525035699, -3.1351802801979831, 0.94080381320088813, 0.76782674729827727},
        {0.79798873121757508, 0.81726319384717827, 0.19059777012593726, -3.1351382211247683, 0.93906420782762101, 0.7689510613676801},
    };

    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    for (const auto& seg : p) {
        BOOST_CHECK(seg.end() > seg.start());
    }
}

BOOST_AUTO_TEST_SUITE_END()

// Tests for path construction at degenerate and near-degenerate waypoint geometries.
// These verify the path-layer decisions (segment types, dropping waypoints) that are
// prerequisites for correct TOTG behavior at cusps and near-cusps. All tests express
// desired end-state behavior; most will fail until the implementation is complete.

BOOST_AUTO_TEST_SUITE(extremal_path_construction_tests)

// Exact collinear: middle point lies exactly on the chord. The blend radius is a
// singularity (infinite), so no blend is emitted. Two linear segments remain.
BOOST_AUTO_TEST_CASE(exact_collinear_no_blend) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LL");
}

// Near-collinear: the middle waypoint deviates from the chord by 0.001, well within
// max_blend_deviation=0.1. The natural blend radius is enormous; min_curvature caps it
// at 1/min_curvature, producing a small-radius arc that retains C1 continuity.
// Structure is LCL; the arc radius is capped, not the path topology.
BOOST_AUTO_TEST_CASE(near_collinear_emits_capped_arc) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.001}, {2.0, 0.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LCL");
}

// Same near-collinear geometry, but with a tight min_blend_curvature=1.0 (max radius=1.0).
// The natural blend radius is ~50000; the cap clamps it to <= 1.0.
BOOST_AUTO_TEST_CASE(near_collinear_arc_radius_respects_min_curvature) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.001}, {2.0, 0.0}};
    constexpr double min_curvature = 1.0;
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1).set_min_blend_curvature(min_curvature));
    for (const auto& seg : p) {
        seg.visit([](const auto& s) {
            if constexpr (std::is_same_v<std::decay_t<decltype(s)>, path::segment::circular>) {
                BOOST_CHECK_LE(s.radius, 1.0 / min_curvature);
            }
        });
    }
}

// Near-collinear outside tolerance: cross-track deviation (0.5) exceeds max_blend_deviation
// (0.1), so the waypoint represents a genuine direction change and a blend is emitted.
BOOST_AUTO_TEST_CASE(near_collinear_outside_deviation_produces_blend) {
    using namespace viam::trajex::totg;
    // Cross-track deviation of (1, 0.5) from chord (0,0)-(2,0) is 0.5 > max_blend_deviation=0.1.
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.5}, {2.0, 0.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LCL");
}

// Exact reversal: path goes A -> B -> A. No blend arc is geometrically valid at a true
// reversal; the path must be L-L.
BOOST_AUTO_TEST_CASE(exact_reversal_emits_ll) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LL");
}

// Near-reversal within the curvature band: a 170-degree turn with max_blend_curvature=100
// produces a blend with curvature ~105 (radius ~0.0096), which exceeds the limit.
// The near-reversal is too tight to blend usefully; emit L-L.
BOOST_AUTO_TEST_CASE(near_reversal_within_curvature_band_emits_ll) {
    using namespace viam::trajex::totg;
    // Outgoing direction at 170 degrees from incoming (1,0): corner at (1,0), third waypoint
    // at (1+cos(170deg), sin(170deg)).
    const double angle_rad = degrees_to_radians(170.0);
    const xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0 + std::cos(angle_rad), std::sin(angle_rad)},
    };
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1).set_max_blend_curvature(100.0));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LL");
}

// Non-reversal corner with max_blend_curvature set: a 90-degree turn has curvature ~4,
// well below max_blend_curvature=100, so a blend is emitted normally.
BOOST_AUTO_TEST_CASE(non_reversal_corner_outside_curvature_band_emits_lcl) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1).set_max_blend_curvature(100.0));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LCL");
}

// Z-staircase: three equal-length segments with two 90-degree corners. With a large
// deviation budget, each blend consumes its full half-segment. Together the two blends
// consume the middle segment entirely, producing L-C-C-L (4 segments).
BOOST_AUTO_TEST_CASE(z_staircase_adjacent_blends_emit_lccl) {
    using namespace viam::trajex::totg;
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 2.0}};
    const path p = path::create(waypoints, path::options{}.set_max_blend_deviation(100.0));
    BOOST_CHECK_EQUAL(path_type_sequence(p), "LCCL");
}

BOOST_AUTO_TEST_SUITE_END()  // extremal_path_construction_tests

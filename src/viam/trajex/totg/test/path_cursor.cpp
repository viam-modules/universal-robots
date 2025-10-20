// Path cursor tests: sequential traversal and seeking
// Extracted from test.cpp lines 2259-2798

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(path_cursor_tests)

BOOST_AUTO_TEST_CASE(construct_and_get_path_reference) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Should have reference to path
    BOOST_CHECK_EQUAL(&cursor.path(), &p);
    BOOST_CHECK_EQUAL(cursor.path().length(), p.length());
}

BOOST_AUTO_TEST_CASE(construct_with_minimal_path) {
    using namespace viam::trajex::totg;

    // Create minimal valid path (2 distinct waypoints)
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    // Should be able to construct cursor with minimal path
    BOOST_CHECK_NO_THROW({ auto c = p.create_cursor(); (void)c; });
}

BOOST_AUTO_TEST_CASE(initial_position_at_start) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    BOOST_CHECK_EQUAL(cursor.position(), arc_length{0.0});
    BOOST_CHECK(cursor.position() != p.length());
    BOOST_CHECK_EQUAL(static_cast<double>(cursor.position()), 0.0);
}

BOOST_AUTO_TEST_CASE(forward_integration_small_steps) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Take several small forward steps
    cursor.seek_by(arc_length{0.1});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.1, 0.01);
    BOOST_CHECK(cursor.position() != arc_length{0.0});
    BOOST_CHECK(cursor.position() != p.length());

    cursor.seek_by(arc_length{0.3});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.4, 0.01);

    cursor.seek_by(arc_length{0.6});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 1.0, 0.01);
    BOOST_CHECK_EQUAL(cursor.position(), p.length());
}

BOOST_AUTO_TEST_CASE(forward_integration_to_end) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Advance past end (should overflow to sentinel)
    cursor.seek_by(p.length() + arc_length{10.0});

    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(backward_integration_small_steps) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Start at end
    cursor.seek(p.length());
    BOOST_CHECK_EQUAL(cursor.position(), p.length());

    // Take several small backward steps (negative delta)
    cursor.seek_by(arc_length{-0.2});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.8, 0.01);
    BOOST_CHECK(cursor.position() != arc_length{0.0});
    BOOST_CHECK(cursor.position() != p.length());

    cursor.seek_by(arc_length{-0.3});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.5, 0.01);

    cursor.seek_by(arc_length{-0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.0, 0.01);
    BOOST_CHECK_EQUAL(cursor.position(), arc_length{0.0});
}

BOOST_AUTO_TEST_CASE(backward_integration_to_start) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    cursor.seek(p.length());

    // Advance past start (should underflow to sentinel)
    cursor.seek_by(arc_length{-100.0});

    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(bidirectional_integration) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Forward then backward
    cursor.seek_by(arc_length{0.6});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.6, 0.01);

    cursor.seek_by(arc_length{-0.2});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.4, 0.01);

    cursor.seek_by(arc_length{0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.9, 0.01);

    // This goes slightly past 0, so should underflow to sentinel
    cursor.seek_by(arc_length{-0.9});
    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(reset_to_start_and_end) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    cursor.seek_by(arc_length{0.5});

    cursor.seek(arc_length{0.0});
    BOOST_CHECK_EQUAL(cursor.position(), arc_length{0.0});
    BOOST_CHECK_EQUAL(static_cast<double>(cursor.position()), 0.0);

    cursor.seek(p.length());
    BOOST_CHECK_EQUAL(cursor.position(), p.length());
    BOOST_CHECK_EQUAL(static_cast<double>(cursor.position()), static_cast<double>(p.length()));
}

BOOST_AUTO_TEST_CASE(reset_to_specific_position) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    cursor.seek(arc_length{0.3});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.3, 0.01);

    // Underflows to sentinel at start, overflows to sentinel at end
    cursor.seek(arc_length{-5.0});
    BOOST_CHECK(cursor == cursor.end());

    cursor.seek(arc_length{0.5});  // Reset to valid position
    BOOST_CHECK(cursor != cursor.end());

    cursor.seek(p.length() + arc_length{10.0});
    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(configuration_query_at_positions) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // At start
    auto config_start = cursor.configuration();
    BOOST_CHECK_CLOSE(config_start(0), 0.0, 0.01);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 0.01);

    // At midpoint
    cursor.seek(p.length() / 2.0);
    auto config_mid = cursor.configuration();
    BOOST_CHECK_CLOSE(config_mid(0), 0.5, 0.01);
    BOOST_CHECK_CLOSE(config_mid(1), 0.5, 0.01);

    // At end
    cursor.seek(p.length());
    auto config_end = cursor.configuration();
    BOOST_CHECK_CLOSE(config_end(0), 1.0, 0.01);
    BOOST_CHECK_CLOSE(config_end(1), 1.0, 0.01);
}

BOOST_AUTO_TEST_CASE(tangent_query) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Tangent should be constant along linear segment
    auto tangent_start = cursor.tangent();
    cursor.seek_by(arc_length{0.5});
    auto tangent_mid = cursor.tangent();

    BOOST_CHECK_CLOSE(tangent_start(0), 1.0, 0.01);  // Unit vector in x direction
    BOOST_CHECK_CLOSE(tangent_start(1), 0.0, 0.01);
    BOOST_CHECK_CLOSE(tangent_mid(0), 1.0, 0.01);
    BOOST_CHECK_CLOSE(tangent_mid(1), 0.0, 0.01);
}

BOOST_AUTO_TEST_CASE(curvature_query) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Curvature should be zero for linear segment
    auto curvature = cursor.curvature();
    BOOST_CHECK_SMALL(curvature(0), 1e-6);
    BOOST_CHECK_SMALL(curvature(1), 1e-6);
}

BOOST_AUTO_TEST_CASE(integration_across_multiple_segments) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create path with circular blend
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    path::options opts;
    opts.set_max_blend_deviation(0.1);
    path p = path::create(waypoints, opts);

    // Path should have 3 segments: linear, circular, linear
    BOOST_CHECK_EQUAL(p.size(), 3U);

    path::cursor cursor = p.create_cursor();

    // Integrate forward across all segments
    const double step = 0.05;
    while (cursor != cursor.end()) {
        // Should be able to query at any position
        auto config = cursor.configuration();
        auto tangent = cursor.tangent();
        auto curvature = cursor.curvature();

        BOOST_CHECK_EQUAL(config.shape(0), 2U);
        BOOST_CHECK_EQUAL(tangent.shape(0), 2U);
        BOOST_CHECK_EQUAL(curvature.shape(0), 2U);

        cursor.seek_by(arc_length{step});
    }
}

BOOST_AUTO_TEST_CASE(backward_integration_across_multiple_segments) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create path with circular blend
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    path::options opts;
    opts.set_max_blend_deviation(0.1);
    path p = path::create(waypoints, opts);

    path::cursor cursor = p.create_cursor();
    cursor.seek(p.length());

    // Integrate backward across all segments
    const double step = -0.05;  // Negative for backward
    while (cursor != cursor.end()) {
        // Should be able to query at any position
        auto config = cursor.configuration();
        BOOST_CHECK_EQUAL(config.shape(0), 2U);

        cursor.seek_by(arc_length{step});
    }
}

BOOST_AUTO_TEST_CASE(hint_optimization_forward_sequential) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create path with many segments
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};
    path p = path::create(waypoints);

    path::cursor cursor = p.create_cursor();

    // Sequential forward access should benefit from hints
    // We can't measure performance directly, but verify correctness
    const double step = 0.1;
    while (cursor != cursor.end()) {
        // Verify queries work correctly (hint must be valid)
        auto config = cursor.configuration();
        BOOST_CHECK_EQUAL(config.shape(0), 2U);

        cursor.seek_by(arc_length{step});
    }
}

BOOST_AUTO_TEST_CASE(advance_by_and_sentinel_detection) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);
    path::cursor cursor = p.create_cursor();

    // Normal advancement - should advance full amount
    cursor.seek_by(arc_length{0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.5, 0.01);
    BOOST_CHECK(cursor != cursor.end());

    // Advance to exactly the end - should be at length, not sentinel
    cursor.seek_by(arc_length{0.5});
    BOOST_CHECK_EQUAL(cursor.position(), p.length());
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 1.0, 0.01);
    BOOST_CHECK(cursor != cursor.end());

    // Try to advance past end - should overflow to sentinel
    cursor.seek_by(arc_length{0.1});
    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(advance_by_negative_and_start_clamping) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);
    path::cursor cursor = p.create_cursor(p.length());  // Start at end

    // Normal backward advancement - should advance full amount (negative)
    cursor.seek_by(arc_length{-0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.5, 0.01);
    BOOST_CHECK(cursor != cursor.end());

    // Try to advance past start - should underflow to sentinel
    cursor.seek_by(arc_length{-1.0});
    BOOST_CHECK(cursor == cursor.end());

    // Reset to start
    cursor.seek(arc_length{0.0});
    BOOST_CHECK_EQUAL(cursor.position(), arc_length{0.0});
    BOOST_CHECK(cursor != cursor.end());

    // Try to advance when at start - should underflow to sentinel
    cursor.seek_by(arc_length{-0.1});
    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(seek_to_position_and_clamping) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);
    path::cursor cursor = p.create_cursor();

    // Seek to valid position
    cursor.seek(arc_length{0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), 0.5, 0.01);
    BOOST_CHECK(cursor != cursor.end());

    // Try to seek past end - should overflow to sentinel
    cursor.seek(arc_length{10.0});
    BOOST_CHECK(cursor == cursor.end());

    // Seek before start - should underflow to sentinel
    cursor.seek(arc_length{-5.0});
    BOOST_CHECK(cursor == cursor.end());
}

BOOST_AUTO_TEST_CASE(create_cursor_at_various_positions) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    // Create at start (default)
    path::cursor cursor1 = p.create_cursor();
    BOOST_CHECK_EQUAL(cursor1.position(), arc_length{0.0});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor1.position()), 0.0, 0.01);

    // Create at middle
    path::cursor cursor2 = p.create_cursor(arc_length{0.5});
    BOOST_CHECK_CLOSE(static_cast<double>(cursor2.position()), 0.5, 0.01);
    BOOST_CHECK(cursor2.position() != arc_length{0.0});
    BOOST_CHECK(cursor2.position() != p.length());

    // Create at end
    path::cursor cursor3 = p.create_cursor(p.length());
    BOOST_CHECK_EQUAL(cursor3.position(), p.length());
    BOOST_CHECK_CLOSE(static_cast<double>(cursor3.position()), 1.0, 0.01);

    // Create past end - should clamp
    path::cursor cursor4 = p.create_cursor(arc_length{10.0});
    BOOST_CHECK_EQUAL(cursor4.position(), p.length());

    // Create before start - should clamp
    path::cursor cursor5 = p.create_cursor(arc_length{-5.0});
    BOOST_CHECK_EQUAL(cursor5.position(), arc_length{0.0});
}

BOOST_AUTO_TEST_CASE(large_jumps_across_many_segments) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create path with many segments (12 segments: 6 linear + 6 circular blends)
    xt::xarray<double> waypoints = {
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
        {3.0, 1.0}, {3.0, 2.0}, {2.0, 2.0}, {1.0, 2.0}
    };
    path::options opts;
    opts.set_max_blend_deviation(0.1);
    path p = path::create(waypoints, opts);

    // Should have multiple segments (linear + circular blends)
    BOOST_CHECK_GT(p.size(), 5U);  // At least 6 segments to test large jumps

    path::cursor cursor = p.create_cursor();

    // Perform large non-sequential jumps and verify correctness
    // These jumps exercise the binary search fallback in update_hint()

    // Jump to near end
    arc_length pos1 = p.length() * 0.9;
    cursor.seek(pos1);
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), static_cast<double>(pos1), 0.01);
    auto config1 = cursor.configuration();
    BOOST_CHECK_EQUAL(config1.shape(0), 2U);

    // Jump back to near beginning
    arc_length pos2 = p.length() * 0.1;
    cursor.seek(pos2);
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), static_cast<double>(pos2), 0.01);
    auto config2 = cursor.configuration();
    BOOST_CHECK_EQUAL(config2.shape(0), 2U);

    // Jump to middle
    arc_length pos3 = p.length() * 0.5;
    cursor.seek(pos3);
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), static_cast<double>(pos3), 0.01);
    auto config3 = cursor.configuration();
    BOOST_CHECK_EQUAL(config3.shape(0), 2U);

    // Jump to near start
    arc_length pos4 = p.length() * 0.05;
    cursor.seek(pos4);
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), static_cast<double>(pos4), 0.01);
    auto config4 = cursor.configuration();
    BOOST_CHECK_EQUAL(config4.shape(0), 2U);

    // Jump to near end again
    arc_length pos5 = p.length() * 0.95;
    cursor.seek(pos5);
    BOOST_CHECK_CLOSE(static_cast<double>(cursor.position()), static_cast<double>(pos5), 0.01);
    auto config5 = cursor.configuration();
    BOOST_CHECK_EQUAL(config5.shape(0), 2U);

    // Verify tangent and curvature queries also work after jumps
    auto tangent = cursor.tangent();
    BOOST_CHECK_EQUAL(tangent.shape(0), 2U);
    auto curvature = cursor.curvature();
    BOOST_CHECK_EQUAL(curvature.shape(0), 2U);
}

BOOST_AUTO_TEST_CASE(dereference_operator_returns_segment_view) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create path with circular blend to have different segment types
    xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    path::options opts;
    opts.set_max_blend_deviation(0.1);
    path p = path::create(waypoints, opts);

    // Should have 3 segments: linear, circular, linear
    BOOST_CHECK_EQUAL(p.size(), 3U);

    path::cursor cursor = p.create_cursor();

    // Start on first segment (linear)
    auto view1 = *cursor;
    BOOST_CHECK(view1.is<path::segment::linear>());
    BOOST_CHECK_EQUAL(view1.start(), arc_length{0.0});

    // Seek to middle - should be on circular blend
    cursor.seek(p.length() * 0.5);
    auto view2 = *cursor;
    BOOST_CHECK(view2.is<path::segment::circular>());

    // Seek to end - should be on last linear segment
    cursor.seek(p.length() * 0.95);
    auto view3 = *cursor;
    BOOST_CHECK(view3.is<path::segment::linear>());

    // Verify view gives access to segment bounds
    BOOST_CHECK(view3.start() < view3.end());
    BOOST_CHECK_EQUAL(view3.length(), view3.end() - view3.start());

    // Can use dereference result to query geometry
    auto config = view3.configuration(cursor.position());
    BOOST_CHECK_EQUAL(config.shape(0), 2U);
}


BOOST_AUTO_TEST_SUITE_END()

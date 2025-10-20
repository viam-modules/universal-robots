// Basic path tests: creation, validation, queries, segment lookup
// Extracted from test.cpp lines 150-376

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(path_tests)

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

BOOST_AUTO_TEST_SUITE_END()

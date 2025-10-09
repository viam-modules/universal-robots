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

BOOST_AUTO_TEST_CASE(create_path) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    BOOST_CHECK_NO_THROW(static_cast<void>(path::create(acc, 0.0)));
}

BOOST_AUTO_TEST_CASE(validates_max_deviation) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    BOOST_CHECK_THROW(static_cast<void>(path::create(acc, -0.1)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(requires_minimum_two_waypoints) {
    using namespace viam::trajex::totg;

    // Single waypoint should throw
    xt::xarray<double> single = {{1.0, 2.0, 3.0}};
    BOOST_CHECK_THROW(static_cast<void>(path::create(single, 0.0)), std::invalid_argument);

    // Two waypoints should work
    xt::xarray<double> two = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    BOOST_CHECK_NO_THROW(static_cast<void>(path::create(two, 0.0)));
}

BOOST_AUTO_TEST_CASE(create_from_array_vs_accumulator) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};

    // Create from accumulator
    waypoint_accumulator acc{waypoints};
    path p1 = path::create(acc, 0.0);

    // Create from array directly
    path p2 = path::create(waypoints, 0.0);

    // Should produce equivalent paths
    BOOST_CHECK_EQUAL(p1.dof(), p2.dof());
    BOOST_CHECK_EQUAL(p1.size(), p2.size());
    BOOST_CHECK_EQUAL(static_cast<double>(p1.length()), static_cast<double>(p2.length()));
}

BOOST_AUTO_TEST_CASE(path_dof_matches_waypoints) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}};
    path p = path::create(waypoints, 0.0);

    BOOST_CHECK_EQUAL(p.dof(), 4);
}

BOOST_AUTO_TEST_CASE(linear_path_length) {
    using namespace viam::trajex::totg;

    // Simple 3-4-5 triangle in 2D
    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints, 0.0);

    // Two segments: 3.0 + 4.0 = 7.0
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 7.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_path_segment_count) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    path p = path::create(waypoints, 0.0);

    // 3 waypoints => 2 segments
    BOOST_CHECK_EQUAL(p.size(), 2);
}

BOOST_AUTO_TEST_CASE(segment_lookup_at_start) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    xt::xarray<double> waypoints = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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

BOOST_AUTO_TEST_CASE(circular_blends_throws) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};

    // Non-zero max_deviation should throw (not implemented yet)
    BOOST_CHECK_THROW(static_cast<void>(path::create(waypoints, 0.1)), std::runtime_error);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajectory_generation_tests)

BOOST_AUTO_TEST_CASE(generate_trajectory) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(std::move(p), options)));
}

BOOST_AUTO_TEST_CASE(validates_velocity_dof) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0},  // Wrong DOF
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_acceleration_dof) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5}  // Wrong DOF
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(std::move(p), options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(custom_integration_parameters) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
                                .delta = 0.0005,
                                .epsilon = 1e-9};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(std::move(p), options)));
}

BOOST_AUTO_TEST_CASE(validates_delta_positive) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Should be able to get path reference (trajectory now owns the path)
    BOOST_CHECK_EQUAL(traj.path().dof(), 3);
}

BOOST_AUTO_TEST_CASE(trajectory_dof_accessor) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0, 1.0},
                                .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_EQUAL(traj.dof(), 4);
}

BOOST_AUTO_TEST_CASE(trajectory_duration_is_valid) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Duration should be non-negative (currently 0.0 until implementation)
    BOOST_CHECK(traj.duration().count() >= 0.0);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_negative_time) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_THROW(traj.sample(trajectory::seconds{-1.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_time_beyond_duration) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

    trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    trajectory traj = trajectory::create(std::move(p), options);

    // Beyond duration should throw
    const auto beyond_duration = traj.duration() + trajectory::seconds{1.0};
    BOOST_CHECK_THROW(traj.sample(beyond_duration), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_returns_valid_structure) {
    using namespace viam::trajex::totg;

    xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);  // Linear segments only
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
    path p = path::create(waypoints, 0.0);

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
    path p = path::create(waypoints, 0.0);

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

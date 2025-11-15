// Trajectory tests: generation and queries
// Extracted from test.cpp lines 1563-1753

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/types/arc_length.hpp>

#include <boost/test/unit_test.hpp>

#include "test_utils.hpp"

namespace {

using namespace viam::trajex::totg;

trajectory create_trajectory_with_integration_points(path p, std::vector<trajectory::integration_point> points) {
    const trajectory::options opts{.max_velocity = xt::ones<double>({p.dof()}), .max_acceleration = xt::ones<double>({p.dof()})};
    return trajectory::create(std::move(p), opts, std::move(points));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(trajectory_generation_tests)

BOOST_AUTO_TEST_CASE(generate_trajectory) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(p, options)));
}

BOOST_AUTO_TEST_CASE(validates_velocity_dof) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0},  // Wrong DOF
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(p, options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_acceleration_dof) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0}, .max_acceleration = xt::xarray<double>{0.5, 0.5}  // Wrong DOF
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(p, options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(custom_integration_parameters) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
                                      .delta = trajectory::seconds{0.0005},
                                      .epsilon = 1e-9};

    BOOST_CHECK_NO_THROW(static_cast<void>(trajectory::create(p, options)));
}

BOOST_AUTO_TEST_CASE(validates_delta_positive) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
        .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
        .delta = trajectory::seconds{0.0}  // Invalid
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(p, options)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_epsilon_positive) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const path p = path::create(waypoints);

    const trajectory::options options{
        .max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
        .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5},
        .epsilon = -1e-6  // Invalid
    };

    BOOST_CHECK_THROW(static_cast<void>(trajectory::create(p, options)), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajectory_tests)

BOOST_AUTO_TEST_CASE(trajectory_has_path_reference) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

    // Should be able to get path reference (trajectory now owns the path)
    BOOST_CHECK_EQUAL(traj.path().dof(), 3);
}

BOOST_AUTO_TEST_CASE(trajectory_dof_accessor) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_EQUAL(traj.dof(), 4);
}

BOOST_AUTO_TEST_CASE(trajectory_duration_is_valid) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

    // Duration should be non-negative (currently 0.0 until implementation)
    BOOST_CHECK(traj.duration().count() >= 0.0);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_negative_time) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_THROW(traj.sample(trajectory::seconds{-1.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_throws_on_time_beyond_duration) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

    // Beyond duration should throw
    const auto beyond_duration = traj.duration() + trajectory::seconds{1.0};
    BOOST_CHECK_THROW(traj.sample(beyond_duration), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_at_returns_valid_structure) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const trajectory traj = trajectory::create(std::move(p), options);

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

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    path p = path::create(waypoints);

    const trajectory::options options{.max_velocity = xt::xarray<double>{1.0, 1.0, 1.0},
                                      .max_acceleration = xt::xarray<double>{0.5, 0.5, 0.5}};

    const size_t expected_dof = p.dof();
    const trajectory traj = trajectory::create(std::move(p), options);

    BOOST_CHECK_EQUAL(traj.dof(), expected_dof);
    BOOST_CHECK_EQUAL(traj.path().dof(), traj.dof());
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajectory_sampling_tests)

using namespace viam::trajex::totg;
using namespace viam::trajex::totg::test;
using namespace viam::trajex;

// Boundary condition tests

BOOST_AUTO_TEST_CASE(sample_at_start) {
    // Create simple linear path: (0,0) -> (1,0)
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    // Create trajectory with known integration points
    // Simple constant velocity: s_dot = 1.0, duration = 1.0s
    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at t=0
    auto sample = traj.sample(trajectory::seconds{0.0});

    BOOST_CHECK_EQUAL(sample.time.count(), 0.0);
    BOOST_CHECK_EQUAL(sample.configuration.shape(0), 2U);
    BOOST_CHECK_CLOSE(sample.configuration(0), 0.0, 0.001);
    BOOST_CHECK_CLOSE(sample.configuration(1), 0.0, 0.001);
    BOOST_CHECK_CLOSE(sample.velocity(0), 1.0, 0.001);  // Constant s_dot = 1.0
    BOOST_CHECK_CLOSE(sample.velocity(1), 0.0, 0.001);  // No y motion
}

BOOST_AUTO_TEST_CASE(sample_at_end) {
    // Create simple linear path: (0,0) -> (1,0)
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    // Create trajectory with known integration points
    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at t=duration
    auto sample = traj.sample(trajectory::seconds{1.0});

    BOOST_CHECK_EQUAL(sample.time.count(), 1.0);
    BOOST_CHECK_CLOSE(sample.configuration(0), 1.0, 0.001);  // At end of path
    BOOST_CHECK_CLOSE(sample.configuration(1), 0.0, 0.001);
    BOOST_CHECK_CLOSE(sample.velocity(0), 1.0, 0.001);  // Still moving at end
}

BOOST_AUTO_TEST_CASE(sample_before_start_throws) {
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample before start should throw
    BOOST_CHECK_THROW(traj.sample(trajectory::seconds{-0.1}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(sample_after_end_throws) {
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample after end should throw
    BOOST_CHECK_THROW(traj.sample(trajectory::seconds{1.1}), std::out_of_range);
}

// Chain rule verification tests

BOOST_AUTO_TEST_CASE(constant_velocity_linear_path) {
    // Linear path: (0,0) -> (2,0) with constant velocity
    // Expected: q_dot = q' * s_dot = (1,0) * 1.0 = (1,0)
    //           q_ddot = q' * s_ddot + q'' * s_dot^2 = (1,0) * 0.0 + (0,0) * 1.0 = (0,0)
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {2.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at midpoint
    auto sample = traj.sample(trajectory::seconds{1.0});

    // Position should be at s=1.0 -> (1,0)
    BOOST_CHECK_CLOSE(sample.configuration(0), 1.0, 0.001);
    BOOST_CHECK_CLOSE(sample.configuration(1), 0.0, 0.001);

    // Velocity: q' * s_dot = (1,0) * 1.0 = (1,0)
    BOOST_CHECK_CLOSE(sample.velocity(0), 1.0, 0.001);
    BOOST_CHECK_CLOSE(sample.velocity(1), 0.0, 0.001);

    // Acceleration: q' * s_ddot + q'' * s_dot^2 = (1,0) * 0 + (0,0) * 1 = (0,0)
    BOOST_CHECK_CLOSE(sample.acceleration(0), 0.0, 0.001);
    BOOST_CHECK_CLOSE(sample.acceleration(1), 0.0, 0.001);
}

BOOST_AUTO_TEST_CASE(constant_acceleration_linear_path) {
    // Linear path: (0,0) -> (2,0) with constant acceleration
    // Start at rest, accelerate at s_ddot = 2.0
    // At t=0: s=0, s_dot=0, s_ddot=2
    // At t=sqrt(2): s=2 (path end), s_dot=2*sqrt(2) (using s = 0.5*a*t^2)
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {2.0, 0.0}};
    path p = path::create(waypoints);
    const double t_end = std::sqrt(2.0);  // Time to reach s=2 with a=2

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 0.0, .s_ddot = 2.0},
        {.time = trajectory::seconds{t_end}, .s = arc_length{2.0}, .s_dot = 2.0 * t_end, .s_ddot = 2.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at t=1.0
    auto sample = traj.sample(trajectory::seconds{1.0});

    // Position at s=1.0 -> (1,0)
    BOOST_CHECK_CLOSE(sample.configuration(0), 1.0, 0.001);
    BOOST_CHECK_CLOSE(sample.configuration(1), 0.0, 0.001);

    // Velocity: q' * s_dot = (1,0) * 2.0 = (2,0)
    BOOST_CHECK_CLOSE(sample.velocity(0), 2.0, 0.001);
    BOOST_CHECK_CLOSE(sample.velocity(1), 0.0, 0.001);

    // Acceleration: q' * s_ddot + q'' * s_dot^2 = (1,0) * 2.0 + (0,0) * 4.0 = (2,0)
    BOOST_CHECK_CLOSE(sample.acceleration(0), 2.0, 0.001);
    BOOST_CHECK_CLOSE(sample.acceleration(1), 0.0, 0.001);
}

BOOST_AUTO_TEST_CASE(parabolic_interpolation_midpoint) {
    // Test parabolic interpolation between two points
    // Linear path for simplicity, focus on time interpolation
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {4.0, 0.0}};
    path p = path::create(waypoints);

    // Two points: accelerate from rest to reach path end at s=4
    // t=0: s=0, s_dot=0
    // t=2: s=4, s_dot=4
    // Using parabolic: s = s0 + s_dot0*dt + 0.5*s_ddot*dt^2
    // 4 = 0 + 0*2 + 0.5*s_ddot*4 -> s_ddot = 2.0
    // At t=1 (midpoint): s = 0 + 0 + 0.5*2*1 = 1.0, s_dot = 0 + 2*1 = 2.0
    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 0.0, .s_ddot = 2.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{4.0}, .s_dot = 4.0, .s_ddot = 2.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at midpoint t=1.0
    auto sample = traj.sample(trajectory::seconds{1.0});

    // Position at s=1.0 -> (1.0, 0)
    BOOST_CHECK_CLOSE(sample.configuration(0), 1.0, 0.001);

    // Velocity: q' * s_dot = (1,0) * 2.0 = (2,0)
    BOOST_CHECK_CLOSE(sample.velocity(0), 2.0, 0.001);

    // Acceleration: q' * s_ddot = (1,0) * 2.0 = (2,0)
    BOOST_CHECK_CLOSE(sample.acceleration(0), 2.0, 0.001);
}

BOOST_AUTO_TEST_CASE(cursor_maintains_position_across_samples) {
    // Verify cursor doesn't lose position when sampling multiple times
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();
    cursor.seek(trajectory::seconds{0.5});

    // Sample multiple times at same position
    auto s1 = cursor.sample();
    auto s2 = cursor.sample();
    auto s3 = cursor.sample();

    // All should be identical
    BOOST_CHECK_EQUAL(s1.time.count(), 0.5);
    BOOST_CHECK_EQUAL(s2.time.count(), 0.5);
    BOOST_CHECK_EQUAL(s3.time.count(), 0.5);

    BOOST_CHECK_CLOSE(s1.configuration(0), s2.configuration(0), 0.001);
    BOOST_CHECK_CLOSE(s2.configuration(0), s3.configuration(0), 0.001);
}

BOOST_AUTO_TEST_CASE(cursor_seek_updates_position) {
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Start at t=0
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.0);
    auto s0 = cursor.sample();
    BOOST_CHECK_CLOSE(s0.configuration(0), 0.0, 0.001);

    // Seek to t=0.5
    cursor.seek(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.5);
    auto s1 = cursor.sample();
    BOOST_CHECK_CLOSE(s1.configuration(0), 0.5, 0.001);

    // Seek to t=1.0
    cursor.seek(trajectory::seconds{1.0});
    BOOST_CHECK_EQUAL(cursor.time().count(), 1.0);
    auto s2 = cursor.sample();
    BOOST_CHECK_CLOSE(s2.configuration(0), 1.0, 0.001);
}

BOOST_AUTO_TEST_CASE(cursor_seek_by_advances_position) {
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {2.0, 0.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = 1.0, .s_ddot = 0.0}};

    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.0);

    // Advance by 0.5s increments
    cursor.seek_by(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.5);
    auto s1 = cursor.sample();
    BOOST_CHECK_CLOSE(s1.configuration(0), 0.5, 0.001);

    cursor.seek_by(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 1.0);
    auto s2 = cursor.sample();
    BOOST_CHECK_CLOSE(s2.configuration(0), 1.0, 0.001);

    cursor.seek_by(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 1.5);
    auto s3 = cursor.sample();
    BOOST_CHECK_CLOSE(s3.configuration(0), 1.5, 0.001);
}

// Hint optimization correctness tests

BOOST_AUTO_TEST_CASE(cursor_seek_within_current_integration_point) {
    // Verify fast path in seek() (lines 258-267 in trajectory.cpp)
    // When seeking within current integration point, hint should remain valid
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 5.0;
    path p = path::create(waypoints);

    // Create 6 integration points at t=0, 1, 2, 3, 4, 5 (must end at path.length() = 5.0)
    std::vector<trajectory::integration_point> points;
    for (int i = 0; i <= 5; ++i) {
        points.push_back(
            {.time = trajectory::seconds{static_cast<double>(i)}, .s = arc_length{static_cast<double>(i)}, .s_dot = 1.0, .s_ddot = 0.0});
    }
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Seek to t=0.5 (within first integration point interval [0, 1])
    cursor.seek(trajectory::seconds{0.5});
    auto s1 = cursor.sample();
    BOOST_CHECK_CLOSE(s1.configuration(0), 0.5, 0.001);

    // Seek to t=0.7 (still within same interval, should use fast path)
    cursor.seek(trajectory::seconds{0.7});
    auto s2 = cursor.sample();
    BOOST_CHECK_CLOSE(s2.configuration(0), 0.7, 0.001);

    // Seek to t=0.3 (backward within same interval, should use fast path)
    cursor.seek(trajectory::seconds{0.3});
    auto s3 = cursor.sample();
    BOOST_CHECK_CLOSE(s3.configuration(0), 0.3, 0.001);
}

BOOST_AUTO_TEST_CASE(cursor_seek_to_adjacent_integration_points) {
    // Verify forward-by-one path (lines 269-282) and backward-by-one path (lines 284-293)
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 5.0;
    path p = path::create(waypoints);

    // Create 6 integration points at t=0, 1, 2, 3, 4, 5 (must end at path.length() = 5.0)
    std::vector<trajectory::integration_point> points;
    for (int i = 0; i <= 5; ++i) {
        points.push_back(
            {.time = trajectory::seconds{static_cast<double>(i)}, .s = arc_length{static_cast<double>(i)}, .s_dot = 1.0, .s_ddot = 0.0});
    }
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Establish hint at point 0 by seeking to t=0.5
    cursor.seek(trajectory::seconds{0.5});
    auto s1 = cursor.sample();
    BOOST_CHECK_CLOSE(s1.configuration(0), 0.5, 0.001);

    // Seek forward to next integration point interval [1, 2]
    cursor.seek(trajectory::seconds{1.5});
    auto s2 = cursor.sample();
    BOOST_CHECK_CLOSE(s2.configuration(0), 1.5, 0.001);

    // Seek backward to previous integration point interval [0, 1]
    cursor.seek(trajectory::seconds{0.8});
    auto s3 = cursor.sample();
    BOOST_CHECK_CLOSE(s3.configuration(0), 0.8, 0.001);
}

BOOST_AUTO_TEST_CASE(cursor_seek_large_jump) {
    // Verify binary search path (lines 295-309) for large time jumps
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 6.0;
    path p = path::create(waypoints);

    // Create 7 integration points at t=0, 1, 2, 3, 4, 5, 6 (must end at path.length() = 6.0)
    std::vector<trajectory::integration_point> points;
    for (int i = 0; i <= 6; ++i) {
        points.push_back(
            {.time = trajectory::seconds{static_cast<double>(i)}, .s = arc_length{static_cast<double>(i)}, .s_dot = 1.0, .s_ddot = 0.0});
    }
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Establish hint at point 0 by seeking to t=0.5
    cursor.seek(trajectory::seconds{0.5});
    auto s1 = cursor.sample();
    BOOST_CHECK_CLOSE(s1.configuration(0), 0.5, 0.001);

    // Large jump forward to t=4.5 (should trigger binary search)
    cursor.seek(trajectory::seconds{4.5});
    auto s2 = cursor.sample();
    BOOST_CHECK_CLOSE(s2.configuration(0), 4.5, 0.001);

    // Large jump backward to t=1.5 (should trigger binary search)
    cursor.seek(trajectory::seconds{1.5});
    auto s3 = cursor.sample();
    BOOST_CHECK_CLOSE(s3.configuration(0), 1.5, 0.001);
}

// Circular blend trajectory test

BOOST_AUTO_TEST_CASE(trajectory_sampling_on_circular_blend) {
    // Create path with sharp 90-degree corner to generate circular blend
    // Path: (0,0) -> (1,0) -> (1,1)
    const xt::xarray<double> waypoints = {
        {0.0, 0.0},
        {1.0, 0.0},  // Right
        {1.0, 1.0}   // Up (90-degree turn)
    };
    path p = path::create(waypoints, path::options{}.set_max_blend_deviation(0.1));

    // Path should have: linear1, circular_blend, linear2
    BOOST_REQUIRE(p.size() >= 2);  // At least one blend

    // Find the circular segment by checking path segments
    arc_length blend_start = arc_length{0.0};
    arc_length blend_end = arc_length{0.0};
    bool found_circular = false;

    auto it = p.begin();
    arc_length current_s{0.0};
    while (it != p.end()) {
        const auto& seg_view = *it;
        if (seg_view.is<path::segment::circular>()) {
            blend_start = current_s;
            blend_end = current_s + seg_view.length();
            found_circular = true;
            break;
        }
        current_s += seg_view.length();
        ++it;
    }

    BOOST_REQUIRE(found_circular);  // Path must have a circular blend

    // Create trajectory with integration points
    // Sample on the circular blend at constant velocity
    const double blend_mid_s = static_cast<double>((blend_start + blend_end) / 2.0);
    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{blend_mid_s}, .s = arc_length{blend_mid_s}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{static_cast<double>(p.length())}, .s = p.length(), .s_dot = 1.0, .s_ddot = 0.0}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Sample at blend_mid_s (on circular segment)
    auto sample = traj.sample(trajectory::seconds{blend_mid_s});

    // Verify:
    // 1. Configuration is on the circular arc
    BOOST_CHECK_EQUAL(sample.configuration.shape(0), 2);

    // 2. Velocity should be non-zero (constant velocity along path)
    const double vel_magnitude = std::sqrt((sample.velocity(0) * sample.velocity(0)) + (sample.velocity(1) * sample.velocity(1)));
    BOOST_CHECK_GT(vel_magnitude, 0.5);  // Should be around 1.0

    // 3. Acceleration should be non-zero due to curvature term (centripetal acceleration)
    // Even though s_ddot = 0, we have q_ddot = q''(s) * s_dot^2
    // For circular motion, this is the centripetal term
    const double accel_magnitude =
        std::sqrt((sample.acceleration(0) * sample.acceleration(0)) + (sample.acceleration(1) * sample.acceleration(1)));
    BOOST_CHECK_GT(accel_magnitude, 0.01);  // Should have centripetal acceleration
}

// Sentinel state tests

BOOST_AUTO_TEST_CASE(cursor_seek_to_negative_time_becomes_sentinel) {
    // Verify seeking to negative time produces sentinel state
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 2.0;
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = 1.0, .s_ddot = 0.0}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Seek to negative time
    cursor.seek(trajectory::seconds{-0.5});

    // Cursor should be at sentinel
    BOOST_CHECK(cursor == cursor.end());

    // Sampling at sentinel should throw
    BOOST_CHECK_THROW(cursor.sample(), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(cursor_seek_beyond_duration_becomes_sentinel) {
    // Verify seeking beyond duration produces sentinel state
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 2.0;
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = 1.0, .s_ddot = 0.0}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Seek beyond duration
    cursor.seek(trajectory::seconds{2.5});

    // Cursor should be at sentinel
    BOOST_CHECK(cursor == cursor.end());

    // Sampling at sentinel should throw
    BOOST_CHECK_THROW(cursor.sample(), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(cursor_seek_by_from_sentinel_state) {
    // Verify seek_by doesn't change sentinel state
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 2.0;
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = 1.0, .s_ddot = 0.0},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = 1.0, .s_ddot = 0.0}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    auto cursor = traj.create_cursor();

    // Test negative sentinel
    cursor.seek(trajectory::seconds{-0.5});
    BOOST_CHECK(cursor == cursor.end());

    cursor.seek_by(trajectory::seconds{1.0});  // Try to advance
    BOOST_CHECK(cursor == cursor.end());       // Should still be sentinel

    // Test positive sentinel
    cursor.seek(trajectory::seconds{2.5});
    BOOST_CHECK(cursor == cursor.end());

    cursor.seek_by(trajectory::seconds{-1.0});  // Try to go backward
    BOOST_CHECK(cursor == cursor.end());        // Should still be sentinel
}

BOOST_AUTO_TEST_SUITE_END()

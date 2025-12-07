// End-to-end integration tests
// Extracted from test.cpp lines 2143-2257

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/hertz.hpp>

#include <boost/test/unit_test.hpp>

#include "test_utils.hpp"

namespace {

using namespace viam::trajex::totg;

trajectory create_trajectory_with_integration_points(path p, std::vector<trajectory::integration_point> points) {
    const trajectory::options opts{.max_velocity = xt::ones<double>({p.dof()}), .max_acceleration = xt::ones<double>({p.dof()})};
    return trajectory::create(std::move(p), opts, std::move(points));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(end_to_end_tests)

BOOST_AUTO_TEST_CASE(waypoints_to_samples_smoke_test) {
    using namespace viam::trajex;
    using namespace viam::trajex::totg;
    using namespace viam::trajex::totg::test;

    // 1. Create waypoints
    const xt::xarray<double> waypoints = {
        {0.0, 0.0},  // Start
        {1.0, 0.0},  // Move right
        {1.0, 1.0}   // Move up
    };

    // 2. Create path
    path p = path::create(waypoints);  // Linear segments only
    BOOST_CHECK_EQUAL(p.size(), 2U);
    BOOST_CHECK_CLOSE(static_cast<double>(p.length()), 2.0, 0.001);

    // 3. Create trajectory with known integration points for testing
    // Simple constant velocity trajectory
    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}},
        {.time = trajectory::seconds{2.0}, .s = arc_length{2.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // 4. Verify trajectory has correct duration
    BOOST_CHECK_CLOSE(traj.duration().count(), 2.0, 0.001);

    // 5. Create uniform sampler
    const uniform_sampler sampler{trajectory::seconds{0.1}};

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
    using namespace viam::trajex;
    using namespace viam::trajex::totg;
    using namespace viam::trajex::totg::test;
    using viam::trajex::arc_length;

    // Create simple trajectory with known integration points
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}};
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}},
        {.time = trajectory::seconds{1.0}, .s = arc_length{std::sqrt(2.0)}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    // Create cursor
    auto cursor = traj.create_cursor();

    // Verify initial state
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.0);
    BOOST_CHECK_EQUAL(&cursor.trajectory(), &traj);

    // Advance and sample
    cursor.seek(trajectory::seconds{0.5});
    BOOST_CHECK_EQUAL(cursor.time().count(), 0.5);

    auto sample = cursor.sample();
    BOOST_CHECK_EQUAL(sample.time.count(), 0.5);
    BOOST_CHECK_EQUAL(sample.configuration.shape(0), 2U);
}

BOOST_AUTO_TEST_CASE(quantized_sampler_end_to_end) {
    using namespace viam::trajex;
    using namespace viam::trajex::totg;
    using namespace viam::trajex::totg::test;
    using viam::trajex::arc_length;
    using viam::trajex::types::hertz;

    // Create trajectory with known integration points
    // Path length = 1.0, duration = 1.0s, constant velocity
    // At 8 Hz: putative = 8, num_samples = 9, dt = 1.0/8 = 0.125 (exact in binary!)
    xt::xarray<double> waypoints = xt::xarray<double>::from_shape({2, 1});
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 1.0;
    path p = path::create(waypoints);

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}},
        {.time = trajectory::seconds{1.0}, .s = arc_length{1.0}, .s_dot = arc_velocity{1.0}, .s_ddot = arc_acceleration{0.0}}};
    const trajectory traj = create_trajectory_with_integration_points(std::move(p), std::move(points));

    BOOST_REQUIRE_EQUAL(traj.duration().count(), 1.0);

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

BOOST_AUTO_TEST_CASE(ur_arm_incremental_waypoints_with_reversals) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Real UR arm waypoint sequence that includes direction reversals
    // Values are in degrees, convert to radians
    constexpr double deg_to_rad = M_PI / 180.0;

    // All waypoints including the initial all-zeros position
    std::vector<std::array<double, 6>> waypoints_deg = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 0: Start at zero
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 1
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 2
        // TODO(RSDK-12711): Add these waypoints back in once we do not need to
        // break up segments according to their dot product.
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 3: reversal back to position 1
        // {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // 4
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 5: reversal back to position 1 again
        // {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 6
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 7
        // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 8: back to zero
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 9
        // {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 10
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 11
        // {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // 12
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 13
        // {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 14
        // {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 15
        // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 16: back to zero again
    };

    // Convert to radians and create xt::xarray
    const std::vector<size_t> shape = {waypoints_deg.size(), 6};
    xt::xarray<double> waypoints_rad = xt::zeros<double>(shape);
    for (size_t i = 0; i < waypoints_deg.size(); ++i) {
        for (size_t j = 0; j < 6; ++j) {
            waypoints_rad(i, j) = waypoints_deg[i][j] * deg_to_rad;
        }
    }

    // Typical UR arm constraints (from working configuration)
    trajectory::options opts;
    opts.max_velocity = xt::ones<double>({6}) * (50 * deg_to_rad);         // 50 deg/s = 0.873 rad/s
    opts.max_acceleration = xt::ones<double>({6}) * (150.0 * deg_to_rad);  // 150 deg/s^2 = 2.618 rad/s^2

    // Observer to log integration events
    struct test_observer : trajectory::integration_observer {
        void on_started_forward_integration(trajectory::phase_point pt) override {
            std::cout << "[FORWARD START] s=" << pt.s << " s_dot=" << pt.s_dot << "\n";
        }
        void on_hit_limit_curve(trajectory::phase_point pt,
                                viam::trajex::arc_velocity s_dot_max_acc,
                                viam::trajex::arc_velocity s_dot_max_vel) override {
            std::cout << "[HIT LIMIT] s=" << pt.s << " s_dot=" << pt.s_dot << " acc_limit=" << s_dot_max_acc
                      << " vel_limit=" << s_dot_max_vel << "\n";
        }
        void on_started_backward_integration(trajectory::phase_point pt) override {
            std::cout << "[BACKWARD START] s=" << pt.s << " s_dot=" << pt.s_dot << "\n";
        }
        void on_trajectory_extended(const trajectory& traj) override {
            std::cout << "[SPLICE] points=" << traj.get_integration_points().size() << " duration=" << traj.duration().count() << "s\n";
        }
    };

    // Try incrementally longer subsets: [0,1], [0,1,2], [0,1,2,3], etc.
    for (size_t num_waypoints = 2; num_waypoints <= waypoints_deg.size(); ++num_waypoints) {
        BOOST_TEST_CONTEXT("Testing " << num_waypoints << " waypoints") {
            // Extract subset
            const xt::xarray<double> subset = xt::view(waypoints_rad, xt::range(0, num_waypoints), xt::all());

            // Create path with blending to make it differentiable at waypoints
            // Max deviation of 0.1 radians allows circular blends around sharp corners
            path::options path_opts;
            path_opts.set_max_deviation(0.1);
            path p = path::create(subset, path_opts);

            std::cout << "\n=== Testing " << num_waypoints << " waypoints (path length=" << static_cast<double>(p.length()) << ") ===\n";

            // Add observer to track what's happening
            test_observer observer;
            opts.observer = &observer;

            try {
                const trajectory traj = trajectory::create(std::move(p), opts);

                // If we succeeded, verify basic properties
                BOOST_CHECK(traj.duration().count() > 0.0);

                // Verify we can sample it
                auto sampler = uniform_sampler::quantized_for_trajectory(traj, hertz{5.0});
                int sample_count = 0;
                for (const auto& s : traj.samples(sampler)) {
                    BOOST_CHECK_EQUAL(s.configuration.shape(0), 6U);
                    ++sample_count;
                }
                BOOST_CHECK(sample_count > 0);

                BOOST_TEST_MESSAGE("PASS: Succeeded with " << num_waypoints << " waypoints");
            } catch (const std::exception& e) {
                // Log the failure and fail the test (but continue testing other waypoint counts)
                BOOST_TEST_MESSAGE("FAIL: " << num_waypoints << " waypoints: " << e.what());
                BOOST_CHECK_MESSAGE(false, "Failed to generate trajectory for " << num_waypoints << " waypoints: " << e.what());
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()

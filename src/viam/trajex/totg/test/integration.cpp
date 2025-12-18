#include <deque>

#include <boost/test/unit_test.hpp>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/hertz.hpp>

#include "test_utils.hpp"

namespace {

using namespace viam::trajex::totg;
using viam::trajex::arc_length;
using viam::trajex::arc_velocity;

trajectory create_trajectory_with_integration_points(path p, std::vector<trajectory::integration_point> points) {
    const trajectory::options opts{.max_velocity = xt::ones<double>({p.dof()}), .max_acceleration = xt::ones<double>({p.dof()})};
    return trajectory::create(std::move(p), opts, std::move(points));
}

// Observer that validates expectations as events occur

class expectation_observer : public trajectory::integration_observer {
   public:
    struct expected_forward_start {
        arc_length s;
        arc_velocity s_dot;
        std::optional<double> tolerance_percent;
    };

    struct expected_hit_limit {
        arc_length s;
        arc_velocity s_dot;
        std::optional<arc_velocity> s_dot_max_acc;
        std::optional<arc_velocity> s_dot_max_vel;
        std::optional<double> tolerance_percent;
    };

    struct expected_backward_start {
        arc_length s;
        arc_velocity s_dot;
        std::optional<double> tolerance_percent;
    };

    struct expected_splice {
        std::optional<trajectory::seconds> duration;
        std::optional<double> tolerance_percent;
    };

    using integration_expectation = std::variant<expected_forward_start, expected_hit_limit, expected_backward_start, expected_splice>;

    explicit expectation_observer(double default_tolerance_percent = 0.1) : default_tolerance_percent_(default_tolerance_percent) {}

    expectation_observer& expect_forward_start(arc_length s, arc_velocity s_dot, std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_forward_start{s, s_dot, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_hit_limit(arc_length s,
                                           arc_velocity s_dot,
                                           std::optional<arc_velocity> acc_limit = std::nullopt,
                                           std::optional<arc_velocity> vel_limit = std::nullopt,
                                           std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_hit_limit{s, s_dot, acc_limit, vel_limit, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_backward_start(arc_length s, arc_velocity s_dot, std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_backward_start{s, s_dot, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_splice(std::optional<trajectory::seconds> duration = std::nullopt,
                                        std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_splice{duration, tolerance_percent});
        return *this;
    }

    void verify_all_expectations_met() const {
        BOOST_TEST_CONTEXT("Checking all expectations were met") {
            BOOST_CHECK_MESSAGE(expectations_.empty(), "Not all expected events occurred. Remaining: " << expectations_.size());
        }
    }

    void on_started_forward_integration(const trajectory& /*traj*/, trajectory::phase_point pt) override {
        BOOST_TEST_CONTEXT("on_started_forward_integration event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(), "Unexpected forward_start event at s=" << pt.s << " s_dot=" << pt.s_dot);

            const auto* expected = std::get_if<expected_forward_start>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got forward_start at s=" << pt.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            BOOST_CHECK_CLOSE(static_cast<double>(pt.s), static_cast<double>(expected->s), tol);
            BOOST_CHECK_CLOSE(static_cast<double>(pt.s_dot), static_cast<double>(expected->s_dot), tol);

            expectations_.pop_front();
        }
    }

    void on_hit_limit_curve(const trajectory& /*traj*/,
                            trajectory::phase_point pt,
                            arc_velocity s_dot_max_acc,
                            arc_velocity s_dot_max_vel) override {
        BOOST_TEST_CONTEXT("on_hit_limit_curve event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(), "Unexpected hit_limit event at s=" << pt.s << " s_dot=" << pt.s_dot);

            const auto* expected = std::get_if<expected_hit_limit>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got hit_limit at s=" << pt.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            BOOST_CHECK_CLOSE(static_cast<double>(pt.s), static_cast<double>(expected->s), tol);
            BOOST_CHECK_CLOSE(static_cast<double>(pt.s_dot), static_cast<double>(expected->s_dot), tol);

            if (expected->s_dot_max_acc.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(s_dot_max_acc), static_cast<double>(*expected->s_dot_max_acc), tol);
            }

            if (expected->s_dot_max_vel.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(s_dot_max_vel), static_cast<double>(*expected->s_dot_max_vel), tol);
            }

            expectations_.pop_front();
        }
    }

    void on_started_backward_integration(const trajectory& /*traj*/, trajectory::phase_point pt) override {
        BOOST_TEST_CONTEXT("on_started_backward_integration event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(), "Unexpected backward_start event at s=" << pt.s << " s_dot=" << pt.s_dot);

            const auto* expected = std::get_if<expected_backward_start>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got backward_start at s=" << pt.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            BOOST_CHECK_CLOSE(static_cast<double>(pt.s), static_cast<double>(expected->s), tol);
            BOOST_CHECK_CLOSE(static_cast<double>(pt.s_dot), static_cast<double>(expected->s_dot), tol);

            expectations_.pop_front();
        }
    }

    void on_trajectory_extended(const trajectory& traj) override {
        BOOST_TEST_CONTEXT("on_trajectory_extended event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(), "Unexpected splice event at duration=" << traj.duration().count());

            const auto* expected = std::get_if<expected_splice>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got splice at duration=" << traj.duration().count());

            if (expected->duration.has_value()) {
                const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);
                BOOST_CHECK_CLOSE(traj.duration().count(), expected->duration->count(), tol);
            }

            expectations_.pop_front();
        }
    }

   private:
    double default_tolerance_percent_;
    std::deque<integration_expectation> expectations_;
};

// Validates fundamental trajectory invariants

void validate_trajectory_invariants(const trajectory& traj, double tolerance_percent = 0.1) {
    const auto& points = traj.get_integration_points();
    const auto& opts = traj.get_options();
    const auto& p = traj.path();

    BOOST_TEST_CONTEXT("Validating trajectory invariants") {
        // Boundary conditions
        BOOST_TEST_CONTEXT("Boundary conditions") {
            BOOST_REQUIRE(!points.empty());

            const auto& first = points.front();
            BOOST_CHECK_SMALL(static_cast<double>(first.s), 1e-9);
            BOOST_CHECK_SMALL(static_cast<double>(first.s_dot), 1e-9);

            const auto& last = points.back();
            BOOST_CHECK_CLOSE(static_cast<double>(last.s), static_cast<double>(p.length()), tolerance_percent);
            BOOST_CHECK_SMALL(static_cast<double>(last.s_dot), 1e-9);
        }

        // Monotonicity
        BOOST_TEST_CONTEXT("Monotonicity") {
            for (size_t i = 1; i < points.size(); ++i) {
                BOOST_CHECK_LT(points[i - 1].time.count(), points[i].time.count());
                BOOST_CHECK_LE(static_cast<double>(points[i - 1].s), static_cast<double>(points[i].s));
            }
        }

        // Phase plane velocity limits
        BOOST_TEST_CONTEXT("Phase plane velocity limits") {
            auto cursor = p.create_cursor(arc_length{0.0});

            for (size_t i = 0; i < points.size(); ++i) {
                const auto& pt = points[i];
                cursor.seek(pt.s);

                const auto tangent = cursor.tangent();

                // Compute s_dot_max_vel from equation 22
                double s_dot_max_vel = std::numeric_limits<double>::infinity();
                for (size_t j = 0; j < opts.max_velocity.size(); ++j) {
                    const double tangent_abs = std::abs(tangent(static_cast<long>(j)));
                    if (tangent_abs > 1e-9) {
                        s_dot_max_vel = std::min(s_dot_max_vel, opts.max_velocity(static_cast<long>(j)) / tangent_abs);
                    }
                }

                // Compute s_dot_max_acc from equation 31
                double s_dot_max_acc = std::numeric_limits<double>::infinity();
                for (size_t j = 0; j < opts.max_acceleration.size(); ++j) {
                    const double tangent_val = tangent(static_cast<long>(j));
                    const double tangent_sq = tangent_val * tangent_val;
                    if (tangent_sq > 1e-9) {
                        s_dot_max_acc = std::min(s_dot_max_acc, std::sqrt(opts.max_acceleration(static_cast<long>(j)) / tangent_sq));
                    }
                }

                const double s_dot_limit = std::min(s_dot_max_vel, s_dot_max_acc);
                const double actual_s_dot = static_cast<double>(pt.s_dot);

                // Allow small violations due to tolerance
                const double allowed_limit = s_dot_limit * (1.0 + tolerance_percent / 100.0);

                BOOST_TEST_CONTEXT("Integration point " << i << " at s=" << static_cast<double>(pt.s)) {
                    BOOST_CHECK_LE(actual_s_dot, allowed_limit);

                    if (actual_s_dot > allowed_limit) {
                        BOOST_TEST_MESSAGE("s_dot=" << actual_s_dot << " exceeds limit=" << s_dot_limit << " (vel_limit=" << s_dot_max_vel
                                                    << ", acc_limit=" << s_dot_max_acc << ")");
                    }
                }
            }
        }
    }
}

// Test fixture for trajectory integration tests

struct trajectory_test_fixture {
    path::options path_opts;
    trajectory::options traj_opts;
    expectation_observer observer;
    double validation_tolerance_percent = 0.1;
    size_t dof_;
    std::vector<xt::xarray<double>> waypoints_;

    explicit trajectory_test_fixture(size_t dof = 6, double expectation_tolerance_percent = 0.1)
        : observer(expectation_tolerance_percent), dof_(dof) {}

    trajectory_test_fixture& add_waypoint_deg(const std::vector<double>& wp_deg) {
        xt::xarray<double> wp_rad = xt::zeros<double>({dof_});
        constexpr double deg_to_rad = M_PI / 180.0;
        for (size_t i = 0; i < dof_ && i < wp_deg.size(); ++i) {
            wp_rad(static_cast<long>(i)) = wp_deg[i] * deg_to_rad;
        }
        waypoints_.push_back(std::move(wp_rad));
        return *this;
    }

    trajectory_test_fixture& add_waypoint_rad(const std::vector<double>& wp_rad) {
        xt::xarray<double> wp = xt::zeros<double>({dof_});
        for (size_t i = 0; i < dof_ && i < wp_rad.size(); ++i) {
            wp(static_cast<long>(i)) = wp_rad[i];
        }
        waypoints_.push_back(std::move(wp));
        return *this;
    }

    trajectory_test_fixture& set_max_velocity(const xt::xarray<double>& max_vel) {
        traj_opts.max_velocity = max_vel;
        return *this;
    }

    trajectory_test_fixture& set_max_acceleration(const xt::xarray<double>& max_acc) {
        traj_opts.max_acceleration = max_acc;
        return *this;
    }

    trajectory_test_fixture& set_max_deviation(double max_dev) {
        path_opts.set_max_deviation(max_dev);
        return *this;
    }

    trajectory create_and_validate() {
        BOOST_REQUIRE_MESSAGE(!waypoints_.empty(), "Must add waypoints before calling create_and_validate");

        // Convert vector of waypoints to 2D xarray
        xt::xarray<double> waypoints = xt::zeros<double>({waypoints_.size(), dof_});
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            for (size_t j = 0; j < dof_; ++j) {
                waypoints(i, j) = waypoints_[i](static_cast<long>(j));
            }
        }

        // Create path
        path p = path::create(waypoints, path_opts);

        // Attach observer
        traj_opts.observer = &observer;

        // Generate trajectory
        trajectory traj = trajectory::create(std::move(p), traj_opts);

        // Verify all expectations were met
        observer.verify_all_expectations_met();

        // Validate trajectory invariants
        validate_trajectory_invariants(traj, validation_tolerance_percent);

        return traj;
    }
};

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
        void on_started_forward_integration(const trajectory&, trajectory::phase_point pt) override {
            std::cout << "[FORWARD START] s=" << pt.s << " s_dot=" << pt.s_dot << "\n";
        }
        void on_hit_limit_curve(const trajectory&,
                                trajectory::phase_point pt,
                                viam::trajex::arc_velocity s_dot_max_acc,
                                viam::trajex::arc_velocity s_dot_max_vel) override {
            std::cout << "[HIT LIMIT] s=" << pt.s << " s_dot=" << pt.s_dot << " acc_limit=" << s_dot_max_acc
                      << " vel_limit=" << s_dot_max_vel << "\n";
        }
        void on_started_backward_integration(const trajectory&, trajectory::phase_point pt) override {
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

BOOST_AUTO_TEST_CASE(three_waypoint_baseline_behavior) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // BASELINE: First working three-waypoint trajectory case
    // This documents CURRENT algorithm behavior as of initial implementation.
    //
    // If this fails after changes:
    //   1. Check: Is the new trajectory VALID?
    //   2. Check: Is it BETTER? (faster, smoother, fewer violations)
    //   3. If YES to both: Update these expectations
    //   4. If NO: You broke it - fix the regression

    constexpr double deg_to_rad = M_PI / 180.0;

    // Create fixture
    trajectory_test_fixture fixture(6, 0.1);

    // Set constraints (same as ur_arm_incremental_waypoints_with_reversals)
    fixture.set_max_velocity(xt::ones<double>({6}) * (50.0 * deg_to_rad))
        .set_max_acceleration(xt::ones<double>({6}) * (1.0 * deg_to_rad))
        .set_max_deviation(0.1);

    // Override delta to match test expectations
    fixture.traj_opts.delta = trajectory::seconds{0.0001};

    // Add waypoints
    fixture.add_waypoint_deg({0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
        .add_waypoint_deg({-45.0, -45.0, 0.0, 0.0, 0.0, 0.0})
        .add_waypoint_deg({-45.0, -90.0, 0.0, 0.0, 0.0, 0.0});

    // Set up expectations based on observed behavior
    // Event sequence from running test:
    // [FORWARD START] s=0 s_dot=0
    // [HIT LIMIT] s=0.718023 s_dot=0.18827 acc_limit=0.152973 vel_limit=1.23413
    // [BACKWARD START] s=0.718022 s_dot=0.152973
    // [SPLICE] duration=7.7013s
    // [FORWARD START] s=0.718022 s_dot=0.152973
    // [HIT LIMIT] s=0.720836 s_dot=0.152973 acc_limit=0.152972 vel_limit=1.23049
    // [BACKWARD START] s=1.85532 s_dot=0
    // [SPLICE] duration=20.1621s

    fixture.observer.expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.718023},
                          arc_velocity{0.18827},
                          arc_velocity{0.15297},  // acc_limit
                          arc_velocity{1.23413}   // vel_limit
                          )
        .expect_backward_start(arc_length{0.71802}, arc_velocity{0.15297})
        .expect_splice(trajectory::seconds{7.7013})
        .expect_forward_start(arc_length{0.71802}, arc_velocity{0.15297})
        .expect_hit_limit(arc_length{0.72084},
                          arc_velocity{0.15297},
                          arc_velocity{0.15297},  // acc_limit
                          arc_velocity{1.23049}   // vel_limit
                          )
        .expect_backward_start(arc_length{1.85532}, arc_velocity{0.0})
        .expect_splice(trajectory::seconds{20.1621});

    const trajectory traj = fixture.create_and_validate();
}

BOOST_AUTO_TEST_SUITE_END()

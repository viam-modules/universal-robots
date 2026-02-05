#include <algorithm>
#include <cmath>
#include <deque>

#include <boost/test/unit_test.hpp>

#if defined(__has_include) && (__has_include(<xtensor/containers/xadapt.hpp>))
#include <xtensor/containers/xadapt.hpp>
#else
#include <xtensor/xadapt.hpp>
#endif

#include <Eigen/Core>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/test/test_utils.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/angles.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_operations.hpp>
#include <viam/trajex/types/hertz.hpp>

// Legacy trajectory generator
#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>

namespace {

using namespace viam::trajex::totg;
using viam::trajex::arc_length;
using viam::trajex::arc_velocity;
using viam::trajex::degrees_to_radians;

constexpr bool k_log_met_expectations = true;

trajectory create_trajectory_with_integration_points(path p, std::vector<trajectory::integration_point> points) {
    const trajectory::options opts{.max_velocity = xt::ones<double>({p.dof()}), .max_acceleration = xt::ones<double>({p.dof()})};
    return trajectory::create(std::move(p), opts, std::move(points));
}

// Observer that validates expectations as events occur

class expectation_observer final : public trajectory::integration_observer {
   public:
    struct expected_forward_start {
        std::optional<arc_length> s;
        std::optional<arc_velocity> s_dot;
        std::optional<double> tolerance_percent;
    };

    struct expected_hit_limit {
        std::optional<arc_length> s;
        std::optional<arc_velocity> s_dot;
        std::optional<arc_velocity> s_dot_max_acc;
        std::optional<arc_velocity> s_dot_max_vel;
        std::optional<double> tolerance_percent;
    };

    struct expected_backward_start {
        std::optional<arc_length> s;
        std::optional<arc_velocity> s_dot;
        std::optional<trajectory::switching_point_kind> kind;
        std::optional<double> tolerance_percent;
    };

    struct expected_splice {
        std::optional<trajectory::seconds> duration;
        std::optional<std::size_t> num_pruned;
        std::optional<double> tolerance_percent;
    };

    using integration_expectation = std::variant<expected_forward_start, expected_hit_limit, expected_backward_start, expected_splice>;

    explicit expectation_observer(double default_tolerance_percent = 0.1) : default_tolerance_percent_(default_tolerance_percent) {}

    expectation_observer& expect_forward_start(std::optional<arc_length> s = std::nullopt,
                                               std::optional<arc_velocity> s_dot = std::nullopt,
                                               std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_forward_start{s, s_dot, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_hit_limit(std::optional<arc_length> s = std::nullopt,
                                           std::optional<arc_velocity> s_dot = std::nullopt,
                                           std::optional<arc_velocity> acc_limit = std::nullopt,
                                           std::optional<arc_velocity> vel_limit = std::nullopt,
                                           std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_hit_limit{s, s_dot, acc_limit, vel_limit, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_backward_start(std::optional<arc_length> s = std::nullopt,
                                                std::optional<arc_velocity> s_dot = std::nullopt,
                                                std::optional<trajectory::switching_point_kind> kind = std::nullopt,
                                                std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_backward_start{s, s_dot, kind, tolerance_percent});
        return *this;
    }

    expectation_observer& expect_splice(std::optional<trajectory::seconds> duration = std::nullopt,
                                        std::optional<std::size_t> num_pruned = std::nullopt,
                                        std::optional<double> tolerance_percent = std::nullopt) {
        expectations_.push_back(expected_splice{duration, num_pruned, tolerance_percent});
        return *this;
    }

    void verify_all_expectations_met() const {
        BOOST_TEST_CONTEXT("Checking all expectations were met") {
            BOOST_CHECK_MESSAGE(expectations_.empty(), "Not all expected events occurred. Remaining: " << expectations_.size());
        }
    }

    const std::deque<integration_expectation>& get_expectations() const {
        return expectations_;
    }

    double get_default_tolerance_percent() const {
        return default_tolerance_percent_;
    }

    void on_started_forward_integration(const trajectory&, started_forward_event event) override {
        BOOST_TEST_CONTEXT("on_started_forward_integration event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(),
                                  "Unexpected forward_start event at s=" << event.start.s << " s_dot=" << event.start.s_dot);

            const auto* expected = std::get_if<expected_forward_start>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got forward_start at s=" << event.start.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            if (expected->s.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.start.s), static_cast<double>(*expected->s), tol);
            }
            if (expected->s_dot.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.start.s_dot), static_cast<double>(*expected->s_dot), tol);
            }

            if constexpr (k_log_met_expectations) {
                std::cout << "Met expectation: forward_start at s=" << event.start.s << " s_dot=" << event.start.s_dot << "\n";
            }

            expectations_.pop_front();
        }
    }

    void on_hit_limit_curve(const trajectory&, limit_hit_event event) override {
        BOOST_TEST_CONTEXT("on_hit_limit_curve event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(),
                                  "Unexpected hit_limit event at s=" << event.breach.s << " s_dot=" << event.breach.s_dot);

            const auto* expected = std::get_if<expected_hit_limit>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got hit_limit at s=" << event.breach.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            if (expected->s.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.breach.s), static_cast<double>(*expected->s), tol);
            }
            if (expected->s_dot.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.breach.s_dot), static_cast<double>(*expected->s_dot), tol);
            }

            if (expected->s_dot_max_acc.has_value()) {
                const double actual = static_cast<double>(event.s_dot_max_acc);
                const double expected_val = static_cast<double>(*expected->s_dot_max_acc);
                if (std::isinf(expected_val)) {
                    BOOST_CHECK_EQUAL(actual, expected_val);
                } else {
                    BOOST_CHECK_CLOSE(actual, expected_val, tol);
                }
            }

            if (expected->s_dot_max_vel.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.s_dot_max_vel), static_cast<double>(*expected->s_dot_max_vel), tol);
            }

            if constexpr (k_log_met_expectations) {
                std::cout << "Met expectation: hit_limit at s=" << event.breach.s << " s_dot=" << event.breach.s_dot
                          << " acc_limit=" << event.s_dot_max_acc << " vel_limit=" << event.s_dot_max_vel << "\n";
            }

            expectations_.pop_front();
        }
    }

    void on_started_backward_integration(const trajectory&, started_backward_event event) override {
        BOOST_TEST_CONTEXT("on_started_backward_integration event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(),
                                  "Unexpected backward_start event at s=" << event.start.s << " s_dot=" << event.start.s_dot);

            const auto* expected = std::get_if<expected_backward_start>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got backward_start at s=" << event.start.s);

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            if (expected->s.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.start.s), static_cast<double>(*expected->s), tol);
            }
            if (expected->s_dot.has_value()) {
                BOOST_CHECK_CLOSE(static_cast<double>(event.start.s_dot), static_cast<double>(*expected->s_dot), tol);
            }

            // Check switching point kind if expectation includes it
            if (expected->kind.has_value()) {
                BOOST_CHECK_EQUAL(static_cast<int>(event.kind), static_cast<int>(*expected->kind));
            }

            if constexpr (k_log_met_expectations) {
                std::cout << "Met expectation: backward_start at s=" << event.start.s << " s_dot=" << event.start.s_dot
                          << " kind=" << static_cast<int>(event.kind) << "\n";
            }

            expectations_.pop_front();
        }
    }

    void on_trajectory_extended(const trajectory& traj, splice_event event) override {
        BOOST_TEST_CONTEXT("on_trajectory_extended event") {
            BOOST_REQUIRE_MESSAGE(!expectations_.empty(), "Unexpected splice event at duration=" << traj.duration().count());

            const auto* expected = std::get_if<expected_splice>(&expectations_.front());
            BOOST_REQUIRE_MESSAGE(expected != nullptr, "Expected different event type, got splice at duration=" << traj.duration().count());

            const double tol = expected->tolerance_percent.value_or(default_tolerance_percent_);

            if (expected->duration.has_value()) {
                BOOST_CHECK_CLOSE(traj.duration().count(), expected->duration->count(), tol);
            }

            if (expected->num_pruned.has_value()) {
                BOOST_CHECK_EQUAL(event.pruned.size(), *expected->num_pruned);
            }

            if constexpr (k_log_met_expectations) {
                std::cout << "Met expectation: splice at duration=" << traj.duration().count() << " pruned=" << event.pruned.size() << "\n";
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
    const auto& p = traj.path();

    BOOST_TEST_CONTEXT("Validating trajectory invariants") {
        // Boundary conditions
        BOOST_TEST_CONTEXT("Boundary conditions") {
            BOOST_REQUIRE(!points.empty());

            const auto& first = points.front();
            BOOST_CHECK_EQUAL(static_cast<double>(first.s), 0.0);
            BOOST_CHECK_EQUAL(static_cast<double>(first.s_dot), 0.0);
            BOOST_CHECK_EQUAL(static_cast<double>(first.time.count()), 0.0);

            const auto& last = points.back();
            BOOST_CHECK_EQUAL(static_cast<double>(last.s), static_cast<double>(p.length()));
            BOOST_CHECK_EQUAL(static_cast<double>(last.s_dot), 0.0);
            BOOST_CHECK_EQUAL(static_cast<double>(last.s_ddot), 0.0);
        }

        // Monotonicity
        BOOST_TEST_CONTEXT("Monotonicity") {
            for (size_t i = 1; i < points.size(); ++i) {
                BOOST_CHECK_LT(points[i - 1].time.count(), points[i].time.count());
                BOOST_CHECK_LT(static_cast<double>(points[i - 1].s), static_cast<double>(points[i].s));
            }
        }

        // Phase plane constraints (velocity limits and acceleration bounds)
        BOOST_TEST_CONTEXT("Phase plane constraints") {
            auto cursor = p.create_cursor(arc_length{0.0});

            for (size_t i = 0; i < points.size(); ++i) {
                const auto& pt = points[i];
                cursor.seek(pt.s);

                // Validate velocity limits
                const auto limits = traj.get_velocity_limits(cursor);
                const double s_dot_max_vel = static_cast<double>(limits.s_dot_max_vel);
                const double s_dot_max_acc = static_cast<double>(limits.s_dot_max_acc);
                const double s_dot_limit = std::min(s_dot_max_vel, s_dot_max_acc);
                const double actual_s_dot = static_cast<double>(pt.s_dot);

                // Validate acceleration bounds
                const auto bounds = traj.get_acceleration_bounds(cursor, pt.s_dot);
                const double s_ddot_min = static_cast<double>(bounds.s_ddot_min);
                const double s_ddot_max = static_cast<double>(bounds.s_ddot_max);
                const double actual_s_ddot = static_cast<double>(pt.s_ddot);

                BOOST_TEST_CONTEXT("Integration point " << i << " at s=" << static_cast<double>(pt.s)) {
                    BOOST_CHECK(std::isfinite(actual_s_dot));
                    BOOST_CHECK(std::isfinite(actual_s_ddot));
                    BOOST_CHECK_GE(actual_s_dot, 0.0);

                    // Check velocity limits
                    const double allowed_vel_limit = s_dot_limit * (1.0 + tolerance_percent / 100.0);
                    BOOST_CHECK_LE(actual_s_dot, allowed_vel_limit);

                    if (actual_s_dot > allowed_vel_limit) {
                        BOOST_TEST_MESSAGE("s_dot=" << actual_s_dot << " exceeds limit=" << s_dot_limit << " (vel_limit=" << s_dot_max_vel
                                                    << ", acc_limit=" << s_dot_max_acc << ")");
                    }

                    // Check acceleration bounds
                    const double tolerance_margin = std::max(std::abs(s_ddot_max), std::abs(s_ddot_min)) * (tolerance_percent / 100.0);
                    const double allowed_min = s_ddot_min - tolerance_margin;
                    const double allowed_max = s_ddot_max + tolerance_margin;

                    BOOST_CHECK_GE(actual_s_ddot, allowed_min);
                    BOOST_CHECK_LE(actual_s_ddot, allowed_max);

                    if (actual_s_ddot < allowed_min || actual_s_ddot > allowed_max) {
                        BOOST_TEST_MESSAGE("s_ddot=" << actual_s_ddot << " outside bounds=[" << s_ddot_min << ", " << s_ddot_max << "]");
                    }
                }
            }
        }

        // Integration point kinematic consistency
        //
        // Validates that consecutive integration points form a consistent kinematic sequence.
        // Starting from point i with (s, s_dot, s_ddot), constant-acceleration integration
        // should approximately reach point i+1's (s, s_dot). This checks that the stored
        // acceleration values and time steps produce the claimed trajectory.
        BOOST_TEST_CONTEXT("Integration point kinematic consistency") {
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                const auto& curr = points[i];
                const auto& next = points[i + 1];

                const auto dt = next.time - curr.time;

                // Constant-acceleration kinematics: integrate from current point
                const auto predicted_s_dot = curr.s_dot + (curr.s_ddot * dt);
                const auto predicted_s = curr.s + (curr.s_dot * dt) + (0.5 * (curr.s_ddot * dt) * dt);

                BOOST_TEST_CONTEXT("Point " << i << " -> " << (i + 1) << " (dt=" << dt.count() << "s, s=" << static_cast<double>(curr.s)
                                            << " -> " << static_cast<double>(next.s) << ")") {
                    // Position check (always use relative error)
                    BOOST_CHECK_CLOSE(static_cast<double>(predicted_s), static_cast<double>(next.s), tolerance_percent);

                    // Velocity check: use absolute error for near-zero, relative error otherwise
                    const double next_s_dot_val = static_cast<double>(next.s_dot);
                    const double predicted_s_dot_val = static_cast<double>(predicted_s_dot);

                    if (std::abs(next_s_dot_val) < 1e-9) {
                        // Near-zero velocity: check absolute error
                        // BOOST_CHECK_CLOSE would divide by ~zero and give meaningless result
                        const double abs_error = std::abs(predicted_s_dot_val - next_s_dot_val);
                        BOOST_TEST_CONTEXT("Near-zero velocity: checking absolute error") {
                            BOOST_CHECK_LT(abs_error, 1e-9);
                            if (abs_error >= 1e-9) {
                                BOOST_TEST_MESSAGE("Absolute error: " << abs_error << " (predicted=" << predicted_s_dot_val
                                                                      << ", actual=" << next_s_dot_val << ")");
                            }
                        }
                    } else {
                        // Normal case: check relative error
                        BOOST_CHECK_CLOSE(predicted_s_dot_val, next_s_dot_val, tolerance_percent);
                        if (std::abs((predicted_s_dot_val - next_s_dot_val) / next_s_dot_val) * 100.0 > tolerance_percent) {
                            BOOST_TEST_MESSAGE("Velocity mismatch: predicted=" << predicted_s_dot_val << ", actual=" << next_s_dot_val
                                                                               << ", curr.s_ddot=" << static_cast<double>(curr.s_ddot));
                        }
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
    xt::xarray<double> waypoints_;

    // Trajectory-wide expectations (optional)
    struct expected_duration {
        trajectory::seconds duration;
        std::optional<double> tolerance_percent;
    };
    struct expected_path_length {
        arc_length length;
        std::optional<double> tolerance_percent;
    };

    std::optional<expected_duration> expected_duration_;
    std::optional<size_t> expected_integration_point_count_;
    std::optional<expected_path_length> expected_path_length_;

    // Legacy comparison (A/B testing against reference implementation)
    bool compare_with_legacy_ = false;
    std::optional<double> legacy_duration_;
    std::optional<double> legacy_path_length_;

    explicit trajectory_test_fixture(size_t dof = 6, double expectation_tolerance_percent = 0.1)
        : observer(expectation_tolerance_percent), dof_(dof) {}

    trajectory_test_fixture& set_waypoints_deg(const xt::xarray<double>& waypoints_deg) {
        waypoints_ = degrees_to_radians(waypoints_deg);
        return *this;
    }

    trajectory_test_fixture& set_waypoints_rad(const xt::xarray<double>& waypoints_rad) {
        waypoints_ = waypoints_rad;
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

    trajectory_test_fixture& expect_duration(trajectory::seconds duration, std::optional<double> tolerance_percent = std::nullopt) {
        expected_duration_ = expected_duration{duration, tolerance_percent};
        return *this;
    }

    trajectory_test_fixture& expect_integration_point_count(size_t count) {
        expected_integration_point_count_ = count;
        return *this;
    }

    trajectory_test_fixture& expect_path_length(arc_length length, std::optional<double> tolerance_percent = std::nullopt) {
        expected_path_length_ = expected_path_length{length, tolerance_percent};
        return *this;
    }

    trajectory_test_fixture& enable_legacy_comparison() {
        compare_with_legacy_ = true;
        return *this;
    }

    void try_legacy_comparison(const xt::xarray<double>& waypoints) {
        try {
            // Convert waypoints to legacy format (std::list<Eigen::VectorXd>)
            std::list<Eigen::VectorXd> legacy_waypoints;
            for (size_t i = 0; i < waypoints.shape(0); ++i) {
                Eigen::VectorXd wp(static_cast<Eigen::Index>(dof_));
                for (size_t j = 0; j < dof_; ++j) {
                    wp(static_cast<Eigen::Index>(j)) = waypoints(i, j);
                }
                legacy_waypoints.push_back(wp);
            }

            // Convert limits to Eigen format
            Eigen::VectorXd legacy_max_vel(static_cast<Eigen::Index>(dof_));
            Eigen::VectorXd legacy_max_acc(static_cast<Eigen::Index>(dof_));
            for (size_t i = 0; i < dof_; ++i) {
                legacy_max_vel(static_cast<Eigen::Index>(i)) = traj_opts.max_velocity(i);
                legacy_max_acc(static_cast<Eigen::Index>(i)) = traj_opts.max_acceleration(i);
            }

            // Create legacy path and trajectory
            const Path legacy_path(legacy_waypoints, path_opts.max_blend_deviation());
            const Trajectory legacy_traj(legacy_path, legacy_max_vel, legacy_max_acc, traj_opts.delta.count());

            if (legacy_traj.isValid()) {
                legacy_duration_ = legacy_traj.getDuration();
                legacy_path_length_ = legacy_path.getLength();
                std::cout << "[LEGACY] duration=" << *legacy_duration_ << "s, path_length=" << *legacy_path_length_ << "\n";
            } else {
                std::cout << "[LEGACY] FAILED: invalid trajectory\n";
            }
        } catch (const std::exception& e) {
            std::cout << "[LEGACY] FAILED: " << e.what() << "\n";
        }
    }

    void validate_against_legacy(const trajectory& traj) {
        BOOST_TEST_CONTEXT("Legacy comparison") {
            const double new_duration = traj.duration().count();
            const double new_path_length = static_cast<double>(traj.path().length());

            if (legacy_path_length_.has_value()) {
                const double path_ratio = new_path_length / *legacy_path_length_;
                const double path_diff_percent = std::abs((new_path_length - *legacy_path_length_) / *legacy_path_length_) * 100.0;

                std::cout << "[COMPARISON] Path: new=" << new_path_length << ", legacy=" << *legacy_path_length_ << ", "
                          << "ratio=" << path_ratio << ", diff=" << path_diff_percent << "%\n";

                // Path should be about the same length (within 5%)
                BOOST_CHECK_CLOSE(new_path_length, *legacy_path_length_, 5.0);
            }

            if (legacy_duration_.has_value()) {
                const double duration_ratio = new_duration / *legacy_duration_;
                const double duration_diff_percent = ((new_duration - *legacy_duration_) / *legacy_duration_) * 100.0;

                std::cout << "[COMPARISON] Duration: new=" << new_duration << "s, legacy=" << *legacy_duration_ << "s, "
                          << "ratio=" << duration_ratio << ", diff=" << duration_diff_percent << "%\n";

                // We should do NO WORSE than legacy: duration should be about the same or shorter
                // Allow up to 5% slower (for numerical differences), but prefer faster
                BOOST_CHECK_LE(new_duration, *legacy_duration_ * 1.05);

                BOOST_TEST_MESSAGE("Duration comparison: new=" << new_duration << "s vs legacy=" << *legacy_duration_ << "s (ratio="
                                                               << duration_ratio << ", diff=" << duration_diff_percent << "%)");
            }
        }
    }

    trajectory create_and_validate() {
        BOOST_REQUIRE_MESSAGE(waypoints_.size() > 0, "Must set waypoints before calling create_and_validate");

        // Try legacy comparison if enabled
        if (compare_with_legacy_) {
            try_legacy_comparison(waypoints_);
        }

        // Create path
        path p = path::create(waypoints_, path_opts);

        // Validate path length expectation
        if (expected_path_length_.has_value()) {
            const double tol = expected_path_length_->tolerance_percent.value_or(validation_tolerance_percent);
            BOOST_TEST_CONTEXT("Path length expectation") {
                BOOST_CHECK_CLOSE(static_cast<double>(p.length()), static_cast<double>(expected_path_length_->length), tol);
            }
        }

        // Validate that expectations follow TOTG invariant:
        // Must have at least 3 events: forward_start, backward_start from endpoint, splice
        const auto& expectations = observer.get_expectations();
        BOOST_REQUIRE_MESSAGE(expectations.size() >= 3, "Must have at least 3 expected events (TOTG invariant)");

        BOOST_TEST_CONTEXT("Expectation coherence (TOTG invariant)") {
            // First must be forward_start
            const auto* first_forward = std::get_if<expectation_observer::expected_forward_start>(&expectations.front());
            BOOST_REQUIRE_MESSAGE(first_forward != nullptr, "First expectation must be forward_start (TOTG invariant)");

            // Second-to-last must be backward_start (from endpoint)
            const auto& second_last = expectations[expectations.size() - 2];
            const auto* backward = std::get_if<expectation_observer::expected_backward_start>(&second_last);
            BOOST_REQUIRE_MESSAGE(backward != nullptr, "Second-to-last expectation must be backward_start (TOTG invariant)");

            // Backward expectation must specify k_path_end kind (TOTG always ends with backward from endpoint)
            BOOST_REQUIRE_MESSAGE(backward->kind.has_value(),
                                  "Second-to-last expectation must specify switching point kind (TOTG invariant)");
            BOOST_REQUIRE_MESSAGE(*backward->kind == trajectory::switching_point_kind::k_path_end,
                                  "Second-to-last expectation must be backward_start with k_path_end kind (TOTG invariant)");

            // Sanity check: backward expectation should be approximately at path endpoint (if s is specified)
            if (backward->s.has_value()) {
                const double backward_tol = backward->tolerance_percent.value_or(observer.get_default_tolerance_percent());
                BOOST_CHECK_CLOSE(static_cast<double>(*backward->s), static_cast<double>(p.length()), backward_tol);
            }

            // Last must be splice
            const auto* splice = std::get_if<expectation_observer::expected_splice>(&expectations.back());
            BOOST_REQUIRE_MESSAGE(splice != nullptr, "Last expectation must be splice (TOTG invariant)");
        }

        // Attach observer
        traj_opts.observer = &observer;

        // Generate trajectory
        trajectory traj = trajectory::create(std::move(p), traj_opts);

        // Verify all expectations were met
        observer.verify_all_expectations_met();

        // Validate trajectory-wide expectations
        if (expected_duration_.has_value()) {
            const double tol = expected_duration_->tolerance_percent.value_or(validation_tolerance_percent);
            BOOST_TEST_CONTEXT("Trajectory duration expectation") {
                BOOST_CHECK_CLOSE(traj.duration().count(), expected_duration_->duration.count(), tol);
            }
        }

        if (expected_integration_point_count_.has_value()) {
            BOOST_TEST_CONTEXT("Integration point count expectation") {
                BOOST_CHECK_EQUAL(traj.get_integration_points().size(), *expected_integration_point_count_);
            }
        }

        // Compare with legacy if available
        if (legacy_duration_.has_value() || legacy_path_length_.has_value()) {
            validate_against_legacy(traj);
        }

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

    // 5. Create uniform sampler at ~10Hz (dt ~= 0.1s)
    const auto sampler = uniform_sampler::quantized_for_trajectory(traj, types::hertz{10.0});

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
    const xt::xarray<double> waypoints_deg = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 0: Start at zero
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 1
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 2
        // TODO(RSDK-12711): Add these waypoints back in once we do not need to
        // break up segments according to their dot product.
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 3: reversal back to position 1
        {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // 4
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 5: reversal back to position 1 again
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 6
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 7
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 8: back to zero
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 9
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 10
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 11
        {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0},    // 12
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 13
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 14
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 15
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 16: back to zero again
    };

    xt::xarray<double> waypoints_rad = degrees_to_radians(waypoints_deg);

    // Typical UR arm constraints (from working configuration)
    trajectory::options opts;
    opts.max_velocity = degrees_to_radians(xt::ones<double>({6}) * 150.0);    // 50 deg/s
    opts.max_acceleration = degrees_to_radians(xt::ones<double>({6}) * 5.0);  // 150 deg/s^2

    // Observer to log integration events
    class test_observer final : public trajectory::integration_observer {
       public:
        void on_started_forward_integration(const trajectory&, started_forward_event event) override {
            std::cout << "[FORWARD START] s=" << event.start.s << " s_dot=" << event.start.s_dot << "\n";
        }
        void on_hit_limit_curve(const trajectory&, limit_hit_event event) override {
            std::cout << "[HIT LIMIT] s=" << event.breach.s << " s_dot=" << event.breach.s_dot << " acc_limit=" << event.s_dot_max_acc
                      << " vel_limit=" << event.s_dot_max_vel << "\n";
        }
        void on_started_backward_integration(const trajectory&, started_backward_event event) override {
            std::cout << "[BACKWARD START] s=" << event.start.s << " s_dot=" << event.start.s_dot << "\n";
        }
        void on_trajectory_extended(const trajectory& traj, splice_event) override {
            std::cout << "[SPLICE] points=" << traj.get_integration_points().size() << " duration=" << traj.duration().count() << "s\n";
        }
    };

    // Try incrementally longer subsets: [0,1], [0,1,2], [0,1,2,3], etc.
    for (size_t num_waypoints = 2; num_waypoints <= waypoints_deg.shape(0); ++num_waypoints) {
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

            // NEW: Also try legacy trajectory generator for comparison
            double legacy_duration = -1.0;
            bool legacy_valid = false;
            try {
                // Convert waypoints to legacy format (std::list<Eigen::VectorXd>)
                std::list<Eigen::VectorXd> legacy_waypoints;
                for (size_t i = 0; i < num_waypoints; ++i) {
                    Eigen::VectorXd wp(6);
                    for (size_t j = 0; j < 6; ++j) {
                        wp(static_cast<Eigen::Index>(j)) = subset(i, j);
                    }
                    legacy_waypoints.push_back(wp);
                }

                // Convert limits to Eigen format
                Eigen::VectorXd legacy_max_vel(6);
                Eigen::VectorXd legacy_max_acc(6);
                for (size_t i = 0; i < 6; ++i) {
                    legacy_max_vel(static_cast<Eigen::Index>(i)) = opts.max_velocity(i);
                    legacy_max_acc(static_cast<Eigen::Index>(i)) = opts.max_acceleration(i);
                }

                // Create legacy path and trajectory
                const Path legacy_path(legacy_waypoints, 0.1);  // maxDeviation=0.1
                const Trajectory legacy_traj(legacy_path, legacy_max_vel, legacy_max_acc, 0.001);

                if (legacy_traj.isValid()) {
                    legacy_duration = legacy_traj.getDuration();
                    legacy_valid = true;
                    std::cout << "[LEGACY] duration=" << legacy_duration << "s\n";
                } else {
                    std::cout << "[LEGACY] FAILED: invalid trajectory\n";
                }
            } catch (const std::exception& e) {
                std::cout << "[LEGACY] FAILED: " << e.what() << "\n";
            }

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

                // Compare with legacy if it succeeded
                if (legacy_valid) {
                    const double new_duration = traj.duration().count();
                    const double duration_ratio = new_duration / legacy_duration;
                    const double duration_diff_percent = ((new_duration - legacy_duration) / legacy_duration) * 100.0;

                    std::cout << "[COMPARISON] new=" << new_duration << "s, legacy=" << legacy_duration << "s, "
                              << "ratio=" << duration_ratio << ", diff=" << duration_diff_percent << "%\n";

                    BOOST_TEST_MESSAGE("Duration comparison: new=" << new_duration << "s vs legacy=" << legacy_duration << "s (ratio="
                                                                   << duration_ratio << ", diff=" << duration_diff_percent << "%)");
                }

                BOOST_TEST_MESSAGE("PASS: Succeeded with " << num_waypoints << " waypoints");
            } catch (const std::exception& e) {
                // Log the failure and fail the test (but continue testing other waypoint counts)
                BOOST_TEST_MESSAGE("FAIL: " << num_waypoints << " waypoints: " << e.what());
                BOOST_CHECK_MESSAGE(false, "Failed to generate trajectory for " << num_waypoints << " waypoints: " << e.what());
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(three_waypoint_baseline_behavior_accel_constrained) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Create fixture
    trajectory_test_fixture fixture(6, 0.1);

    fixture.enable_legacy_comparison()
        .set_max_velocity(degrees_to_radians(xt::ones<double>({6}) * 50.0))
        .set_max_acceleration(degrees_to_radians(xt::ones<double>({6}) * 1.0))
        .set_max_deviation(0.1);

    fixture.traj_opts.delta = trajectory::seconds{0.0001};

    // Trajectory-wide expectations
    fixture.expect_duration(trajectory::seconds{20.162}).expect_integration_point_count(201621).expect_path_length(arc_length{1.8553});

    // Set waypoints
    fixture.set_waypoints_deg({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0}, {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0}});

    fixture.observer.expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.718023},
                          arc_velocity{0.18827},
                          arc_velocity{0.15297},  // acc_limit
                          arc_velocity{1.23413}   // vel_limit
                          )
        .expect_backward_start(arc_length{1.46262}, arc_velocity{0.12863}, trajectory::switching_point_kind::k_discontinuous_curvature)
        .expect_splice(trajectory::seconds{13.4162}, size_t{9195})
        .expect_forward_start(arc_length{1.46262}, arc_velocity{0.12863})
        .expect_backward_start(arc_length{1.85532488}, arc_velocity{0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{20.16199}, size_t{93043});

    const trajectory traj = fixture.create_and_validate();
}

BOOST_AUTO_TEST_CASE(three_waypoint_baseline_behavior_vel_constrained) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Create fixture
    trajectory_test_fixture fixture(6, 0.1);

    fixture.enable_legacy_comparison()
        .set_max_velocity(degrees_to_radians(xt::ones<double>({6}) * 1.0))
        .set_max_acceleration(degrees_to_radians(xt::ones<double>({6}) * 50.0))
        .set_max_deviation(0.1);

    fixture.traj_opts.delta = trajectory::seconds{0.0001};

    // Trajectory-wide expectations
    fixture.expect_duration(trajectory::seconds{90.02}).expect_integration_point_count(900201).expect_path_length(arc_length{1.8553});

    // Set waypoints
    fixture.set_waypoints_deg({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0}, {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0}});

    fixture.observer.expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_backward_start(arc_length{1.85532488}, arc_velocity{0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{90.02000000}, size_t{101});

    const trajectory traj = fixture.create_and_validate();
}

BOOST_AUTO_TEST_CASE(RSDK_12979_nondifferentiable_switching_point_requires_zero_acceleration) {
    // RSDK-12979: Test that non-differentiable switching points (where f'_i(s) = 0 for some joint
    // on a circular arc) are handled correctly by using zero acceleration during backward integration.
    //
    // Without the fix (commented out in trajectory.cpp:1537-1539), backward integration uses the
    // local s_ddot_min, which can be significantly negative when other joints have non-zero tangents.
    // This causes entry into the infeasible region and throws an exception.
    //
    // Strategy: Use very high velocity limits (unconstrained) and very low acceleration limits
    // (constrained) with curved paths to create acceleration-dominated dynamics.

    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_test_fixture fixture(2, 0.12);

    // Very high velocity limits (essentially unconstrained)
    // Very low acceleration limits (the actual constraint)
    fixture.enable_legacy_comparison()
        .set_max_velocity(xt::xarray<double>{100.0, 100.0})
        .set_max_acceleration(xt::xarray<double>{0.05, 0.05})
        .set_max_deviation(0.05);  // Enable curves

    fixture.traj_opts.delta = trajectory::seconds{0.002};

    // Waypoints that create a path with curves where joints have different tangent behavior
    fixture.set_waypoints_rad({{0.0, 0.0}, {0.6, 0.0}, {0.1, 0.6}});

    // Expectations - looking for acceleration-constrained pattern:
    // forward_start -> hit_limit -> backward_start (NDE) -> splice -> forward_start -> backward_start (endpoint) -> splice
    fixture.observer.expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit()
        .expect_backward_start(std::nullopt, std::nullopt, trajectory::switching_point_kind::k_discontinuous_curvature)
        .expect_splice()
        .expect_forward_start()
        .expect_hit_limit()
        .expect_backward_start(std::nullopt, std::nullopt, trajectory::switching_point_kind::k_nondifferentiable_extremum)
        .expect_splice()
        .expect_forward_start()
        .expect_backward_start(std::nullopt, std::nullopt, trajectory::switching_point_kind::k_path_end)
        .expect_splice();

    const trajectory traj = fixture.create_and_validate();
}

BOOST_AUTO_TEST_SUITE_END()

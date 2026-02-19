#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>

#include <json/json.h>

#include <boost/test/data/monomorphic.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#if defined(__has_include) && (__has_include(<xtensor/containers/xadapt.hpp>))
#include <xtensor/containers/xadapt.hpp>
#else
#include <xtensor/xadapt.hpp>
#endif

#include <Eigen/Core>

#include <viam/trajex/totg/json_serialization.hpp>
#include <viam/trajex/totg/observers.hpp>
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

constexpr bool k_log_met_expectations = false;

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
    std::shared_ptr<expectation_observer> expectation_observer_;
    composite_integration_observer composite_observer_;
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
    std::optional<double> legacy_generation_time_ms_;
    double legacy_path_tolerance_percent = 5.0;
    double legacy_duration_tolerance_percent = 5.0;

    // Event validation control
    bool allow_any_events_ = false;

    // JSON output for visualization
    std::optional<std::string> json_output_filename_;
    std::shared_ptr<trajectory_integration_event_collector> collector_;

    explicit trajectory_test_fixture(size_t dof = 6, double expectation_tolerance_percent = 0.1)
        : expectation_observer_(std::make_shared<expectation_observer>(expectation_tolerance_percent)), dof_(dof) {}

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

    trajectory_test_fixture& allow_any_events() {
        allow_any_events_ = true;
        return *this;
    }

    trajectory_test_fixture& set_legacy_path_tolerance(double tolerance_percent) {
        legacy_path_tolerance_percent = tolerance_percent;
        return *this;
    }

    trajectory_test_fixture& set_legacy_duration_tolerance(double tolerance_percent) {
        legacy_duration_tolerance_percent = tolerance_percent;
        return *this;
    }

    trajectory_test_fixture& enable_json_output(std::string filename) {
        json_output_filename_ = std::move(filename);
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

            // Create legacy path and trajectory with timing
            const auto legacy_start_time = std::chrono::high_resolution_clock::now();
            const Path legacy_path(legacy_waypoints, path_opts.max_blend_deviation());
            const Trajectory legacy_traj(legacy_path, legacy_max_vel, legacy_max_acc, traj_opts.delta.count());
            const auto legacy_end_time = std::chrono::high_resolution_clock::now();
            legacy_generation_time_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(legacy_end_time - legacy_start_time).count();

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

                // Path should be within configured tolerance
                BOOST_CHECK_CLOSE(new_path_length, *legacy_path_length_, legacy_path_tolerance_percent);
            }

            if (legacy_duration_.has_value()) {
                const double duration_ratio = new_duration / *legacy_duration_;
                const double duration_diff_percent = ((new_duration - *legacy_duration_) / *legacy_duration_) * 100.0;

                std::cout << "[COMPARISON] Duration: new=" << new_duration << "s, legacy=" << *legacy_duration_ << "s, "
                          << "ratio=" << duration_ratio << ", diff=" << duration_diff_percent << "%\n";

                // We should do NO WORSE than legacy: duration should be within configured tolerance
                const double duration_tolerance_multiplier = 1.0 + (legacy_duration_tolerance_percent / 100.0);
                BOOST_CHECK_LE(new_duration, *legacy_duration_ * duration_tolerance_multiplier);

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

        // Validate that expectations follow TOTG invariant (only if expectations were specified)
        if (!allow_any_events_) {
            // Must have at least 3 events: forward_start, backward_start from endpoint, splice
            const auto& expectations = expectation_observer_->get_expectations();
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
                    const double backward_tol =
                        backward->tolerance_percent.value_or(expectation_observer_->get_default_tolerance_percent());
                    BOOST_CHECK_CLOSE(static_cast<double>(*backward->s), static_cast<double>(p.length()), backward_tol);
                }

                // Last must be splice
                const auto* splice = std::get_if<expectation_observer::expected_splice>(&expectations.back());
                BOOST_REQUIRE_MESSAGE(splice != nullptr, "Last expectation must be splice (TOTG invariant)");
            }
        }

        // Set up observer chain - always use composite observer
        traj_opts.observer = &composite_observer_;

        // Add expectation observer if validating events
        if (!allow_any_events_) {
            composite_observer_.add_observer(expectation_observer_);
        }

        // Add collector if JSON output enabled
        if (json_output_filename_.has_value()) {
            collector_ = composite_observer_.add_observer(std::make_shared<trajectory_integration_event_collector>());
        }

        // Generate trajectory
        trajectory traj = trajectory::create(std::move(p), traj_opts);

        // Write JSON if enabled
        if (json_output_filename_.has_value() && collector_) {
            std::ofstream out(*json_output_filename_);
            write_trajectory_json(out, traj, *collector_);
            out.close();
            BOOST_TEST_MESSAGE("Wrote trajectory JSON to " << *json_output_filename_);
        }

        // Verify all expectations were met (only if expectations were specified)
        if (!allow_any_events_) {
            expectation_observer_->verify_all_expectations_met();
        }

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

trajectory create_velocity_switching_test_trajectory(const xt::xarray<double>& waypoints_rad,
                                                     const xt::xarray<double>& max_velocity,
                                                     const xt::xarray<double>& max_acceleration,
                                                     double max_deviation,
                                                     trajectory::seconds delta,
                                                     trajectory_integration_event_collector& collector) {
    path::options popt;
    popt.set_max_deviation(max_deviation);
    path p = path::create(waypoints_rad, popt);

    trajectory::options topt;
    topt.max_velocity = max_velocity;
    topt.max_acceleration = max_acceleration;
    topt.delta = delta;
    topt.observer = &collector;

    return trajectory::create(std::move(p), topt);
}

std::vector<trajectory::integration_observer::started_backward_event> get_backward_events_by_kind(
    const trajectory_integration_event_collector& collector, trajectory::switching_point_kind kind) {
    std::vector<trajectory::integration_observer::started_backward_event> events;
    for (const auto& ev : collector.events()) {
        if (const auto* backward = std::get_if<trajectory::integration_observer::started_backward_event>(&ev)) {
            if (backward->kind == kind) {
                events.push_back(*backward);
            }
        }
    }
    return events;
}

double estimate_eq40_delta(const trajectory& traj, arc_length s) {
    auto cursor = traj.path().create_cursor();

    const double path_length = static_cast<double>(traj.path().length());
    const double s_value = static_cast<double>(s);

    const double h = std::min(1e-3, std::max(1e-6, path_length * 1e-4));
    const double s_before = std::max(0.0, s_value - h);
    const double s_after = std::min(path_length, s_value + h);
    if (s_after <= s_before) {
        throw std::runtime_error{"estimate_eq40_delta: invalid finite-difference interval"};
    }

    cursor.seek(arc_length{s_before});
    const auto limits_before = traj.get_velocity_limits(cursor);

    cursor.seek(arc_length{s_after});
    const auto limits_after = traj.get_velocity_limits(cursor);

    const double curve_slope =
        (static_cast<double>(limits_after.s_dot_max_vel) - static_cast<double>(limits_before.s_dot_max_vel)) / (s_after - s_before);

    cursor.seek(s);
    const auto limits = traj.get_velocity_limits(cursor);
    if (static_cast<double>(limits.s_dot_max_vel) <= 0.0) {
        throw std::runtime_error{"estimate_eq40_delta: velocity limit is non-positive"};
    }

    const auto bounds = traj.get_acceleration_bounds(cursor, limits.s_dot_max_vel);
    const double trajectory_slope = static_cast<double>(bounds.s_ddot_min) / static_cast<double>(limits.s_dot_max_vel);
    return trajectory_slope - curve_slope;
}

// Helper to get UR arm waypoints with reversals for incremental testing
xt::xarray<double> get_ur_arm_waypoints_with_reversals_deg() {
    static const xt::xarray<double> waypoints_deg = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},      // 0: Start at zero
        {-45.0, -45.0, 0.0, 0.0, 0.0, 0.0},  // 1
        {-45.0, -90.0, 0.0, 0.0, 0.0, 0.0},  // 2
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
    return waypoints_deg;
}

// Constraint profile for parameterized trajectory testing
struct constraint_profile {
    std::string name;
    xt::xarray<double> max_velocity;
    xt::xarray<double> max_acceleration;
};

// Print operator for Boost.Test data framework
std::ostream& operator<<(std::ostream& os, const constraint_profile& profile) {
    return os << profile.name;
}

// Current constraint profiles for UR arm testing
std::vector<constraint_profile> get_ur_arm_constraint_profiles() {
    using namespace viam::trajex;
    return {
        {"all_accel_constrained",
         degrees_to_radians(xt::ones<double>({6}) * 150.0),  // 150 deg/s
         degrees_to_radians(xt::ones<double>({6}) * 5.0)},   // 5 deg/s^2

        {"all_vel_constrained",
         degrees_to_radians(xt::ones<double>({6}) * 5.0),     // 150 deg/s
         degrees_to_radians(xt::ones<double>({6}) * 150.0)},  // 5 deg/s^2
    };
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

BOOST_DATA_TEST_CASE(ur_arm_incremental_waypoints_with_reversals,
                     boost::unit_test::data::xrange(size_t{2}, get_ur_arm_waypoints_with_reversals_deg().shape(0) + 1) *
                         boost::unit_test::data::make([]() {
                             static auto data = get_ur_arm_constraint_profiles();
                             return boost::unit_test::data::make(data);
                         }()),
                     num_waypoints,
                     profile) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::totg::test;
    using namespace viam::trajex::types;

    std::cout << "\n=== Testing " << num_waypoints << " waypoints with " << profile.name << " ===\n";

    // NOTE: Using very relaxed tolerance (200%) due to known acceleration bound violations
    // in some integration points for trajectories with reversals (likely RSDK-12981).
    // Some violations are ~10x beyond normal bounds at specific integration points.
    //
    // The goal of the test is to drive this tolerance down across many profiles.
    trajectory_test_fixture fixture(6, 0.1);
    fixture.validation_tolerance_percent = 200.0;

    fixture.allow_any_events();

    // Only enable legacy comparison for up to 5 waypoints (legacy generator crashes on 6+)
    if (num_waypoints <= 5) {
        fixture.enable_legacy_comparison()
            .set_legacy_path_tolerance(10.0)       // Allow 10% path difference (known discrepancy)
            .set_legacy_duration_tolerance(10.0);  // Allow 10% duration difference
    }

    fixture.set_max_velocity(profile.max_velocity).set_max_acceleration(profile.max_acceleration).set_max_deviation(0.1);

    // Extract subset of waypoints for this iteration
    const xt::xarray<double>& full_waypoints_deg = get_ur_arm_waypoints_with_reversals_deg();
    const xt::xarray<double> subset_deg = xt::view(full_waypoints_deg, xt::range(0, num_waypoints), xt::all());
    fixture.set_waypoints_deg(subset_deg);

    // Create and validate (exploratory test - no specific event expectations)
    const auto start_time = std::chrono::high_resolution_clock::now();
    const trajectory traj = fixture.create_and_validate();
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto generation_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Log trajectory summary
    const auto& integration_points = traj.get_integration_points();
    const double path_length = integration_points.empty() ? 0.0 : static_cast<double>(integration_points.back().s);
    std::cout << "  Path length: " << path_length << " rad\n";
    std::cout << "  Duration: " << traj.duration().count() << " s\n";
    std::cout << "  Integration points: " << integration_points.size() << "\n";
    std::cout << "  Generation time: " << generation_time_ms << " ms\n";

    // Compare generation time with legacy if available
    if (fixture.legacy_generation_time_ms_.has_value()) {
        const double time_ratio = static_cast<double>(generation_time_ms) / *fixture.legacy_generation_time_ms_;
        const double time_diff_percent =
            ((static_cast<double>(generation_time_ms) - *fixture.legacy_generation_time_ms_) / *fixture.legacy_generation_time_ms_) * 100.0;

        std::cout << "[COMPARISON] Generation time: new=" << generation_time_ms << "ms, legacy=" << *fixture.legacy_generation_time_ms_
                  << "ms, "
                  << "ratio=" << time_ratio << ", diff=" << time_diff_percent << "%\n";
    }

    // Verify sampling works
    auto sampler = uniform_sampler::quantized_for_trajectory(traj, hertz{5.0});
    int sample_count = 0;
    for (const auto& s : traj.samples(sampler)) {
        BOOST_CHECK_EQUAL(s.configuration.shape(0), 6U);
        ++sample_count;
    }
    BOOST_CHECK(sample_count > 0);

    BOOST_TEST_MESSAGE("PASS: " << profile.name << " with " << num_waypoints << " waypoints, "
                                << "duration=" << traj.duration().count() << "s, "
                                << "points=" << traj.get_integration_points().size());
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

    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
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

    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
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
        .set_max_velocity(xt::xarray<double>{0.165, 0.167})
        .set_max_acceleration(xt::xarray<double>{0.05, 0.04})
        .set_max_deviation(0.05);  // Enable curves

    fixture.traj_opts.delta = trajectory::seconds{0.002};

    // Waypoints that create a path with curves where joints have different tangent behavior
    fixture.set_waypoints_rad({{0.0, 0.0}, {0.6, 0.0}, {0.1, 0.6}});

    // Expectations - looking for acceleration-constrained pattern:
    // forward_start -> hit_limit -> backward_start (NDE) -> splice -> forward_start -> backward_start (endpoint) -> splice
    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.521369}, arc_velocity{0.165}, arc_velocity{0.0383818}, arc_velocity{0.165})
        .expect_backward_start(arc_length{0.521369}, arc_velocity{0.0383818}, trajectory::switching_point_kind::k_discontinuous_curvature)
        .expect_splice(trajectory::seconds{5.78126}, size_t{769})
        .expect_forward_start(arc_length{0.521369}, arc_velocity{0.0383818})
        .expect_hit_limit(arc_length{0.566993}, arc_velocity{0.0471279}, arc_velocity{0.0471279}, arc_velocity{0.176646})
        .expect_backward_start(
            arc_length{0.579219}, arc_velocity{0.0429121}, trajectory::switching_point_kind::k_nondifferentiable_extremum)
        .expect_splice(trajectory::seconds{7.18515}, size_t{77})
        .expect_forward_start(arc_length{0.579219}, arc_velocity{0.0429121})
        .expect_backward_start(arc_length{1.3072}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{14.3426}, size_t{845});

    const trajectory traj = fixture.create_and_validate();
}

BOOST_AUTO_TEST_CASE(trajectory_json_serialization) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Create simple path with 3 waypoints
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {2.0, 0.0, 0.0}};

    path p = path::create(waypoints, path::options{});

    // Attach collector
    trajectory_integration_event_collector collector;
    trajectory::options opts;
    opts.max_velocity = xt::ones<double>({3}) * 1.0;
    opts.max_acceleration = xt::ones<double>({3}) * 1.0;
    opts.observer = &collector;

    // Generate trajectory
    const trajectory traj = trajectory::create(std::move(p), opts);

    // Serialize to JSON
    const std::string json = serialize_trajectory_to_json(traj, collector);

    // Validate JSON structure
    const Json::CharReaderBuilder reader;
    Json::Value root;
    std::istringstream ss(json);
    BOOST_REQUIRE(Json::parseFromStream(reader, ss, &root, nullptr));

    // Check metadata
    BOOST_REQUIRE(root.isMember("metadata"));
    BOOST_REQUIRE(root["metadata"]["dof"].asInt() == 3);
    BOOST_REQUIRE(root["metadata"].isMember("duration"));

    // Check integration_points structure
    BOOST_REQUIRE(root.isMember("integration_points"));
    BOOST_REQUIRE(root["integration_points"]["time"].isArray());
    BOOST_REQUIRE(root["integration_points"]["s"].isArray());

    // Check events structure
    BOOST_REQUIRE(root.isMember("events"));
    BOOST_REQUIRE(root["events"]["forward_starts"].isArray());
    BOOST_REQUIRE(root["events"]["backward_starts"].isArray());

    // Should have at least 1 forward start and 1 backward start
    BOOST_CHECK_GT(root["events"]["forward_starts"].size(), 0);
    BOOST_CHECK_GT(root["events"]["backward_starts"].size(), 0);
}

BOOST_AUTO_TEST_SUITE_END()

// =============================================================================
// Velocity switching point search tests
//
// These tests exercise the continuous and discontinuous velocity switching point
// detection (Kunz & Stilman equations 40-42). Cases 1-11 use high acceleration
// limits so the velocity curve is below the acceleration curve everywhere,
// making the trajectory velocity-dominated. Cases 12-15 use tight curvature
// (small deviation) where the centripetal term pushes the acceleration curve
// below the velocity curve in some regions, testing mixed regimes.
// =============================================================================

BOOST_AUTO_TEST_SUITE(velocity_switching_point_search)

// ---------------------------------------------------------------------------
// Case 1: Velocity curve drops sharply -> velocity escape switching point.
//
// Path turns from a direction where one joint has a high velocity limit to a
// direction where a slower joint becomes the constraint. The velocity curve
// drops, forcing the trajectory to leave the velocity curve.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(sharp_velocity_curve_drop_produces_velocity_escape) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // 2-DOF robot: joint 1 is fast, joint 2 is slow.
    // Path goes along joint 1 then turns 90 deg toward joint 2.
    // Velocity curve drops from ~v_max_1 to ~v_max_2 across the blend.
    // Tight blend so the trajectory hits the velocity curve before the blend,
    // then the curve drops sharply.
    trajectory_test_fixture fixture(2, 1.0);

    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.03);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    fixture.set_waypoints_rad({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});

    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.939658}, arc_velocity{0.4817}, arc_velocity{0.913506}, arc_velocity{0.481691})
        .expect_backward_start(arc_length{0.942656}, arc_velocity{0.386946}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{1.91158}, size_t{11})
        .expect_forward_start(arc_length{0.942656}, arc_velocity{0.386946})
        .expect_backward_start(arc_length{1.96891}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{14.3962}, size_t{5});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 2: Velocity curve rises steeply -> no velocity switching point.
//
// Path goes along the slow joint first, then turns toward the fast joint.
// The velocity curve rises, so the trajectory never needs to leave it due to
// velocity constraints. Only acceleration or path-end switching points occur.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(rising_velocity_curve_no_velocity_switching_point) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Opposite of Case 1: path goes along joint 2 (slow) then turns to joint 1 (fast).
    // Same limits as Case 1 but opposite path direction -- velocity curve rises instead of drops.
    trajectory_test_fixture fixture(2, 1.0);

    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.03);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    fixture.set_waypoints_rad({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}});

    // No velocity switching point expected -- velocity curve rises, not drops.
    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_backward_start(arc_length{1.96891}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{14.396}, size_t{26});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 3: Gradual velocity curve drop -> switching point found via bisection.
//
// Similar to Case 1 but with a wider blend (larger deviation), creating a more
// gradual velocity curve transition. Tests that bisection accurately locates
// the switching point when the sign change spans a wider arc.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(gradual_velocity_curve_drop_bisection_accuracy) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_test_fixture fixture(2, 1.0);

    // Moderate blend deviation -> wider blend -> more gradual velocity curve transition.
    // Same velocity limits as Case 1 but with a wider blend, so the velocity curve
    // drops more gradually across the blend region.
    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.05);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    fixture.set_waypoints_rad({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});

    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.899412}, arc_velocity{0.48216}, arc_velocity{1.17927}, arc_velocity{0.482137})
        .expect_backward_start(arc_length{0.900447}, arc_velocity{0.458772}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{1.82592}, size_t{5})
        .expect_forward_start(arc_length{0.900447}, arc_velocity{0.458772})
        .expect_backward_start(arc_length{1.94819}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{14.3069}, size_t{5});

    fixture.create_and_validate();

    trajectory_integration_event_collector collector;
    const trajectory observed_traj =
        create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}},
                                                  xt::xarray<double>{0.5, 0.08},
                                                  xt::xarray<double>{10.0, 10.0},
                                                  0.05,
                                                  trajectory::seconds{0.001},
                                                  collector);

    const auto velocity_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_REQUIRE_EQUAL(velocity_escapes.size(), 1U);

    const auto s_switch = velocity_escapes.front().start.s;
    const double coarse_step = 0.001;
    const double coarse_index = static_cast<double>(s_switch) / coarse_step;
    const double grid_residual = std::abs(coarse_index - std::round(coarse_index));

    // Coarse scan samples at multiples of delta. Off-grid switching point implies bisection refinement.
    BOOST_CHECK_GT(grid_residual, 1e-6);

    const auto sample_offset = arc_length{1e-4};
    const auto s_before = std::max(arc_length{0.0}, s_switch - sample_offset);
    const auto s_after = std::min(observed_traj.path().length(), s_switch + sample_offset);

    const double delta_before = estimate_eq40_delta(observed_traj, s_before);
    const double delta_after = estimate_eq40_delta(observed_traj, s_after);

    BOOST_CHECK_GT(delta_before, 0.0);
    BOOST_CHECK_LE(delta_after, 0.0);
}

// ---------------------------------------------------------------------------
// Case 4: Multiple velocity switching points -> finds the first.
//
// Path with multiple turns, each causing the velocity curve to drop. The
// search should find the earliest switching point.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(multiple_velocity_drops_finds_first_switching_point) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Zigzag path: joint 1 direction -> joint 2 direction -> joint 1 direction -> joint 2 direction.
    // Each turn from joint 1 to joint 2 creates a velocity drop.
    // Same limits as Case 1 which produces velocity_escape.
    trajectory_test_fixture fixture(2, 1.0);

    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.03);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    fixture.set_waypoints_rad({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}});

    // Two velocity escape switching points (one at each joint-1-to-joint-2 turn).
    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.939658}, arc_velocity{0.4817}, arc_velocity{0.913506}, arc_velocity{0.481691})
        .expect_backward_start(arc_length{0.942656}, arc_velocity{0.386946}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{1.91158}, size_t{11})
        .expect_forward_start(arc_length{0.942656}, arc_velocity{0.386946})
        .expect_hit_limit(arc_length{2.87749}, arc_velocity{0.4817}, arc_velocity{0.913506}, arc_velocity{0.481691})
        .expect_backward_start(arc_length{2.88048}, arc_velocity{0.386946}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{16.1457}, size_t{11})
        .expect_forward_start(arc_length{2.88048}, arc_velocity{0.386946})
        .expect_backward_start(arc_length{3.90674}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{28.6304}, size_t{5});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 5: Multiple velocity escapes on a multi-turn path with wider blends.
//
// Zigzag path with three 90 deg turns at moderate deviation. Each turn from the
// fast joint toward the slow joint creates a velocity drop. Tests that the
// search correctly finds velocity escapes across multiple blend regions with
// wider arcs than Cases 1/3.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(multiple_velocity_escapes_wider_blends) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_test_fixture fixture(2, 1.0);

    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.05);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    // Three turns with moderate blend radius.
    fixture.set_waypoints_rad({{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}, {1.0, 0.5}, {1.0, 1.0}});

    // Two velocity escape switching points (one at each joint-1-to-joint-2 turn).
    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.399412}, arc_velocity{0.48216}, arc_velocity{1.17927}, arc_velocity{0.482137})
        .expect_backward_start(arc_length{0.400447}, arc_velocity{0.458772}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{0.825918}, size_t{5})
        .expect_forward_start(arc_length{0.400447}, arc_velocity{0.458772})
        .expect_hit_limit(arc_length{1.29579}, arc_velocity{0.48216}, arc_velocity{1.17927}, arc_velocity{0.482137})
        .expect_backward_start(arc_length{1.29683}, arc_velocity{0.458772}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{7.63155}, size_t{5})
        .expect_forward_start(arc_length{1.29683}, arc_velocity{0.458772})
        .expect_backward_start(arc_length{1.84457}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{13.8625}, size_t{5});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 6: Constant velocity curve -> no velocity switching point.
//
// Straight-line path with uniform velocity limits. The velocity curve is flat,
// so there is no sign change in Delta(s). Only path-end switching point occurs.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(constant_velocity_curve_no_switching_point) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Straight line -- no blends, constant q', constant velocity limit.
    trajectory_test_fixture fixture(2, 1.0);

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.5}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.1);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    // Diagonal straight line: both joints move equally, q' = (1/sqrt(2), 1/sqrt(2)).
    fixture.set_waypoints_rad({{0.0, 0.0}, {1.0, 1.0}});

    // No velocity switching point -- constant velocity curve on a straight line.
    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_backward_start(arc_length{1.41421}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{2.05002}, size_t{26});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 7: Velocity switching point early in the path.
//
// The trajectory hits the velocity curve quickly and the curve drops almost
// immediately, testing that the sign change detection works near the start
// of the coarse search.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(velocity_switching_point_near_path_start) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // Short first segment so the trajectory hits the velocity curve quickly,
    // then an immediate turn to a slower-constrained direction. High acceleration
    // ensures the trajectory reaches the velocity curve on the short first segment.
    trajectory_test_fixture fixture(2, 1.0);

    fixture.validation_tolerance_percent = 0.5;

    fixture.set_max_velocity(xt::xarray<double>{0.5, 0.08}).set_max_acceleration(xt::xarray<double>{10.0, 10.0}).set_max_deviation(0.03);

    fixture.traj_opts.delta = trajectory::seconds{0.001};

    // Moderate first segment (0.5 rad), then 90 deg turn.
    fixture.set_waypoints_rad({{0.0, 0.0}, {0.5, 0.0}, {0.5, 1.0}});

    fixture.expectation_observer_->expect_forward_start(arc_length{0.0}, arc_velocity{0.0})
        .expect_hit_limit(arc_length{0.439658}, arc_velocity{0.4817}, arc_velocity{0.913506}, arc_velocity{0.481691})
        .expect_backward_start(arc_length{0.442656}, arc_velocity{0.386946}, trajectory::switching_point_kind::k_velocity_escape)
        .expect_splice(trajectory::seconds{0.911583}, size_t{11})
        .expect_forward_start(arc_length{0.442656}, arc_velocity{0.386946})
        .expect_backward_start(arc_length{1.46891}, arc_velocity{0.0}, trajectory::switching_point_kind::k_path_end)
        .expect_splice(trajectory::seconds{13.3962}, size_t{5});

    const trajectory traj = fixture.create_and_validate();
}

// ---------------------------------------------------------------------------
// Case 8: Eq. 40 sign bracket at a velocity escape switching point.
//
// For a velocity escape switching point, Delta(s) = s_ddot_min/s_dot_max_vel - d/ds(s_dot_max_vel)
// should transition from positive (trapped) to non-positive (escape possible).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(eq40_sign_change_brackets_velocity_escape_switching_point) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    const trajectory traj = create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}},
                                                                      xt::xarray<double>{0.5, 0.08},
                                                                      xt::xarray<double>{10.0, 10.0},
                                                                      0.03,
                                                                      trajectory::seconds{0.001},
                                                                      collector);

    validate_trajectory_invariants(traj, 0.5);

    const auto velocity_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_REQUIRE_EQUAL(velocity_escapes.size(), 1U);

    const auto s_switch = velocity_escapes.front().start.s;
    const auto sample_offset = arc_length{1e-4};
    const auto s_before = std::max(arc_length{0.0}, s_switch - sample_offset);
    const auto s_after = std::min(traj.path().length(), s_switch + sample_offset);

    const double delta_before = estimate_eq40_delta(traj, s_before);
    const double delta_after = estimate_eq40_delta(traj, s_after);

    BOOST_CHECK_GT(delta_before, 0.0);
    BOOST_CHECK_LE(delta_after, 0.0);
}

// ---------------------------------------------------------------------------
// Case 9: Rising velocity curve has no Eq. 40 escape transition.
//
// In this geometry the velocity curve rises, so there should be no Delta > 0 to Delta <= 0
// transition and no velocity-escape switching point.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(rising_velocity_curve_has_no_eq40_escape_transition) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    const trajectory traj = create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}},
                                                                      xt::xarray<double>{0.5, 0.08},
                                                                      xt::xarray<double>{10.0, 10.0},
                                                                      0.03,
                                                                      trajectory::seconds{0.001},
                                                                      collector);

    validate_trajectory_invariants(traj, 0.5);

    const auto velocity_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_CHECK(velocity_escapes.empty());

    size_t eq40_crossings = 0;
    std::optional<double> previous_delta;
    auto cursor = traj.path().create_cursor();
    const auto path_length = static_cast<double>(traj.path().length());

    for (size_t i = 1; i < 1000; ++i) {
        const auto s = arc_length{path_length * (static_cast<double>(i) / 1000.0)};
        cursor.seek(s);

        const auto limits = traj.get_velocity_limits(cursor);
        if (static_cast<double>(limits.s_dot_max_vel) <= 0.0 ||
            static_cast<double>(limits.s_dot_max_acc) < static_cast<double>(limits.s_dot_max_vel)) {
            previous_delta.reset();
            continue;
        }

        const double delta = estimate_eq40_delta(traj, s);
        if (previous_delta.has_value() && (*previous_delta > 0.0) && (delta <= 0.0)) {
            ++eq40_crossings;
        }
        previous_delta = delta;
    }

    BOOST_CHECK_EQUAL(eq40_crossings, 0U);
}

// ---------------------------------------------------------------------------
// Case 10: Multiple velocity escapes are ordered and feasible.
//
// Verifies that the velocity-switching search finds multiple escape points in
// increasing arc-length order and each accepted switching point is feasible
// against the acceleration-limited velocity curve.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(multiple_velocity_escapes_are_ordered_and_feasible) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    const trajectory traj =
        create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}},
                                                  xt::xarray<double>{0.5, 0.08},
                                                  xt::xarray<double>{10.0, 10.0},
                                                  0.03,
                                                  trajectory::seconds{0.001},
                                                  collector);

    validate_trajectory_invariants(traj, 0.5);

    const auto velocity_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_REQUIRE_EQUAL(velocity_escapes.size(), 2U);
    BOOST_CHECK_LT(velocity_escapes[0].start.s, velocity_escapes[1].start.s);

    auto cursor = traj.path().create_cursor();
    for (const auto& escape : velocity_escapes) {
        cursor.seek(escape.start.s);
        const auto limits = traj.get_velocity_limits(cursor);

        BOOST_CHECK_LE(static_cast<double>(limits.s_dot_max_vel), static_cast<double>(limits.s_dot_max_acc) + 1e-9);
        BOOST_CHECK_CLOSE(static_cast<double>(escape.start.s_dot), static_cast<double>(limits.s_dot_max_vel), 0.5);
    }
}

// ---------------------------------------------------------------------------
// Case 11: Near-start velocity escape still exhibits Eq. 40 sign bracket.
//
// Same Eq. 40 sign-bracket assertion as Case 8, but for a switching point that
// occurs early in the path.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(near_start_velocity_escape_has_eq40_sign_bracket) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    const trajectory traj = create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {0.5, 0.0}, {0.5, 1.0}},
                                                                      xt::xarray<double>{0.5, 0.08},
                                                                      xt::xarray<double>{10.0, 10.0},
                                                                      0.03,
                                                                      trajectory::seconds{0.001},
                                                                      collector);

    validate_trajectory_invariants(traj, 0.5);

    const auto velocity_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_REQUIRE_GE(velocity_escapes.size(), 1U);

    const auto s_switch = velocity_escapes.front().start.s;
    BOOST_CHECK_LT(s_switch, arc_length{1.0});

    const auto sample_offset = arc_length{1e-4};
    const auto s_before = std::max(arc_length{0.0}, s_switch - sample_offset);
    const auto s_after = std::min(traj.path().length(), s_switch + sample_offset);

    const double delta_before = estimate_eq40_delta(traj, s_before);
    const double delta_after = estimate_eq40_delta(traj, s_after);

    BOOST_CHECK_GT(delta_before, 0.0);
    BOOST_CHECK_LE(delta_after, 0.0);
}

// ---------------------------------------------------------------------------
// Case 12: Boundary produces k_discontinuous_velocity_limit.
//
// Extreme velocity ratio with a tiny blend so the velocity curve drops
// discontinuously at the segment boundary. High acceleration limits ensure
// feasibility (s_dot_max_acc >= s_dot_max_vel at the boundary).
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(boundary_produces_discontinuous_velocity_limit) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // 30 deg turn: the velocity curve has a steep slope at the blend-end boundary.
    // With moderate curvature (kappa ~= 50), centripetal acceleration makes trajectory_slope
    // less negative than curve_slope (condition 41 holds), while the gentler curvature
    // avoids backward-integration failure.
    trajectory_integration_event_collector collector;
    const trajectory traj =
        create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.866, 0.5}},
                                                  xt::xarray<double>{0.5, 0.1},  // 5:1 ratio
                                                  xt::xarray<double>{3.0, 3.0},  // accel tuned so c41 holds and trajectory constructs
                                                  0.0007,                        // kappa ~= 49 for 30 deg turn
                                                  trajectory::seconds{0.001},    // normal step
                                                  collector);

    // Relaxed tolerance: the geometry that triggers eqs 41-42 requires tight curvature
    // which pushes TOTG near its numerical limits.
    // TODO: tighten once TOTG handles high-curvature blends better.
    validate_trajectory_invariants(traj, 50.0);

    const auto disc = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_discontinuous_velocity_limit);
    BOOST_REQUIRE_GE(disc.size(), 1U);

    // Verify the switching point is at the blend-end boundary (around s ~= 1.0)
    BOOST_CHECK_CLOSE(static_cast<double>(disc.front().start.s), 1.0, 5.0);
}

// ---------------------------------------------------------------------------
// Case 13: Both discontinuous velocity limit and continuous velocity escape
// exist on the same trajectory.
//
// Extends the Case 12 geometry with a second turn that produces a velocity
// escape. Verifies Phase 4's select_switching_point correctly picks between
// both types based on arc-length ordering.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(both_discontinuous_velocity_limit_and_velocity_escape) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    // First turn (30 deg) triggers discontinuous velocity limit (same as Case 12).
    // Second turn (90 deg, fast->slow) triggers continuous velocity escape.
    trajectory_integration_event_collector collector;
    const trajectory traj =
        create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.866, 0.5}, {1.866, 1.5}},
                                                  xt::xarray<double>{0.5, 0.1},
                                                  xt::xarray<double>{3.0, 3.0},
                                                  0.0007,
                                                  trajectory::seconds{0.001},
                                                  collector);

    // TODO: tighten once TOTG handles high-curvature blends better.
    validate_trajectory_invariants(traj, 50.0);

    const auto disc = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_discontinuous_velocity_limit);
    const auto vel = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);

    // Both types must be present.
    BOOST_TEST_MESSAGE("Discontinuous: " << disc.size() << ", Velocity escapes: " << vel.size());
    BOOST_CHECK_GE(disc.size(), 1U);
    BOOST_CHECK_GE(vel.size(), 1U);
}

// ---------------------------------------------------------------------------
// Case 14: Multi-turn path with low acceleration -- stress test for switching
// point search (RSDK-12980 regression coverage).
//
// Uses two 90-degree turns with low acceleration, creating multiple regions where
// the switching point search must evaluate and potentially reject brackets
// before finding feasible velocity escapes.  The RSDK-12980 fix ensures that
// a rejected coarse bracket does not terminate the search prematurely.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(multi_turn_low_accel_switching_point_search) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    // Two 90-degree turns with low acceleration and asymmetric velocity limits.
    const trajectory traj =
        create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}, {1.0, 0.5}, {1.0, 1.0}},
                                                  xt::xarray<double>{0.5, 0.08},  // asymmetric velocity
                                                  xt::xarray<double>{2.0, 2.0},   // low-ish acceleration
                                                  0.05,                           // moderate deviation
                                                  trajectory::seconds{0.001},
                                                  collector);

    validate_trajectory_invariants(traj, 5.0);

    // The search must find velocity escapes at the turns.
    const auto vel_escapes = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_velocity_escape);
    BOOST_CHECK_GE(vel_escapes.size(), 1U);
    BOOST_CHECK_GT(traj.duration().count(), 0.0);
}

// ---------------------------------------------------------------------------
// Case 15: Condition 41 false gates a boundary that would otherwise qualify.
//
// Same 30 deg geometry as Case 12, but with a larger blend deviation (0.005
// vs 0.0007).  The larger deviation reduces curvature at the blend, which
// makes curve_slope_before much shallower.  trajectory_slope_before ends up
// MORE negative than curve_slope_before, so condition 41
// (trajectory_slope_before >= curve_slope_before) is false.  Condition 42
// remains true (linear after-segment -> curve_slope_after=0, and
// trajectory_slope_after < 0).
//
// This verifies that a single false condition gates the boundary decision,
// preventing a switching point that Case 12 (with tighter curvature) does
// produce.
//
// Note: condition 42 false is structurally unreachable at integration level
// with linear/circular segments because blend-end boundaries always have
// curve_slope_after = 0 (linear after-segment, q''=0) and
// trajectory_slope_after < 0 (minimum deceleration), making c42 trivially
// true.  Blend-start boundaries are filtered before reaching the
// c41/c42 evaluation because s_dot_max_accel < s_dot_max_vel there.
// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(condition_41_false_gates_boundary) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    trajectory_integration_event_collector collector;
    // Same waypoints, velocities, accelerations as Case 12, but deviation=0.005
    // instead of 0.0007.  The larger blend radius reduces curvature enough that
    // condition 41 flips to false.
    const trajectory traj = create_velocity_switching_test_trajectory(xt::xarray<double>{{0.0, 0.0}, {1.0, 0.0}, {1.866, 0.5}},
                                                                      xt::xarray<double>{0.5, 0.1},  // 5:1 ratio (same as Case 12)
                                                                      xt::xarray<double>{3.0, 3.0},  // same accel
                                                                      0.005,                         // larger deviation -> lower curvature
                                                                      trajectory::seconds{0.001},
                                                                      collector);

    validate_trajectory_invariants(traj, 50.0);

    // With condition 41 false, no discontinuous velocity switching point should be produced.
    const auto disc = get_backward_events_by_kind(collector, trajectory::switching_point_kind::k_discontinuous_velocity_limit);
    BOOST_CHECK_EQUAL(disc.size(), 0U);
}

BOOST_AUTO_TEST_SUITE_END()

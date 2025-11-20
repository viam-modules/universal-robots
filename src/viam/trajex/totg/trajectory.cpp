#include <viam/trajex/totg/trajectory.hpp>

#include <cassert>
#include <functional>
#include <numbers>
#include <optional>
#include <ranges>
#include <stdexcept>

#include <boost/range/adaptors.hpp>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_length.hpp>

namespace viam::trajex::totg {

namespace {

// State machine states for TOTG integration algorithm.
// Internal implementation detail - not exposed in public API.
enum class integration_state : std::uint8_t {
    k_forward_accelerating,              // Forward integration with maximum acceleration
    k_forward_following_velocity_curve,  // Following velocity limit curve tangentially
    k_backward_decelerating              // Backward integration with minimum acceleration
};

// Events that trigger state transitions during trajectory integration.
// These are internal implementation details - external code observes via integration_observer callbacks.
enum class integration_event : std::uint8_t {
    k_none,                    // No event (remain in current state)
    k_hit_velocity_curve,      // Hit velocity limit curve (can potentially follow tangentially)
    k_hit_acceleration_curve,  // Hit acceleration limit curve (must search for switching point)
    k_escaped_limit_curve,     // Dropped below limit curve (can resume normal acceleration)
    k_reached_end,             // Reached end of path
    k_reached_start,           // Reached start of path (backward integration)
    k_hit_forward_trajectory   // Backward trajectory intersected forward trajectory
};

// Computes maximum path velocity (s_dot) from joint velocity and acceleration limits.
// Two constraints apply: centripetal acceleration from path curvature (eq 31) and
// direct velocity limits (eq 36). Returns both so caller can take the minimum.
// See Kunz & Stilman equations 31 and 36.
[[gnu::pure]] auto compute_velocity_limits(const xt::xarray<double>& q_prime,
                                           const xt::xarray<double>& q_double_prime,
                                           const xt::xarray<double>& q_dot_max,
                                           const xt::xarray<double>& q_ddot_max,
                                           double epsilon) {
    struct result {
        double s_dot_max_acc;
        double s_dot_max_vel;
    };

    // Compute the path velocity limit imposed by joint acceleration constraints (equation 31).
    // This is the acceleration limit curve in the phase plane. The derivation in the paper
    // converts joint acceleration bounds into constraints on path velocity by considering
    // the centripetal acceleration term q''(s)*s_dot^2 that appears when following a curved
    // path. The result is a set of downward-facing parabolas centered at the origin, and we
    // take the minimum of their positive bounds to find the feasible path velocity.
    double s_dot_max_accel = std::numeric_limits<double>::infinity();

    // For each pair of joints that are moving along the path, compare their curvature ratios
    // q''(s)/q'(s). When these ratios differ, the joints are curving at different rates, which
    // limits how fast we can traverse the path. This is the pairwise constraint from equation 31.
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        for (size_t j = i + 1; j < q_prime.size(); ++j) {
            if (std::abs(q_prime(j)) < epsilon) {
                continue;
            }

            const double curvature_ratio_i = q_double_prime(i) / q_prime(i);
            const double curvature_ratio_j = q_double_prime(j) / q_prime(j);
            const double curvature_difference = std::abs(curvature_ratio_i - curvature_ratio_j);

            if (curvature_difference < epsilon) {
                continue;
            }

            const double accel_sum = (q_ddot_max(i) / std::abs(q_prime(i))) + (q_ddot_max(j) / std::abs(q_prime(j)));
            const double limit = std::sqrt(accel_sum / curvature_difference);
            s_dot_max_accel = std::min(s_dot_max_accel, limit);
        }
    }

    // A joint that is stationary in path space (q'(s) = 0) but has non-zero curvature (q''(s) != 0)
    // also constrains the path velocity. From equations 19-20, when q'(s) = 0, the constraint
    // q''(s)*s_dot^2 <= q_ddot_max directly limits s_dot. This case appears at points where a joint
    // reaches a local extremum along the path while the path itself is curved.
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) >= epsilon) {
            continue;
        }

        if (std::abs(q_double_prime(i)) < epsilon) {
            continue;
        }

        const double limit = std::sqrt(q_ddot_max(i) / std::abs(q_double_prime(i)));
        s_dot_max_accel = std::min(s_dot_max_accel, limit);
    }

    // Compute the path velocity limit imposed by joint velocity constraints (equation 36).
    // This is the velocity limit curve in the phase plane. For each joint moving along
    // the path, the joint velocity q_dot = q'(s)*s_dot must respect the joint velocity
    // limit, giving us s_dot <= q_dot_max / |q'(s)|. We take the minimum across all joints.
    double s_dot_max_vel = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        const double limit = q_dot_max(i) / std::abs(q_prime(i));
        s_dot_max_vel = std::min(s_dot_max_vel, limit);
    }

    return result{s_dot_max_accel, s_dot_max_vel};
}

// Computes the feasible range of path acceleration (s_ddot) given current path velocity (s_dot)
// and joint acceleration limits. The path acceleration must satisfy joint constraints in all DOF.
// Using chain rule q''(t) = q'(s)*s_ddot + q''(s)*s_dot^2, we solve for s_ddot bounds.
// See Kunz & Stilman equations 22-23.
[[gnu::pure]] auto compute_acceleration_bounds(const xt::xarray<double>& q_prime,
                                               const xt::xarray<double>& q_double_prime,
                                               double s_dot,
                                               const xt::xarray<double>& q_ddot_max,
                                               double epsilon) {
    struct result {
        double s_ddot_min;
        double s_ddot_max;
    };

    double s_ddot_min = -std::numeric_limits<double>::infinity();
    double s_ddot_max = std::numeric_limits<double>::infinity();

    // Each joint independently constrains the feasible range of path acceleration (equations 22-23).
    // From the chain rule q''(t) = q'(s)*s_ddot + q''(s)*s_dot^2, we can solve for s_ddot given
    // the constraint -q_ddot_max <= q''(t) <= q_ddot_max. The centripetal term q''(s)*s_dot^2 is
    // already "using up" part of the acceleration budget at the current path velocity, which tightens
    // the bounds on how much path acceleration we can apply. Each joint shrinks the feasible region,
    // so we take the intersection by using max for lower bounds and min for upper bounds.
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        const double centripetal_term = q_double_prime(i) * s_dot * s_dot;

        // Per equations 22-23: acceleration limit terms use |q'(s)|, centripetal term uses signed q'(s)
        const double min_from_joint = ((-q_ddot_max(i)) / std::abs(q_prime(i))) - (centripetal_term / q_prime(i));
        s_ddot_min = std::max(s_ddot_min, min_from_joint);

        const double max_from_joint = (q_ddot_max(i) / std::abs(q_prime(i))) - (centripetal_term / q_prime(i));
        s_ddot_max = std::min(s_ddot_max, max_from_joint);
    }

    return result{s_ddot_min, s_ddot_max};
}

// Computes the derivative of the velocity limit curve in the phase plane.
// This is d/ds s_dot_max_vel(s), which tells us the slope of the velocity limit curve.
// Used in Algorithm Step 3 to determine if we can leave the curve or must search for switching points.
// See Kunz & Stilman equation 37.
[[gnu::pure]] double compute_velocity_limit_derivative(const xt::xarray<double>& q_prime,
                                                       const xt::xarray<double>& q_double_prime,
                                                       const xt::xarray<double>& q_dot_max,
                                                       double epsilon) {
    // Find which joint is the limiting constraint (has minimum q_dot_max / |q'|)
    double min_limit = std::numeric_limits<double>::infinity();
    size_t limiting_joint = 0;

    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        const double limit = q_dot_max(i) / std::abs(q_prime(i));
        if (limit < min_limit) {
            min_limit = limit;
            limiting_joint = i;
        }
    }

    // Compute derivative for the limiting joint (equation 37)
    // d/ds s_dot_max_vel = -(q_dot_max_i * q''_i) / (q'_i * |q'_i|)
    const double numerator = -q_dot_max(limiting_joint) * q_double_prime(limiting_joint);
    const double denominator = q_prime(limiting_joint) * std::abs(q_prime(limiting_joint));

    // TODO: This check can trigger even when limiting_joint was validly selected, because a joint
    // with |q'| slightly above epsilon (e.g., 1.1*epsilon) produces a denominator of |q'|^2 which
    // can be below epsilon (e.g., 1.21*epsilon^2 << epsilon for small epsilon). This represents a
    // numerically singular case where the joint is barely moving along the path (q' ≈ 0) but happens
    // to be the limiting constraint. The derivative d/ds(1/|q'|) becomes huge and ill-defined.
    //
    // Physical interpretation: When |q'| ≈ 0, the velocity limit s_dot_max = q_dot_max / |q'| is
    // enormous, meaning this joint isn't really constraining velocity in any meaningful sense. The
    // fact that it's the "limiting joint" may be an artifact rather than a real constraint.
    //
    // This needs investigation:
    // 1. Check reference implementation (src/third_party/trajectories/) for how it handles this case
    // 2. Consider whether limiting joint selection should filter out near-singular joints
    // 3. Determine correct behavior: throw, return infinity with sign, or use different epsilon threshold
    // 4. May need tighter check: |denominator| < epsilon^2 to match the q' filter tolerance
    //
    // For now, throw to make this case visible during testing rather than silently returning an
    // incorrect value that could cause incorrect curve-following behavior.
    if (std::abs(denominator) < epsilon) {
        throw std::runtime_error{
            "compute_velocity_limit_derivative: denominator near zero for limiting joint - "
            "velocity limit curve derivative is numerically undefined (joint barely moving along path)"};
    }

    return numerator / denominator;
}

// Performs a single Euler integration step in phase plane (s, s_dot).
// Given current position, velocity, and applied acceleration, computes the next state.
// Uses constant acceleration kinematic equations: v_new = v + a*dt, s_new = s + v*dt + 0.5*a*dt^2.
// This is direction-agnostic - caller determines whether dt is positive (forward) or negative (backward).
[[gnu::const]] auto euler_step(arc_length s, double s_dot, double s_ddot, double dt) {
    struct result {
        arc_length s;
        double s_dot;
    };

    const double s_dot_new = s_dot + (s_ddot * dt);
    const arc_length s_new = s + arc_length{(s_dot * dt) + (0.5 * s_ddot * dt * dt)};

    return result{s_new, s_dot_new};
}

// Validates if a segment boundary is a valid acceleration switching point per equation 38.
// A boundary is valid if the phase trajectory slopes on both sides bracket the acceleration
// limit curve slope change across the discontinuity, and the switching velocity is within
// the velocity limit.
// Searches for an acceleration switching point along segment boundaries.
//
// When forward integration hits the acceleration limit curve, we must find a point where
// backward integration can begin. This occurs at curvature discontinuities (segment boundaries)
// where the acceleration limit curve has a discontinuous change.
//
// TODO: This implementation is incomplete:
// We don't incorporate source/sink filtering to validate that limit curve intersections are
// trajectory sinks (valid stopping points) rather than sources (numerical artifacts from
// overshooting with finite dt). See paper Section VIII-A and reference Trajectory.cpp:152-191.
//
// At each segment boundary, we:
// 1. Sample geometry from both adjacent segments at the exact boundary arc length
// 2. Compute acceleration limit curve on both sides to find the switching velocity (minimum of the two)
// 3. Validate that this boundary satisfies the switching point conditions
//
// Returns the first valid switching point found, or nullopt if none exists before path end.
//
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] trajectory::phase_point find_acceleration_switching_point(path::cursor cursor, const trajectory::options& opt) {
    // Walk forward through segments, checking interior extrema first, then boundaries
    while (true) {
        auto current_segment = *cursor;

        // Check 1: Interior extrema in circular segments (continuous-nondifferentiable case).
        // Within a circular blend, a joint may reach a local extremum where f'_i(s) = 0
        // while the segment continues curving. This creates a continuous-nondifferentiable
        // point in the acceleration limit curve. Reference: Section VII-A case 2, equation 39.
        if (current_segment.is<path::segment::circular>()) {
            std::optional<arc_length> first_extremum;
            double first_extremum_velocity = 0.0;

            // Visit segment to access circular blend parameters
            current_segment.visit([&](const auto& seg_data) {
                using segment_type = std::decay_t<decltype(seg_data)>;
                if constexpr (std::is_same_v<segment_type, path::segment::circular>) {
                    const auto& x = seg_data.x;
                    const auto& y = seg_data.y;
                    const double radius = seg_data.radius;
                    const double total_angle = seg_data.angle_rads;

                    // When a joint reaches a local extremum within a circular arc (f'_i(s) = 0 while path
                    // continues curving), the acceleration limit curve becomes continuous but non-differentiable,
                    // creating a potential switching point (equation 39, Section VII-A case 2).
                    //
                    // For circular parameterization f(theta) = c + r*(x*cos(theta) + y*sin(theta)), the tangent
                    // f'_i(theta) = -x_i*sin(theta) + y_i*cos(theta) is zero when tan(theta) = y_i/x_i.
                    // Since tan has period pi (not 2*pi), both theta = atan2(y_i, x_i) and theta + pi are
                    // solutions, giving up to 2*n candidate extrema per circular segment (n = DOF).
                    for (size_t i = 0; i < x.size(); ++i) {
                        double extremum_angle = std::atan2(y(i), x(i));
                        if (extremum_angle < 0.0) {
                            extremum_angle += 2.0 * std::numbers::pi;
                        }

                        for (const auto angle : {extremum_angle, extremum_angle + std::numbers::pi}) {
                            if (angle < 0.0 || angle >= total_angle) {
                                continue;
                            }

                            const double s_local = radius * angle;
                            const arc_length s_global = current_segment.start() + arc_length{s_local};

                            // Strict < avoids re-checking if cursor is exactly at this extremum
                            if (s_global < cursor.position() || s_global >= current_segment.end()) {
                                continue;
                            }

                            if (!first_extremum.has_value() || s_global < *first_extremum) {
                                first_extremum = s_global;
                            }
                        }
                    }

                    // If we found an extremum, validate it as a switching point
                    if (first_extremum.has_value()) {
                        // Query geometry at the extremum
                        const auto q_prime = current_segment.tangent(*first_extremum);
                        const auto q_double_prime = current_segment.curvature(*first_extremum);

                        // Compute acceleration limit curve at extremum
                        const auto [s_dot_max_acc, s_dot_max_vel] =
                            compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

                        // Validate: acceleration limit curve must have local minimum (derivative changes negative to positive)
                        // Sample slightly before and after the extremum, clamped to segment bounds
                        //
                        // TODO: Replace numerical derivative check with analytical derivative computation.
                        // For circular segments, we can derive the exact formula for d/ds s_dot_max_acc(s) from
                        // equations 29-31 in the paper. The current epsilon-offset approach works but is less
                        // accurate and has epsilon dependency. An analytical solution would be more robust and
                        // faster (no need for 4 extra geometry queries + limit calculations per extremum).
                        const arc_length before_extremum = std::max(*first_extremum - arc_length{opt.epsilon}, current_segment.start());
                        const arc_length after_extremum = std::min(*first_extremum + arc_length{opt.epsilon}, current_segment.end());

                        // Skip validation if extremum is too close to segment boundaries (can't get clean epsilon separation)
                        const bool too_close_to_start = (*first_extremum - before_extremum) < arc_length{opt.epsilon * 0.5};
                        const bool too_close_to_end = (after_extremum - *first_extremum) < arc_length{opt.epsilon * 0.5};

                        if (too_close_to_start || too_close_to_end) {
                            first_extremum.reset();
                        } else {
                            const auto q_prime_before = current_segment.tangent(before_extremum);
                            const auto q_double_prime_before = current_segment.curvature(before_extremum);
                            const auto [s_dot_before, _1] = compute_velocity_limits(
                                q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

                            const auto q_prime_after = current_segment.tangent(after_extremum);
                            const auto q_double_prime_after = current_segment.curvature(after_extremum);
                            const auto [s_dot_after, _2] = compute_velocity_limits(
                                q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

                            // Check if curve has local minimum (decreasing before, increasing after)
                            const bool is_local_minimum = (s_dot_before > s_dot_max_acc) && (s_dot_after > s_dot_max_acc);

                            if (is_local_minimum) {
                                // Valid continuous-nondifferentiable switching point (Section VII-A case 2).
                                // Backward integration starts from this point with zero acceleration, since applying
                                // any non-zero acceleration would move off the non-differentiable cusp. The switching
                                // velocity is the value of the limit curve at this point.
                                first_extremum_velocity = s_dot_max_acc;
                            } else {
                                first_extremum.reset();
                            }
                        }
                    }
                }
            });

            // If we found a valid extremum switching point, return it
            if (first_extremum.has_value()) {
                return trajectory::phase_point{.s = *first_extremum, .s_dot = first_extremum_velocity};
            }
        }

        // Check 2: Segment boundary (discontinuous case)
        const arc_length boundary = current_segment.end();

        // If this boundary is at the path end, no more interior boundaries to check
        if (boundary >= cursor.path().length()) {
            break;
        }

        // Sample geometry from segment ending at boundary
        const auto q_prime_before = current_segment.tangent(boundary);
        const auto q_double_prime_before = current_segment.curvature(boundary);

        // Advance cursor to boundary (moves to next segment)
        cursor.seek(boundary);
        auto segment_after = *cursor;

        // Sample geometry from segment starting at boundary
        const auto q_prime_after = segment_after.tangent(boundary);
        const auto q_double_prime_after = segment_after.curvature(boundary);

        // Compute acceleration limit curve (s_dot_max_acc) on both sides of boundary
        const auto [s_dot_max_acc_before, s_dot_max_vel_before] =
            compute_velocity_limits(q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        const auto [s_dot_max_acc_after, s_dot_max_vel_after] =
            compute_velocity_limits(q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // Switching velocity is the minimum of acceleration limit curve on both sides (equation 38)
        const double s_dot_switching = std::min(s_dot_max_acc_before, s_dot_max_acc_after);

        // Validate switching point conditions:
        // 1. Switching velocity must be within velocity limits
        const double s_dot_max_vel_boundary = std::min(s_dot_max_vel_before, s_dot_max_vel_after);
        if (s_dot_switching > s_dot_max_vel_boundary + opt.epsilon) {
            continue;  // Switching velocity exceeds velocity limit
        }

        // 2. Must be able to reach this point from the left and continue forward
        // Compute acceleration bounds at switching velocity on both sides
        const auto [s_ddot_min_before, s_ddot_max_before] =
            compute_acceleration_bounds(q_prime_before, q_double_prime_before, s_dot_switching, opt.max_acceleration, opt.epsilon);

        const auto [s_ddot_min_after, s_ddot_max_after] =
            compute_acceleration_bounds(q_prime_after, q_double_prime_after, s_dot_switching, opt.max_acceleration, opt.epsilon);

        // Check that we can pass through this point in forward direction
        // For a discontinuous switching point with piecewise-constant curvature, we need:
        // - Positive acceleration available on both sides (can reach from left, continue on right)
        // - The switching velocity is at the acceleration limit curve on at least one side (touching the limit)
        const bool can_reach_from_left = s_ddot_max_before > -opt.epsilon;
        const bool can_continue_on_right = s_ddot_max_after > -opt.epsilon;
        const bool touches_mvc = (std::abs(s_dot_switching - s_dot_max_acc_before) < opt.epsilon) ||
                                 (std::abs(s_dot_switching - s_dot_max_acc_after) < opt.epsilon);

        // TODO: Equation 38: Validate that this discontinuity is a trajectory SINK.
        // A discontinuity is a valid switching point only if the maximum acceleration trajectory
        // flows into it (is a sink), not away from it (source). We check this by comparing
        // the trajectory slope (s_ddot_max/s_dot) with the limit curve slope (d/ds s_dot_max_acc).

        if (can_reach_from_left && can_continue_on_right && touches_mvc) {
            return trajectory::phase_point{.s = boundary, .s_dot = s_dot_switching};
        }
    }

    // No valid switching point found before path end - return end of path as switching point
    return trajectory::phase_point{.s = cursor.path().length(), .s_dot = 0.0};
}

// Searches for a velocity switching point where escape from velocity curve becomes possible.
// Implements both continuous (equation 40) and discontinuous (equations 41-42) cases from Section VII-B.
// Returns the switching point on velocity curve. If no escape point found, returns end of path.
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] trajectory::phase_point find_velocity_switching_point(path::cursor cursor, const trajectory::options& opt) {
    const arc_length path_length = cursor.path().length();

    std::optional<trajectory::phase_point> discontinuous_switching_point;
    std::optional<trajectory::phase_point> continuous_switching_point;

    // Phase 1: Check for discontinuous switching points at segment boundaries (equations 41-42).
    // Section VII-B case 2: f''_i(s) being discontinuous is a necessary condition for
    // s_ddot_min(s, s_dot_max_vel(s)) being discontinuous.
    // We need to check ALL boundaries to find the earliest one, not return immediately.
    auto boundary_cursor = cursor;
    while (boundary_cursor.position() < path_length) {
        auto current_segment = *boundary_cursor;
        const arc_length boundary = current_segment.end();

        // Make sure we are not at the end of the path
        if (boundary >= path_length) {
            break;
        }

        // Only check boundaries ahead of our starting position
        if (boundary <= cursor.position()) {
            boundary_cursor.seek(boundary);
            continue;
        }

        // Sample geometry from segment ending at boundary
        const auto q_prime_before = current_segment.tangent(boundary);
        const auto q_double_prime_before = current_segment.curvature(boundary);

        // Advance cursor to boundary (moves to next segment)
        boundary_cursor.seek(boundary);
        auto segment_after = *boundary_cursor;

        // Sample geometry from segment starting at boundary
        const auto q_prime_after = segment_after.tangent(boundary);
        const auto q_double_prime_after = segment_after.curvature(boundary);

        // Compute velocity limits on both sides
        const auto [s_dot_max_acc_before, s_dot_max_vel_before] =
            compute_velocity_limits(q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        const auto [s_dot_max_acc_after, s_dot_max_vel_after] =
            compute_velocity_limits(q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // Skip if velocity limits are degenerate on either side
        if (std::abs(s_dot_max_vel_before) < opt.epsilon || std::abs(s_dot_max_vel_after) < opt.epsilon) {
            continue;
        }

        // Compute curve slopes on both sides (equation 37)
        const double curve_slope_before =
            compute_velocity_limit_derivative(q_prime_before, q_double_prime_before, opt.max_velocity, opt.epsilon);

        const double curve_slope_after =
            compute_velocity_limit_derivative(q_prime_after, q_double_prime_after, opt.max_velocity, opt.epsilon);

        // Compute minimum accelerations at velocity limits on both sides
        const auto [s_ddot_min_before, s_ddot_max_before] =
            compute_acceleration_bounds(q_prime_before, q_double_prime_before, s_dot_max_vel_before, opt.max_acceleration, opt.epsilon);

        const auto [s_ddot_min_after, s_ddot_max_after] =
            compute_acceleration_bounds(q_prime_after, q_double_prime_after, s_dot_max_vel_after, opt.max_acceleration, opt.epsilon);

        // Check equations 41 and 42:
        // (s_ddot_min(s-, s_dot_max_vel(s-)) / s_dot >= d/ds s_dot_max_vel(s-))  (41)
        // AND
        // (s_ddot_min(s+, s_dot_max_vel(s+)) / s_dot <= d/ds s_dot_max_vel(s+))  (42)
        const double trajectory_slope_before = s_ddot_min_before / s_dot_max_vel_before;
        const double trajectory_slope_after = s_ddot_min_after / s_dot_max_vel_after;

        const bool condition_41 = (trajectory_slope_before >= curve_slope_before - opt.epsilon);
        const bool condition_42 = (trajectory_slope_after <= curve_slope_after + opt.epsilon);

        if (condition_41 && condition_42) {
            // Valid discontinuous switching point found
            // Switching velocity is the minimum of velocity limits on both sides
            const double switching_velocity = std::min(s_dot_max_vel_before, s_dot_max_vel_after);

            // Record first discontinuous switching point, but don't return yet - need to check continuous cases too
            discontinuous_switching_point = trajectory::phase_point{.s = boundary, .s_dot = switching_velocity};
            break;  // First discontinuous point found, can stop searching for more discontinuous points
        }
    }

    // Phase 2: Coarse forward search for continuous escape condition (equation 40).
    // Walk along the velocity limit curve until we find a point where s_ddot_min/s_dot <= curve_slope,
    // indicating the trajectory can drop below the curve and resume normal acceleration.
    // If we already found a discontinuous switching point, we can bound the search to stop there.
    std::optional<arc_length> escape_region_start;
    arc_length previous_position = cursor.position();
    const arc_length search_limit = discontinuous_switching_point.has_value() ? discontinuous_switching_point->s : path_length;

    auto search_cursor = cursor;  // this creates a copy of cursor but preserves the pattern above of having a `boundary_cursor`
    const double step_size = opt.delta.count();
    while (search_cursor.position() < search_limit) {
        // Advance cursor by step size
        const arc_length next_position = search_cursor.position() + arc_length{step_size};
        if (next_position >= search_limit) {
            break;  // Reached search limit without finding escape
        }
        search_cursor.seek(next_position);

        // Query geometry at current position
        const auto q_prime = search_cursor.tangent();
        const auto q_double_prime = search_cursor.curvature();

        // Compute velocity limit and its derivative at this position
        const auto [s_dot_max_acc, s_dot_max_vel] =
            compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // Velocity limit must be positive to evaluate escape condition
        if (s_dot_max_vel < opt.epsilon) {
            previous_position = search_cursor.position();
            continue;  // Skip positions with degenerate velocity limits
        }

        const double curve_slope = compute_velocity_limit_derivative(q_prime, q_double_prime, opt.max_velocity, opt.epsilon);

        // Compute minimum acceleration at velocity limit (equation 40 condition)
        const auto [s_ddot_min, s_ddot_max] =
            compute_acceleration_bounds(q_prime, q_double_prime, s_dot_max_vel, opt.max_acceleration, opt.epsilon);

        // Escape condition: trajectory slope (s_ddot_min / s_dot) is less than or equal to curve slope.
        // This means applying minimum acceleration would cause trajectory to drop below velocity limit.
        const double trajectory_slope = s_ddot_min / s_dot_max_vel;

        if (trajectory_slope <= curve_slope + opt.epsilon) {
            // Found escape region - record where we first detected it
            escape_region_start = search_cursor.position();
            break;
        }

        // Update previous position for next iteration
        previous_position = search_cursor.position();
    }

    // Phase 3: Bisection refinement to find exact continuous switching point (if escape region found).
    if (escape_region_start.has_value()) {
        // We know escape is possible somewhere in the last step [previous_position, escape_region_start].
        // Bisect over this interval to find the exact location where the escape condition becomes true.
        arc_length before = previous_position;
        arc_length after = *escape_region_start;

        // TODO: Eleminiate this hardcoded constant.
        constexpr int max_bisection_iterations = 100;
        for (int iteration = 0; iteration < max_bisection_iterations; ++iteration) {
            // Check convergence
            if ((after - before) < arc_length{opt.epsilon}) {
                break;
            }

            // Evaluate midpoint
            const arc_length mid = before + arc_length{(static_cast<double>(after - before) / 2.0)};
            search_cursor.seek(mid);

            const auto q_prime = search_cursor.tangent();
            const auto q_double_prime = search_cursor.curvature();

            const auto [s_dot_max_acc, s_dot_max_vel] =
                compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

            if (s_dot_max_vel < opt.epsilon) {
                // Degenerate case - skip to after
                before = mid;
                continue;
            }

            const double curve_slope = compute_velocity_limit_derivative(q_prime, q_double_prime, opt.max_velocity, opt.epsilon);

            const auto [s_ddot_min, s_ddot_max] =
                compute_acceleration_bounds(q_prime, q_double_prime, s_dot_max_vel, opt.max_acceleration, opt.epsilon);

            const double trajectory_slope = s_ddot_min / s_dot_max_vel;

            if (trajectory_slope <= curve_slope + opt.epsilon) {
                // Midpoint satisfies escape condition - narrow to [before, mid]
                after = mid;
            } else {
                // Midpoint doesn't satisfy escape - narrow to [mid, after]
                before = mid;
            }
        }

        // Store the refined continuous switching point
        search_cursor.seek(after);
        const auto q_prime = search_cursor.tangent();
        const auto q_double_prime = search_cursor.curvature();
        const auto [s_dot_max_acc, s_dot_max_vel] =
            compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // TODO: Implement Addendum E validation - reject switching points where s_dot_max_vel > s_dot_max_acc.
        // Per the reference implementation (Trajectory.cpp:129-133), if the velocity switching point has
        // s_dot > s_dot_max_acc, backward integration will immediately hit the acceleration limit curve,
        // causing the algorithm to fail. We should continue searching forward until finding a switching
        // point where s_dot_max_vel <= s_dot_max_acc + epsilon.

        continuous_switching_point = trajectory::phase_point{.s = after, .s_dot = s_dot_max_vel};
    }

    // Phase 4: Return whichever switching point comes first.
    // If both exist, return the one with smaller s (earlier along the path).
    // If only one exists, return that one.
    // If neither exists, return end of path.
    if (discontinuous_switching_point.has_value() && continuous_switching_point.has_value()) {
        // Both types found - return whichever is earlier
        if (discontinuous_switching_point->s < continuous_switching_point->s) {
            return *discontinuous_switching_point;
        } else if (continuous_switching_point->s < discontinuous_switching_point->s) {
            return *continuous_switching_point;
        } else {
            // Same location (within epsilon) - use whichever has lower velocity (more conservative)
            return (discontinuous_switching_point->s_dot < continuous_switching_point->s_dot) ? *discontinuous_switching_point
                                                                                              : *continuous_switching_point;
        }
    } else if (discontinuous_switching_point.has_value() && !continuous_switching_point.has_value()) {
        return *discontinuous_switching_point;
    } else if (continuous_switching_point.has_value() && !discontinuous_switching_point.has_value()) {
        return *continuous_switching_point;
    } else {
        // No switching point found - return end of path
        return trajectory::phase_point{.s = path_length, .s_dot = 0.0};
    }
}

// Unified switching point search that calls both acceleration and velocity searches.
// Per the reference implementation and paper, we must search for BOTH types of switching points
// and return whichever comes first along the path. The curves can cross, so even if we hit the
// acceleration curve, the velocity switching point might come before the acceleration switching point.
//
// When both switching points are at the same location (within epsilon), we return whichever has
// lower velocity, as this is more conservative and less likely to cause immediate limit curve hit
// during backward integration.
//
// TODO: Performance optimization - bound velocity search by acceleration result.
// The reference implementation (Trajectory.cpp:129-133) demonstrates that after finding the
// acceleration switching point, we can bound the velocity search to stop at that position.
// This can reduce velocity search cost by 10-100x when there's an acceleration switching point nearby.
// For now, we call both independently, but we should pass acceleration result to velocity search.
//
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] trajectory::phase_point find_switching_point(path::cursor cursor, const trajectory::options& opt) {
    // Always search for both types of switching points
    auto accel_sp = find_acceleration_switching_point(cursor, opt);
    auto vel_sp = find_velocity_switching_point(cursor, opt);

    // Determine if switching points are at distinct locations
    const auto s_difference = std::abs(static_cast<double>(accel_sp.s - vel_sp.s));
    const bool at_same_location = (s_difference < opt.epsilon);

    if (at_same_location) {
        // Both switching points at same location (within epsilon)
        // Use whichever has lower velocity (more conservative)
        return (accel_sp.s_dot < vel_sp.s_dot) ? accel_sp : vel_sp;
    }

    // Return whichever comes first along the path
    return (accel_sp.s < vel_sp.s) ? accel_sp : vel_sp;
}

}  // namespace

trajectory::integration_observer::~integration_observer() = default;

trajectory::trajectory(class path p, options opt, integration_points points)
    : path_{std::move(p)}, options_{std::move(opt)}, integration_points_{std::move(points)} {
    if (path_.empty() || path_.length() <= arc_length{0.0}) {
        throw std::invalid_argument{"Path must not be empty"};
    }

    // Must have at least 2 integration points (start and end)
    if (integration_points_.size() < 2) {
        throw std::invalid_argument{"Trajectory must have at least 2 integration points"};
    }

    // First point must be at t=0, s=0 (trajectory start)
    if (integration_points_.front().time != seconds{0.0}) {
        throw std::invalid_argument{"First integration point must have time == 0"};
    }

    if (integration_points_.front().s != arc_length{0.0}) {
        throw std::invalid_argument{"First integration point must have arc length == 0"};
    }

    // Last point must reach end of path
    if (integration_points_.back().s != path_.length()) {
        throw std::invalid_argument{"Last integration point must have arc length == path.length()"};
    }

    // Set duration from last integration point
    duration_ = integration_points_.back().time;
}

trajectory::trajectory(class path p, options opt) : path_{std::move(p)}, options_{std::move(opt)} {
    // Verify NSDMI set duration to zero (safety check in case NSDMI is broken)
    assert(duration_ == seconds{0.0});

    // Start every trajectory at rest at the beginning
    integration_points_.push_back({.time = seconds{0.0}, .s = arc_length{0.0}, .s_dot = 0.0, .s_ddot = 0.0});
}

trajectory trajectory::create(class path p, options opt, integration_points points) {
    if (opt.max_velocity.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_velocity DOF doesn't match path DOF"};
    }

    if (opt.max_acceleration.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_acceleration DOF doesn't match path DOF"};
    }

    if (!xt::all(xt::isfinite(opt.max_velocity) && opt.max_velocity >= 0.0)) {
        throw std::invalid_argument{"max_velocity must be finite and non-negative"};
    }

    if (!xt::all(xt::isfinite(opt.max_acceleration) && opt.max_acceleration >= 0.0)) {
        throw std::invalid_argument{"max_acceleration must be finite and non-negative"};
    }

    if (opt.delta <= seconds{0.0}) {
        throw std::invalid_argument{"delta must be positive"};
    }

    if (opt.epsilon <= 0.0) {
        throw std::invalid_argument{"epsilon must be positive"};
    }

    if (points.empty()) {
        // Production path: run the TOTG algorithm to compute time-optimal parameterization.
        // Algorithm from Kunz & Stilman Section VI: phase plane (s, ṡ) integration.

        // Construct trajectory object early so we can pass it to observers during integration
        // Constructor initializes trajectory with initial point at rest (0, 0)
        trajectory traj{std::move(p), std::move(opt)};

        // Helper to detect intersection between backward trajectory and forward trajectory in phase plane.
        // Returns index in forward trajectory where intersection occurs, or nullopt if no intersection.
        // Intersection means backward point's s_dot exceeds forward trajectory's interpolated s_dot at same s.
        const auto find_trajectory_intersection = [&](arc_length backward_s, double backward_s_dot) -> std::optional<size_t> {
            const auto& forward_points = traj.integration_points_;

            // Forward trajectory must have at least 2 points by the time we're in backward integration.
            // If not, the state machine has a serious logic bug (how did we reach backward with no forward points?).
            if (forward_points.size() < 2) [[unlikely]] {
                throw std::logic_error{"find_trajectory_intersection called with fewer than 2 forward points - state machine bug"};
            }

            // Binary search for the pair of forward points that bracket backward_s
            // We need forward_points[i].s <= backward_s < forward_points[i+1].s
            auto it =
                std::upper_bound(forward_points.begin(), forward_points.end(), backward_s, [](arc_length s, const integration_point& pt) {
                    return s < pt.s;
                });

            // If backward_s is before first forward point, no intersection possible
            if (it == forward_points.begin()) {
                return std::nullopt;
            }

            // Get bracketing points: pt0.s <= backward_s < pt1.s (or pt1 is end)
            auto pt1_it = it;
            auto pt0_it = std::prev(it);
            const auto& pt0 = *pt0_it;

            // Handle edge case: backward_s is at or past last forward point
            if (pt1_it == forward_points.end()) {
                // Backward is beyond forward trajectory - compare with last point
                if (backward_s_dot > pt0.s_dot + traj.options_.epsilon) {
                    // Intersection at last forward point
                    return std::distance(forward_points.begin(), pt0_it);
                }
                return std::nullopt;
            }

            const auto& pt1 = *pt1_it;

            // Interpolate forward trajectory's s_dot at backward_s using linear interpolation
            // between pt0 and pt1. Forward uses piecewise constant acceleration, which gives
            // piecewise linear velocity: s_dot(s) = s_dot0 + (s_ddot0 / s_dot0) * (s - s0)
            //
            // But for intersection detection, simple linear interpolation of s_dot is sufficient
            // and more robust (avoids division by near-zero s_dot).
            const double s_range = static_cast<double>(pt1.s - pt0.s);
            if (s_range < traj.options_.epsilon) [[unlikely]] {
                // Points are too close - compare directly with pt0
                if (backward_s_dot > pt0.s_dot + traj.options_.epsilon) {
                    return std::distance(forward_points.begin(), pt0_it);
                }
                return std::nullopt;
            }

            const double s_offset = static_cast<double>(backward_s - pt0.s);
            const double interpolation_factor = s_offset / s_range;
            const double forward_s_dot_interp = pt0.s_dot + (interpolation_factor * (pt1.s_dot - pt0.s_dot));

            // Intersection occurs if backward's s_dot exceeds forward's interpolated s_dot.
            // Backward integration starts with low s_dot and increases as s decreases, eventually
            // crossing above the forward trajectory's s_dot at the same s position.
            if (backward_s_dot > forward_s_dot_interp + traj.options_.epsilon) {
                // Intersection found - return index of pt0 (start of bracketing interval)
                return std::distance(forward_points.begin(), pt0_it);
            }

            return std::nullopt;
        };

        // State transition function - validates and performs state transitions based on events
        const auto transition = [](integration_state current_state, integration_event event) -> integration_state {
            switch (current_state) {
                case integration_state::k_forward_accelerating:
                    switch (event) {
                        case integration_event::k_hit_velocity_curve:
                            // Hit velocity limit curve - can potentially follow it tangentially
                            return integration_state::k_forward_following_velocity_curve;
                        case integration_event::k_reached_end:
                            // Reaching end means we found a switching point. Could be from normal forward
                            // acceleration reaching the end, or from hitting acceleration curve and finding
                            // a switching point (which gets handled inline and transitions via this event).
                            return integration_state::k_backward_decelerating;
                        default:
                            throw std::logic_error{"Invalid event for k_forward_accelerating state"};
                    }

                case integration_state::k_forward_following_velocity_curve:
                    switch (event) {
                        case integration_event::k_escaped_limit_curve:
                            return integration_state::k_forward_accelerating;
                        case integration_event::k_reached_end:
                            // Reaching end while on curve is a switching point - need backward pass
                            return integration_state::k_backward_decelerating;
                        default:
                            throw std::logic_error{"Invalid event for k_forward_following_velocity_curve state"};
                    }

                case integration_state::k_backward_decelerating:
                    switch (event) {
                        case integration_event::k_hit_forward_trajectory:
                            // Spliced backward into forward, resume forward integration
                            return integration_state::k_forward_accelerating;
                        default:
                            // Note: k_reached_start is NOT a valid transition - backward integration
                            // reaching s=0 without intersection is a physics violation (would require
                            // non-zero initial velocity) and throws an error instead of transitioning.
                            throw std::logic_error{"Invalid event for k_backward_decelerating state"};
                    }
            }

            throw std::logic_error{"Unhandled state in transition function"};
        };

        // State machine for integration algorithm
        integration_state state = integration_state::k_forward_accelerating;

        // Create path cursor for querying geometry during integration
        path::cursor path_cursor = traj.path_.create_cursor();

        // Scratch buffer for backward integration points. Backward integration builds a trajectory
        // moving from switching point toward start. Points are accumulated here, then reversed and
        // spliced into the main trajectory when intersection with forward trajectory is found.
        //
        // TODO: Revisit how state is passed between integration states. Consider using stateful
        // events that carry context, rather than state machine-scoped variables.
        std::vector<integration_point> backward_points;

        // Integration loop runs until we've covered the entire path.
        // Loop exits when last integration point reaches path end exactly.
        while (traj.integration_points_.back().s != traj.path_.length()) {
            integration_event event = integration_event::k_none;

            switch (state) {
                case integration_state::k_forward_accelerating: {
                    // On first entry to this state, notify observer we're starting forward integration
                    if (traj.integration_points_.size() == 1) {
                        // Only initial point exists - this is the very start of trajectory generation
                        if (traj.options_.observer) {
                            traj.options_.observer->on_started_forward_integration({.s = arc_length{0.0}, .s_dot = 0.0});
                        }
                    }

                    // Starting point is the last integration point (known to be feasible)
                    const auto& current_point = traj.integration_points_.back();

                    // Position cursor at current integration point (source of truth)
                    // This ensures each state is self-contained and doesn't depend on cursor
                    // position from previous state. O(1) amortized if cursor is nearby.
                    path_cursor.seek(current_point.s);

                    // Query path geometry at current position to compute maximum acceleration
                    const auto q_prime = path_cursor.tangent();
                    const auto q_double_prime = path_cursor.curvature();

                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, current_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    if (s_ddot_min > s_ddot_max) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: acceleration bounds are infeasible"};
                    }

                    // Compute candidate next point via Euler integration with maximum acceleration
                    const auto [next_s, next_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_max, traj.options_.delta.count());

                    // Forward integration should move "up and to the right" in phase plane
                    if ((next_s <= current_point.s) || (next_s_dot < current_point.s_dot)) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: forward integration must increase both s and s_dot"};
                    }

                    // Check if this step would take us past the end of the path
                    if (next_s >= traj.path_.length()) {
                        // Reached end - this is a switching point at rest. This becomes the initial point
                        // for backward integration to correct the over-optimistic forward trajectory.
                        integration_point switching_point{
                            .time = current_point.time + traj.options_.delta, .s = traj.path_.length(), .s_dot = 0.0, .s_ddot = 0.0};

                        // Initialize backward integration with switching point as starting position
                        backward_points.push_back(std::move(switching_point));

                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Probe the candidate next point to check if it would exceed limit curves.
                    // Create a temporary cursor at the candidate position to query geometry there.
                    auto probe_cursor = path_cursor;
                    probe_cursor.seek(next_s);
                    const auto probe_q_prime = probe_cursor.tangent();
                    const auto probe_q_double_prime = probe_cursor.curvature();

                    // Check if the candidate next point would violate velocity or acceleration limits
                    const auto [s_dot_max_acc, s_dot_max_vel] = compute_velocity_limits(probe_q_prime,
                                                                                        probe_q_double_prime,
                                                                                        traj.options_.max_velocity,
                                                                                        traj.options_.max_acceleration,
                                                                                        traj.options_.epsilon);
                    const auto s_dot_limit = std::min(s_dot_max_acc, s_dot_max_vel);

                    if (s_dot_limit <= 0.0) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive"};
                    }

                    // Check if the candidate point exceeds the limit curve
                    if ((s_dot_limit - next_s_dot) < traj.options_.epsilon) {
                        // Candidate point would hit or exceed limit curve.
                        //
                        // TODO: For robustness (Section VIII-A of paper), we should:
                        // 1. Use bisection to find exact intersection with limit curve
                        // 2. Check if intersection point is a trajectory source or sink
                        // 3. Only transition if it's a sink (not a source due to numerical overshoot)
                        // For now, we transition immediately, which may be less robust with large dt.
                        //
                        // Note: The phase_point passed to observer is the INFEASIBLE candidate point
                        // that exceeded limits, not a feasible point on the limit curve.

                        if (traj.options_.observer) {
                            traj.options_.observer->on_hit_limit_curve({.s = next_s, .s_dot = next_s_dot}, s_dot_max_acc, s_dot_max_vel);
                        }

                        // Determine which limit curve we hit by comparing the two curves.
                        // The lower curve is the active constraint at this position.
                        //
                        // TODO: When curves are within epsilon of each other, it's arbitrary whether we treat
                        // this as hitting the velocity curve (and potentially following it) or the acceleration
                        // curve (and searching for a switching point). Current choice: if curves are within
                        // epsilon, treat as acceleration curve hit (search for switching point). This is
                        // conservative in the sense that searching is always safe, while curve following
                        // requires the curve to be well-defined. However, investigate whether there's a
                        // better heuristic (e.g., based on curve derivatives or previous state).

                        if ((s_dot_max_acc - s_dot_max_vel) > traj.options_.epsilon) {
                            event = integration_event::k_hit_velocity_curve;
                            break;
                        }

                        // Hit limit curve. Search forward along the path for a switching point where
                        // backward integration can begin. The unified search checks for both acceleration
                        // discontinuities and velocity escape points, returning whichever comes first.
                        auto switching_point = find_switching_point(path_cursor, traj.options_);

                        // Initialize backward integration from switching point
                        integration_point sp{.time = current_point.time + traj.options_.delta,  // Placeholder, corrected during splice
                                             .s = switching_point.s,
                                             .s_dot = switching_point.s_dot,
                                             .s_ddot = 0.0};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Candidate point is feasible - accept it and advance
                    // Note: Don't call maybe_on_trajectory_extended() here - forward points are tentative
                    // and may be discarded during backward integration. Only finalized segments trigger that callback.
                    integration_point next_point{
                        .time = current_point.time + traj.options_.delta, .s = next_s, .s_dot = next_s_dot, .s_ddot = s_ddot_max};
                    traj.integration_points_.push_back(std::move(next_point));

                    // Continue in same state (no event)
                    // Cursor will be repositioned on next iteration entry
                    break;
                }

                case integration_state::k_forward_following_velocity_curve: {
                    // Algorithm Step 3: Analyze velocity limit curve and decide whether to escape,
                    // follow tangentially, or search for switching point.
                    //
                    // We know we're on a velocity curve (acceleration curves go directly to searching).
                    // Check if we can escape back below the curve, must search for switching point,
                    // or should follow the curve tangentially.

                    // Starting point is the last feasible integration point
                    const auto& current_point = traj.integration_points_.back();

                    // Position cursor at current point to query geometry
                    path_cursor.seek(current_point.s);
                    const auto q_prime = path_cursor.tangent();
                    const auto q_double_prime = path_cursor.curvature();

                    // Compute the velocity limit at current position. We're following this curve,
                    // so our trajectory velocity should be on or very close to this limit.
                    const auto [s_dot_max_acc, s_dot_max_vel] = compute_velocity_limits(
                        q_prime, q_double_prime, traj.options_.max_velocity, traj.options_.max_acceleration, traj.options_.epsilon);

                    // Compute the slope of the velocity limit curve in phase plane (eq 37).
                    // This is d/ds s_dot_max_vel(s), telling us how the velocity limit changes along the path.
                    const double curve_slope =
                        compute_velocity_limit_derivative(q_prime, q_double_prime, traj.options_.max_velocity, traj.options_.epsilon);

                    // Compute feasible acceleration bounds at the velocity limit per paper Algorithm Step 3.
                    // The paper specifies evaluating s_ddot_min/max at s_dot_max_vel(s), not at current trajectory velocity.
                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, s_dot_max_vel, traj.options_.max_acceleration, traj.options_.epsilon);

                    // TODO: Investigate whether we want a richer exception type for algorithm failures.
                    // A custom exception hierarchy could distinguish between different failure modes
                    // (infeasible path, numerical issues, constraint violations) for better error handling.
                    if (s_ddot_min > s_ddot_max) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: acceleration bounds are infeasible during curve following"};
                    }

                    // Check exit conditions by comparing acceleration bounds with curve slope.
                    // The ratio s_ddot / s_dot gives the slope of a trajectory in phase plane:
                    // ds_dot/ds = (ds_dot/dt)/(ds/dt) = s_ddot/s_dot (from chain rule in phase plane).
                    // We compare this with the limit curve's slope to determine if we can escape or must search.
                    //
                    // Three cases (from paper Section VI, step 3):
                    // 1. s_ddot_max / s_dot < curve_slope - epsilon: Can escape (trajectory curves below limit)
                    // 2. s_ddot_min / s_dot > curve_slope + epsilon: Must search (trapped on curve)
                    // 3. Otherwise: Follow curve tangentially
                    //
                    // Note: We use s_dot_max_vel in the slope calculations per Algorithm Step 3.

                    // TODO: Revisit whether we need epsilon checks before dividing by s_dot_max_vel here. During forward
                    // integration following the velocity curve, s_dot_max_vel should always be positive. However, if the
                    // velocity limit itself becomes very small (e.g., tight curvature with low velocity limits), the division
                    // could become numerically unstable. For now, throw if s_dot_max_vel is near zero as a canary.
                    if (s_dot_max_vel < traj.options_.epsilon) [[unlikely]] {
                        throw std::runtime_error{
                            "TOTG algorithm error: cannot evaluate curve exit conditions with near-zero velocity limit"};
                    }

                    if ((curve_slope - (s_ddot_max / s_dot_max_vel)) > traj.options_.epsilon) {
                        // Maximum acceleration trajectory curves away from the limit curve (less steeply upward).
                        // The trajectory's slope (s_ddot_max / s_dot) is less than the limit curve's slope,
                        // meaning we're moving upward more slowly than the curve, effectively dropping below it.
                        // We can escape the curve and resume normal maximum acceleration integration.
                        event = integration_event::k_escaped_limit_curve;
                        break;
                    }

                    if (((s_ddot_min / s_dot_max_vel) - curve_slope) > traj.options_.epsilon) {
                        // Trapped on limit curve with no way to escape via normal acceleration.
                        // Search forward for a switching point where backward integration can begin.
                        // The unified search checks for both acceleration discontinuities and velocity
                        // escape points, returning whichever comes first.
                        auto switching_point = find_switching_point(path_cursor, traj.options_);

                        // Initialize backward integration from switching point
                        integration_point sp{.time = current_point.time + traj.options_.delta,  // Placeholder, corrected during splice
                                             .s = switching_point.s,
                                             .s_dot = switching_point.s_dot,
                                             .s_ddot = 0.0};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Neither escape nor search conditions met - follow the limit curve tangentially.
                    // Compute the tangent acceleration: the rate of change that keeps us on the curve.
                    const double s_ddot_curve = curve_slope * s_dot_max_vel;

                    // Validate that tangent acceleration is within feasible bounds.
                    // If the curve's tangent falls outside our acceleration capabilities, the trajectory
                    // is infeasible at this point (algorithm error - shouldn't reach here).
                    // TODO: Investigate whether we want a richer exception type for infeasible trajectories.
                    // Could provide diagnostic information about which constraints failed and where.
                    if (s_ddot_curve < s_ddot_min || s_ddot_curve > s_ddot_max) [[unlikely]] {
                        throw std::runtime_error{
                            "TOTG algorithm error: velocity curve tangent acceleration outside feasible bounds - "
                            "trajectory is infeasible"};
                    }

                    // Integrate forward along the curve with tangent acceleration
                    const auto [next_s, next_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_curve, traj.options_.delta.count());

                    // Curve following must advance along path (s increases), but s_dot can decrease
                    // when following a descending velocity limit curve (negative slope).
                    if (next_s <= current_point.s) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: curve following must increase s"};
                    }

                    // Check if we've reached the end of the path while following the curve
                    if (next_s >= traj.path_.length()) {
                        // Reached end while on velocity limit curve - this is a switching point at rest.
                        // The forward trajectory hit the end with non-zero velocity (on the curve),
                        // so we must perform backward integration to create a feasible deceleration.
                        integration_point switching_point{
                            .time = current_point.time + traj.options_.delta, .s = traj.path_.length(), .s_dot = 0.0, .s_ddot = 0.0};

                        // Initialize backward integration with switching point as starting position
                        backward_points.push_back(std::move(switching_point));

                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Accept the point and continue following the curve.
                    // The next iteration will re-evaluate whether we're still on the curve or can escape.
                    integration_point next_point{
                        .time = current_point.time + traj.options_.delta, .s = next_s, .s_dot = next_s_dot, .s_ddot = s_ddot_curve};
                    traj.integration_points_.push_back(std::move(next_point));

                    // No event - remain in same state and re-evaluate on next iteration
                    // The loop will check exit conditions again with updated position
                    break;
                }

                case integration_state::k_backward_decelerating: {
                    // Backward integration from a switching point toward the start, building a trajectory
                    // that decelerates to meet constraints. Uses minimum (negative) acceleration to move
                    // "up and left" in phase plane: s decreases, s_dot increases.

                    // On first entry to this state, validate switching point and notify observer
                    if (backward_points.size() == 1) {
                        const auto& switching_point = backward_points.back();
                        const auto& last_forward = traj.integration_points_.back();

                        // Switching point must be "down and to the right" of last forward point:
                        // - Higher s (further along path)
                        // - Lower s_dot (slower, often at rest)
                        // This ensures backward integration can increase s_dot while decreasing s.
                        if ((switching_point.s <= last_forward.s) || (switching_point.s_dot >= last_forward.s_dot)) [[unlikely]] {
                            throw std::runtime_error{
                                "TOTG algorithm error: switching point must be down and to the right of last forward point "
                                "(higher s, lower s_dot)"};
                        }

                        if (traj.options_.observer) {
                            traj.options_.observer->on_started_backward_integration(
                                {.s = switching_point.s, .s_dot = switching_point.s_dot});
                        }
                    }

                    // Starting point is the last backward integration point. On first entry to this state,
                    // the initial backward point was pushed by the forward state that found the switching point.
                    const auto& current_point = backward_points.back();

                    // Position cursor at current backward point to query geometry (source of truth)
                    path_cursor.seek(current_point.s);

                    // Query path geometry at current position to compute minimum acceleration
                    const auto q_prime = path_cursor.tangent();
                    const auto q_double_prime = path_cursor.curvature();

                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, current_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    if (s_ddot_min > s_ddot_max) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: acceleration bounds are infeasible during backward integration"};
                    }

                    // Minimum acceleration must be negative to produce backward motion (decreasing s)
                    if (s_ddot_min >= 0.0) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: backward integration requires negative minimum acceleration"};
                    }

                    // Compute candidate next point via Euler integration with negative dt and minimum acceleration.
                    // Negative dt reverses time direction, reconstructing velocities that led to current point.
                    // With s_ddot_min < 0 and dt < 0, s_dot increases (up) while s decreases (left).
                    const auto [candidate_s, candidate_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_min, -traj.options_.delta.count());

                    // Backward integration must decrease s (move backward) and not decrease s_dot.
                    // At degenerate points (s_ddot ≈ 0), s_dot may stay constant (horizontal movement).
                    if ((candidate_s >= current_point.s) || (candidate_s_dot < current_point.s_dot)) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: backward integration must decrease s and not decrease s_dot"};
                    }

                    // Check exit condition 1: Would candidate reach or pass the start of the path?
                    if (candidate_s <= arc_length{0.0}) {
                        // Backward integration reached s=0 without intersecting forward trajectory.
                        // This is physically impossible: backward integration moves "up and to the left"
                        // in phase plane (s decreasing, s_dot increasing), so at s=0 we would have s_dot > 0.
                        // But the trajectory must start at rest (s=0, s_dot=0). This velocity discontinuity
                        // indicates the trajectory is infeasible - there's no way to satisfy both the
                        // initial condition (rest at start) and the switching point constraint (rest at end).
                        throw std::runtime_error{
                            "TOTG algorithm error: backward integration reached start without intersecting "
                            "forward trajectory - trajectory is infeasible (would require non-zero initial velocity)"};
                    }

                    // Query geometry at candidate position to check if it would hit limit curves.
                    // Backward integration hitting a limit curve indicates the trajectory is infeasible -
                    // we cannot decelerate from the switching point without violating joint constraints.
                    auto probe_cursor = path_cursor;
                    probe_cursor.seek(candidate_s);
                    const auto probe_q_prime = probe_cursor.tangent();
                    const auto probe_q_double_prime = probe_cursor.curvature();

                    const auto [s_dot_max_acc, s_dot_max_vel] = compute_velocity_limits(probe_q_prime,
                                                                                        probe_q_double_prime,
                                                                                        traj.options_.max_velocity,
                                                                                        traj.options_.max_acceleration,
                                                                                        traj.options_.epsilon);
                    const auto s_dot_limit = std::min(s_dot_max_acc, s_dot_max_vel);

                    if (s_dot_limit <= 0.0) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive during backward integration"};
                    }

                    // Check exit condition 2: Does candidate intersect forward trajectory in phase plane?
                    // Intersection means backward's s_dot has dropped below forward's s_dot at same s position.
                    //
                    // Performance optimization: Skip intersection check if candidate is still beyond the last
                    // forward point. While integrating backward from switching point, we start far from the
                    // forward trajectory and gradually approach it. Early exit eliminates expensive binary
                    // search until we're actually in range to intersect.
                    //
                    // NOTE: We do this check before checking to see if we have hit the velocity limit curve, since
                    // we may be trying to intersect a forward trajectory that was following the curve. If we check
                    // against the limit curve first, we might error out when we should have intersected.
                    std::optional<size_t> intersection_index;
                    if (candidate_s <= traj.integration_points_.back().s) {
                        intersection_index = find_trajectory_intersection(candidate_s, candidate_s_dot);
                    }

                    if (intersection_index.has_value()) {
                        // Found intersection - accept candidate point before splicing
                        integration_point next_point{.time = current_point.time + traj.options_.delta,  // Placeholder, fixed during splice
                                                     .s = candidate_s,
                                                     .s_dot = candidate_s_dot,
                                                     .s_ddot = s_ddot_min};
                        backward_points.push_back(std::move(next_point));

                        // Splice backward trajectory into forward trajectory at intersection point.
                        // The backward trajectory has lower velocities than the over-optimistic forward,
                        // representing the constraint from needing to decelerate to the switching point.

                        // Truncate forward trajectory after intersection point
                        const size_t truncate_index = *intersection_index + 1;  // Keep intersection point
                        traj.integration_points_.erase(traj.integration_points_.begin() + static_cast<std::ptrdiff_t>(truncate_index),
                                                       traj.integration_points_.end());

                        // Record intersection time - backward trajectory timestamps start here
                        const seconds intersection_time = traj.integration_points_.back().time;

                        // Reverse backward trajectory to get correct chronological order (was built end→start, need start→end)
                        std::ranges::reverse(backward_points);

                        // Reserve space to avoid reallocations during bulk append
                        traj.integration_points_.reserve(traj.integration_points_.size() + backward_points.size());

                        // Append backward trajectory with corrected timestamps
                        // First backward point continues from intersection, subsequent points increment by delta
                        for (size_t i = 0; i < backward_points.size(); ++i) {
                            auto corrected = backward_points[i];
                            corrected.time = intersection_time + seconds{static_cast<double>(i + 1) * traj.options_.delta.count()};
                            traj.integration_points_.push_back(corrected);
                        }

                        // Update trajectory duration to reflect new endpoint before invoking callbacks.
                        // Observers receiving the trajectory by reference may query its duration.
                        traj.duration_ = traj.integration_points_.back().time;

                        // Clear backward scratch buffer for potential reuse if algorithm continues
                        backward_points.clear();

                        // Notify observer that trajectory has been extended with finalized backward segment
                        if (traj.options_.observer) {
                            traj.options_.observer->on_trajectory_extended(traj);
                        }

                        event = integration_event::k_hit_forward_trajectory;
                        break;
                    }

                    // Candidate exceeding limit curve is an algorithm error - trajectory is infeasible.
                    // Note: Being AT the limit (within epsilon) is allowed - only EXCEEDING it is rejected.
                    if ((candidate_s_dot - s_dot_limit) > traj.options_.epsilon) [[unlikely]] {
                        throw std::runtime_error{
                            "TOTG algorithm error: backward integration exceeded limit curve - trajectory is infeasible"};
                    }

                    // Candidate point is feasible - accept it and continue backward integration
                    integration_point next_point{.time = current_point.time + traj.options_.delta,  // Placeholder, fixed during splice
                                                 .s = candidate_s,
                                                 .s_dot = candidate_s_dot,
                                                 .s_ddot = s_ddot_min};
                    backward_points.push_back(std::move(next_point));

                    // Continue in same state (no event)
                    // Cursor will be repositioned on next iteration entry
                    break;
                }
            }

            if (event != integration_event::k_none) {
                state = transition(state, event);
            }
        }

        return traj;
    } else {
        // Test path: validate provided integration points
        // First point must be at t=0
        if (points.front().time != seconds{0.0}) {
            throw std::invalid_argument{"First integration point must have time == 0"};
        }

        // Pairwise validation using C++20 ranges
        auto pairs = std::views::iota(size_t{0}, points.size() - 1) |
                     std::views::transform([&](size_t i) { return std::pair{std::cref(points[i]), std::cref(points[i + 1])}; });

        for (const auto& [curr, next] : pairs) {
            // Times must be strictly increasing
            if (next.get().time <= curr.get().time) {
                throw std::invalid_argument{"Integration points must be sorted by strictly increasing time"};
            }

            // Arc lengths must be non-negative and within path bounds
            if (curr.get().s < arc_length{0.0} || curr.get().s > p.length()) {
                throw std::invalid_argument{"Integration point arc lengths must be in [0, path.length()]"};
            }

            // Arc lengths must be monotonically non-decreasing
            if (next.get().s < curr.get().s) {
                throw std::invalid_argument{"Integration point arc lengths must be monotonically non-decreasing"};
            }
        }

        if (points.back().s < arc_length{0.0} || points.back().s > p.length()) {
            throw std::invalid_argument{"Integration point arc lengths must be in [0, path.length()]"};
        }

        // Construct trajectory with integration points
        // Duration is set automatically in constructor from last integration point
        return trajectory{std::move(p), std::move(opt), std::move(points)};
    }
}

struct trajectory::sample trajectory::sample(trajectory::seconds t) const {
    if (t < trajectory::seconds{0.0} || t > duration_) {
        throw std::out_of_range{"Time out of trajectory bounds"};
    }

    return create_cursor().seek(t).sample();
}

trajectory::seconds trajectory::duration() const noexcept {
    return duration_;
}

const class path& trajectory::path() const noexcept {
    return path_;
}

size_t trajectory::dof() const noexcept {
    return path_.dof();
}

const trajectory::integration_points& trajectory::get_integration_points() const noexcept {
    return integration_points_;
}

const trajectory::options& trajectory::get_options() const noexcept {
    return options_;
}

trajectory::cursor trajectory::create_cursor() const {
    return cursor{this};
}

trajectory::cursor::cursor(const class trajectory* traj)
    : traj_{traj}, time_hint_{traj->integration_points_.begin()}, path_cursor_{traj->path_.create_cursor()} {
    // Constructor initializes cursor at trajectory start (t=0, s=0)
    // time_hint_ points to first integration point (if any)
    // path_cursor_ is at arc length 0 by default
}

const trajectory& trajectory::cursor::trajectory() const noexcept {
    return *traj_;
}

trajectory::seconds trajectory::cursor::time() const noexcept {
    return time_;
}

struct trajectory::sample trajectory::cursor::sample() const {
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot sample cursor at sentinel position"};
    }

    assert(time_hint_ != traj_->get_integration_points().end());
    assert(time_hint_->time <= time_);

    // Use the time hint for O(1) lookup of the integration point at or before current time.
    // The hint is maintained by seek() to always point to the correct interval.
    const integration_point& p0 = *time_hint_;

    // Interpolate path space (s, s_dot, s_ddot) using piecewise constant acceleration between
    // integration points. This is the standard kinematic model: constant acceleration
    // produces linear velocity and quadratic position.
    const double dt = (time_ - p0.time).count();
    const double s_dot = p0.s_dot + (p0.s_ddot * dt);
    const double s_ddot = p0.s_ddot;

    // Query the path geometry at the current arc length position. The path_cursor_ has
    // already been positioned by update_path_cursor_position_ in seek().
    const auto q = path_cursor_.configuration();
    const auto q_prime = path_cursor_.tangent();
    const auto q_double_prime = path_cursor_.curvature();

    // Convert from path space (s, s_dot, s_ddot) to joint space (q, q_dot, q_ddot) using the chain rule.
    // This gives us the actual joint velocities and accelerations that result from
    // moving along the path at the computed path velocity and acceleration.
    //
    // q_dot(t) = q'(s) * s_dot(t)
    const auto q_dot = q_prime * s_dot;

    // q_ddot(t) = q'(s) * s_ddot(t) + q''(s) * s_dot(t)^2
    //
    // The second term captures the centripetal acceleration from following a curved path.
    const auto q_ddot = q_prime * s_ddot + q_double_prime * (s_dot * s_dot);

    return {.time = time_, .configuration = q, .velocity = q_dot, .acceleration = q_ddot};
}

void trajectory::cursor::update_path_cursor_position_(seconds t) {
    // Postcondition: seek() must position time_hint_ correctly
    assert(time_hint_ != traj_->get_integration_points().end());
    assert(time_hint_->time <= t);

    // Interpolate arc length at time t using constant acceleration motion from time_hint_
    // s(t) = s0 + s_dot0 * dt + 0.5 * s_ddot0 * dt^2
    // This assumes piecewise constant acceleration between TOTG integration points

    const auto& p0 = *time_hint_;
    const double dt = (t - p0.time).count();
    const double s_interp = static_cast<double>(p0.s) + (p0.s_dot * dt) + (0.5 * p0.s_ddot * dt * dt);

    // Clamp interpolated arc length to path bounds to handle floating point accumulation.
    // Even when t <= duration, s_interp can slightly exceed path.length() due to numerical
    // error in the kinematic integration, which would put the path cursor at sentinel.
    const double s_clamped = std::min(s_interp, static_cast<double>(traj_->path_.length()));

    // Position path cursor at clamped arc length
    path_cursor_.seek(arc_length{s_clamped});
}

trajectory::cursor& trajectory::cursor::seek(seconds t) {
    // Use +/-infinity sentinels for out-of-bounds positions, matching the pattern used by
    // path::cursor. This allows algorithms to detect when they've stepped beyond trajectory
    // bounds without throwing exceptions on every overstep.
    if (t < seconds{0.0}) {
        time_ = seconds{-std::numeric_limits<double>::infinity()};
        return *this;
    }
    if (t > traj_->duration()) {
        time_ = seconds{std::numeric_limits<double>::infinity()};
        return *this;
    }

    time_ = t;

    assert(time_hint_ != traj_->get_integration_points().end());

    // Maintain a hint iterator pointing to the integration point at or before current time.
    // This provides O(1) amortized performance for sequential time queries (the common case
    // during sampling) by checking nearby integration points before falling back to binary
    // search for large time jumps.
    //
    // Strategy:
    // 1. Check if current hint is still valid (small forward step within same interval)
    // 2. Check adjacent intervals (common during uniform sampling)
    // 3. Binary search for large jumps (rare, but handles arbitrary seeks)
    //
    // TODO: Optimize hint update by detecting seek direction (forward vs backward from current
    // position). We can skip checks that won't work based on direction, simplifying logic and
    // potentially improving performance for directional sequential access patterns.

    const auto& points = traj_->get_integration_points();

    // Fast path: Check if current hint is still valid
    if (time_hint_ != points.end() && time_hint_->time <= t) {
        auto next_hint = time_hint_;
        ++next_hint;
        if (next_hint == points.end() || t < next_hint->time) {
            // Hint is still valid, O(1)
            update_path_cursor_position_(t);
            return *this;
        }
    }

    // Check forward by one (common in sequential forward sampling)
    if (time_hint_ != points.end()) {
        auto next_hint = time_hint_;
        ++next_hint;
        if (next_hint != points.end() && next_hint->time <= t) {
            auto next_next = next_hint;
            ++next_next;
            if (next_next == points.end() || t < next_next->time) {
                time_hint_ = next_hint;
                update_path_cursor_position_(t);
                return *this;
            }
        }
    }

    // Check backward by one (common in backward integration)
    if (time_hint_ != points.begin()) {
        auto prev_hint = time_hint_;
        --prev_hint;
        if (prev_hint->time <= t && t < time_hint_->time) {
            time_hint_ = prev_hint;
            update_path_cursor_position_(t);
            return *this;
        }
    }

    // Large jump - use binary search (O(log n))
    // Find first point with time > t (upper_bound)
    auto it =
        std::upper_bound(points.begin(), points.end(), t, [](seconds value, const integration_point& point) { return value < point.time; });

    // The point before upper_bound contains our time
    if (it == points.begin()) {
        // At or before first point
        time_hint_ = points.begin();
    } else {
        --it;
        time_hint_ = it;
    }

    update_path_cursor_position_(t);
    return *this;
}

trajectory::cursor& trajectory::cursor::seek_by(seconds dt) {
    return seek(time_ + dt);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): Must be non-static for consistent cursor API
std::default_sentinel_t trajectory::cursor::end() const noexcept {
    return std::default_sentinel;
}

bool operator==(const trajectory::cursor& c, std::default_sentinel_t) noexcept {
    return !std::isfinite(c.time_.count());
}

bool operator==(std::default_sentinel_t, const trajectory::cursor& c) noexcept {
    return !std::isfinite(c.time_.count());
}

}  // namespace viam::trajex::totg

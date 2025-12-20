#include <viam/trajex/totg/trajectory.hpp>

#include <cassert>
#include <functional>
#include <numbers>
#include <optional>
#include <ranges>
#include <stdexcept>

#include <boost/range/adaptors.hpp>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/private/phase_plane_slope.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/arc_operations.hpp>
#include "viam/trajex/types/arc_acceleration.hpp"
#include "viam/trajex/types/arc_velocity.hpp"

namespace viam::trajex::totg {

// Verify that trajectory::sampled satisfies range concepts (using uniform_sampler as concrete type)
static_assert(std::ranges::range<trajectory::sampled<uniform_sampler>>);
static_assert(std::ranges::input_range<trajectory::sampled<uniform_sampler>>);
static_assert(std::ranges::view<trajectory::sampled<uniform_sampler>>);

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
                                           class epsilon epsilon) {
    struct result {
        arc_velocity s_dot_max_acc;
        arc_velocity s_dot_max_vel;
    };

    // Compute the path velocity limit imposed by joint acceleration constraints (equation 31).
    // This is the acceleration limit curve in the phase plane. The derivation in the paper
    // converts joint acceleration bounds into constraints on path velocity by considering
    // the centripetal acceleration term q''(s)*s_dot^2 that appears when following a curved
    // path. The result is a set of downward-facing parabolas centered at the origin, and we
    // take the minimum of their positive bounds to find the feasible path velocity.
    arc_velocity s_dot_max_accel{std::numeric_limits<double>::infinity()};

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

            const auto curvature_ratio_i = q_double_prime(i) / q_prime(i);
            const auto curvature_ratio_j = q_double_prime(j) / q_prime(j);
            const auto curvature_difference = std::abs(curvature_ratio_i - curvature_ratio_j);

            if (curvature_difference < epsilon) {
                continue;
            }

            const auto accel_sum = (q_ddot_max(i) / std::abs(q_prime(i))) + (q_ddot_max(j) / std::abs(q_prime(j)));
            const auto limit = std::sqrt(accel_sum / curvature_difference);
            s_dot_max_accel = std::min(s_dot_max_accel, arc_velocity{limit});
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

        const auto limit = std::sqrt(q_ddot_max(i) / std::abs(q_double_prime(i)));
        s_dot_max_accel = std::min(s_dot_max_accel, arc_velocity{limit});
    }

    // Compute the path velocity limit imposed by joint velocity constraints (equation 36).
    // This is the velocity limit curve in the phase plane. For each joint moving along
    // the path, the joint velocity q_dot = q'(s)*s_dot must respect the joint velocity
    // limit, giving us s_dot <= q_dot_max / |q'(s)|. We take the minimum across all joints.
    arc_velocity s_dot_max_vel{std::numeric_limits<double>::infinity()};
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        const auto limit = q_dot_max(i) / std::abs(q_prime(i));
        s_dot_max_vel = std::min(s_dot_max_vel, arc_velocity{limit});
    }

    return result{s_dot_max_accel, s_dot_max_vel};
}

// Computes the feasible range of path acceleration (s_ddot) given current path velocity (s_dot)
// and joint acceleration limits. The path acceleration must satisfy joint constraints in all DOF.
// See Kunz & Stilman equations 22-23.
[[gnu::pure]] auto compute_acceleration_bounds(const xt::xarray<double>& q_prime,
                                               const xt::xarray<double>& q_double_prime,
                                               arc_velocity s_dot,
                                               const xt::xarray<double>& q_ddot_max,
                                               class epsilon epsilon) {
    struct result {
        arc_acceleration s_ddot_min;
        arc_acceleration s_ddot_max;
    };

    arc_acceleration s_ddot_min{-std::numeric_limits<double>::infinity()};
    arc_acceleration s_ddot_max{std::numeric_limits<double>::infinity()};

    // Each joint independently constrains the feasible range of path acceleration.
    // From the chain rule q''(t) = q'(s)*s_ddot + q''(s)*s_dot^2, we can solve for s_ddot given
    // the constraint -q_ddot_max <= q''(t) <= q_ddot_max. The centripetal term q''(s)*s_dot^2 is
    // already "using up" part of the acceleration budget at the current path velocity, which tightens
    // the bounds on how much path acceleration we can apply. Each joint shrinks the feasible region,
    // so we take the intersection by using max for lower bounds and min for upper bounds.
    for (size_t i = 0; i < q_prime.size(); ++i) {
        if (std::abs(q_prime(i)) < epsilon) {
            continue;
        }

        // We don't have a strong type for squared velocity, so step out
        // of the strong types for a moment to compute `centripetal_term`.
        const auto s_dot_double = static_cast<double>(s_dot);
        const auto centripetal_term = q_double_prime(i) * (s_dot_double * s_dot_double);

        // Per equations 22-23: acceleration limit terms use |q'(s)|, centripetal term uses signed q'(s)
        const auto min_from_joint = ((-q_ddot_max(i)) / std::abs(q_prime(i))) - (centripetal_term / q_prime(i));
        s_ddot_min = std::max(s_ddot_min, arc_acceleration{min_from_joint});

        const auto max_from_joint = (q_ddot_max(i) / std::abs(q_prime(i))) - (centripetal_term / q_prime(i));
        s_ddot_max = std::min(s_ddot_max, arc_acceleration{max_from_joint});
    }

    if (epsilon.wrap(s_ddot_min) > epsilon.wrap(s_ddot_max)) [[unlikely]] {
        throw std::runtime_error{"TOTG algorithm error: acceleration bounds are infeasible"};
    }

    // Handle degenerate case where bounds are nearly equal (singularity/zero-acceleration point).
    if (epsilon.wrap(s_ddot_max) == epsilon.wrap(s_ddot_min)) {
        s_ddot_min = s_ddot_max = std::min(s_ddot_min, s_ddot_max);
    }
    assert(s_ddot_min <= s_ddot_max);

    return result{s_ddot_min, s_ddot_max};
}

// Computes the derivative of the velocity limit curve in the phase plane.
// This is d/ds s_dot_max_vel(s), which tells us the slope of the velocity limit curve.
// Used in Algorithm Step 3 to determine if we can leave the curve or must search for switching points.
// See Kunz & Stilman equation 37.
[[gnu::pure]] auto compute_velocity_limit_derivative(const xt::xarray<double>& q_prime,
                                                     const xt::xarray<double>& q_double_prime,
                                                     const xt::xarray<double>& q_dot_max,
                                                     class epsilon epsilon) {
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

    const double numerator = -q_dot_max(limiting_joint) * q_double_prime(limiting_joint);
    const double denominator = q_prime(limiting_joint) * std::abs(q_prime(limiting_joint));

    // TODO(RSDK-12759): This check can trigger even when limiting_joint was validly selected, because a joint
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
    // 1. Consider whether limiting joint selection should filter out near-singular joints
    // 2. Determine correct behavior: throw, return infinity with sign, or use different epsilon threshold
    // 3. May need tighter check: |denominator| < epsilon^2 to match the q' filter tolerance
    //
    // For now, throw to make this case visible during testing rather than silently returning an
    // incorrect value that could cause incorrect curve-following behavior.
    if (std::abs(denominator) < epsilon) {
        throw std::runtime_error{
            "compute_velocity_limit_derivative: denominator near zero for limiting joint - "
            "velocity limit curve derivative is numerically undefined (joint barely moving along path)"};
    }

    return phase_plane_slope{numerator / denominator};
}

// Performs a single Euler integration step in phase plane (s, s_dot).
// Given current position, velocity, and applied acceleration, computes the next state.
// Uses constant acceleration kinematic equations: v_new = v + a*dt, s_new = s + v*dt + 0.5*a*dt^2.
// This is direction-agnostic - caller determines whether dt is positive (forward) or negative (backward).
[[gnu::const]] auto euler_step(arc_length s, arc_velocity s_dot, arc_acceleration s_ddot, trajectory::seconds dt, class epsilon epsilon) {
    struct result {
        arc_length s;
        arc_velocity s_dot;
    };

    const auto s_dot_new = s_dot + (s_ddot * dt);
    const auto s_new = s + (s_dot * dt) + (0.5 * s_ddot * dt * dt);

    if (epsilon.wrap(s_new) == epsilon.wrap(s)) [[unlikely]] {
        if (epsilon.wrap(s_dot_new) == epsilon.wrap(s_dot)) [[unlikely]] {
            throw std::runtime_error{"Euler step will not make sufficient progress in the phase plane"};
        }
    }

    return result{s_new, s_dot_new};
}

// Validates if a segment boundary is a valid acceleration switching point per
// Kunz & Stilman equations 38 and 39, along with `Correction 3` to get the dots
// in the right places, and `Correction 4` (dividing by s_dot) to obtain dimensional
// consistency.
//
// A boundary is valid if the phase trajectory slopes on both sides bracket the acceleration
// limit curve slope change across the discontinuity, and the switching velocity is within
// the velocity limit.
// Searches for an acceleration switching point along segment boundaries.
//
// When forward integration hits the acceleration limit curve, we must find a point where
// backward integration can begin. This occurs at curvature discontinuities (segment boundaries)
// where the acceleration limit curve has a discontinuous change.
//
// At each segment boundary, we:
// 1. Sample geometry from both adjacent segments at the exact boundary arc length
// 2. Compute acceleration limit curve on both sides to find the switching velocity (minimum of the two)
// 3. Validate that this boundary satisfies the switching point conditions
//
// Returns the first valid switching point found, or the end of the trajectory if none is found.
//
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] trajectory::phase_point find_acceleration_switching_point(path::cursor cursor, const trajectory::options& opt) {
    // Walk forward through segments, checking interior extrema first, then boundaries

    // Note: We search for VII-A Case 2 before Case 1 because Case 1 occurs when we switch between a circular segment
    // and a straight line segment while Case 2 manifests on the interior of a circular segment. Therefore if there exists a
    // Case 2 switching point it will be fulfilled before Case 1 and the earlier switching point is the one we wish to use.
    while (true) {
        auto current_segment = *cursor;

        // Examine the current segment for switching points where the path velocity limit curve, s_dot_max_acc(s),
        // is continuous, but not differentiable, per VII-A Case 2 in the paper (Kunz & Stilman equation 39). These
        // switching points occur on the interior of circular segments for joint extrema where f'_i(s) = 0.
        //
        // TODO(RSDK-12979): The paper says that at these points, we must use zero acceleration, but we do not return.
        // that information in any meaningful way. Currently, a caller who obtains this type of switching point will
        // erroneously use the local max acceleration, which would lead into the infeasible region.
        if (current_segment.is<path::segment::circular>()) {
            std::optional<arc_length> first_extremum;
            arc_velocity first_extremum_velocity{0.0};

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

                            const arc_length s_local{radius * angle};
                            const auto s_global = current_segment.start() + s_local;

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
                    if (first_extremum) {
                        // Query geometry at the extremum
                        const auto q_prime = current_segment.tangent(*first_extremum);
                        const auto q_double_prime = current_segment.curvature(*first_extremum);

                        // Compute acceleration limit curve at extremum
                        const auto [s_dot_max_acc, s_dot_max_vel] =
                            compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

                        // If this switching point is infeasible with respect to the velocity limit curve,
                        // reject it.
                        if (opt.epsilon.wrap(s_dot_max_vel) < opt.epsilon.wrap(s_dot_max_acc)) {
                            first_extremum.reset();
                            return;
                        }

                        // Validate: acceleration limit curve must have local minimum (derivative changes negative to positive)
                        // Sample slightly before and after the extremum, clamped to segment bounds
                        //
                        // TODO(RSDK-12850): Replace numerical derivative check with analytical derivative computation.
                        // For circular segments, we can derive the exact formula for d/ds s_dot_max_acc(s) from
                        // equations 29-31 in the paper. The current epsilon-offset approach works but is less
                        // accurate and has epsilon dependency. An analytical solution would be more robust and
                        // faster (no need for 4 extra geometry queries + limit calculations per extremum).
                        const auto before_extremum = std::max(*first_extremum - arc_length{opt.epsilon}, current_segment.start());
                        const auto after_extremum = std::min(*first_extremum + arc_length{opt.epsilon}, current_segment.end());

                        // Skip validation if extremum is too close to segment boundaries (can't get clean epsilon separation)
                        const auto half_epsilon = opt.epsilon * 0.5;
                        const auto too_close_to_start = (half_epsilon.wrap(*first_extremum) == half_epsilon.wrap(before_extremum));
                        const auto too_close_to_end = (half_epsilon.wrap(after_extremum) == half_epsilon.wrap(*first_extremum));

                        if (too_close_to_start || too_close_to_end) {
                            first_extremum.reset();
                            return;
                        }

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
            });

            // If we found a valid extremum switching point, return it
            if (first_extremum) {
                return trajectory::phase_point{.s = *first_extremum, .s_dot = first_extremum_velocity};
            }
        }

        // Examine the transition from a circular segment into a straight line segment where
        // the path velocity limit curve is discontinuous if and only if the path curvature
        // VII-A Case 1 : f''(s), is discontinuous, per Kunz & Stilman equation 38, with `Correction 4`.

        const auto boundary = current_segment.end();

        // If at path end, it is impossible to compare against a subsequent segment since there is none
        if (boundary >= cursor.path().length()) {
            break;
        }

        // Sample geometry BEFORE we cross the boundary
        const auto q_prime_before = current_segment.tangent(boundary);
        const auto q_double_prime_before = current_segment.curvature(boundary);

        // Move cursor to next segment
        cursor.seek(boundary);
        auto segment_after = *cursor;

        // Sample geometry AFTER we cross the boundary
        const auto q_prime_after = segment_after.tangent(boundary);
        const auto q_double_prime_after = segment_after.curvature(boundary);

        // Compute s_dot_max_acc on both sides
        const auto [s_dot_max_acc_before, s_dot_max_vel_before] =
            compute_velocity_limits(q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        const auto [s_dot_max_acc_after, s_dot_max_vel_after] =
            compute_velocity_limits(q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // Apply Kunz & Stilman equation 38
        // A discontinuity of s_dot_max_acc(s) is a switching point if and only if:
        // Case A: [s_dot_max_acc(s-) < s_dot_max_acc(s+) ∧ (s_ddot_max(s-, s_dot_max_acc(s-))/s_dot_max_acc(s-)) >= d/ds s_dot_max_acc(s-)]
        // Case B: [s_dot_max_acc(s-) > s_dot_max_acc(s+) ∧ (s_ddot_max(s+, s_dot_max_acc(s+))/s_dot_max_acc(s+)) <= d/ds s_dot_max_acc(s+)]

        bool is_switching_point = false;

        // Case A: Positive step (limit increases)
        if (opt.epsilon.wrap(s_dot_max_acc_after) > opt.epsilon.wrap(s_dot_max_acc_before)) {
            const auto before_boundary = std::max(boundary - arc_length{opt.epsilon}, current_segment.start());
            const auto actual_step_left = boundary - before_boundary;
            if (actual_step_left < opt.epsilon * 0.5) {
                continue;
            }
            const auto q_prime_bb = current_segment.tangent(before_boundary);
            const auto q_double_prime_bb = current_segment.curvature(before_boundary);
            const auto [s_dot_max_acc_bb, _5] =
                compute_velocity_limits(q_prime_bb, q_double_prime_bb, opt.max_velocity, opt.max_acceleration, opt.epsilon);
            // This is d/ds s_dot_max_acc(s-)
            const auto slope_left = (s_dot_max_acc_before - s_dot_max_acc_bb) / actual_step_left;

            // Evaluate s_ddot_min at (s-, s_dot_max_acc(s-))
            //
            // NOTE: `Correction 7`: For this to be a sink, ALL feasible accelerations lead into the
            // infeasible region, so we check against the MIN, not the max.
            const auto [s_ddot_min_at_s_minus, _2] = compute_acceleration_bounds(q_prime_before,
                                                                                 q_double_prime_before,
                                                                                 s_dot_max_acc_before,  // Evaluate at s-'s own limit
                                                                                 opt.max_acceleration,
                                                                                 opt.epsilon);

            is_switching_point = opt.epsilon.wrap(s_ddot_min_at_s_minus / s_dot_max_acc_before) >= opt.epsilon.wrap(slope_left);
        }
        // Case B: Negative step (limit decreases)
        else if (opt.epsilon.wrap(s_dot_max_acc_after) < opt.epsilon.wrap(s_dot_max_acc_before)) {
            // Compute limit curve slopes using numerical approximation
            // TODO(RSDK-12850): Investigate computing numerical derivatives versus epsilon
            // TODO(RSDK-12851): Decide if it is safe to move onto the next switching point
            const auto after_boundary = std::min(boundary + arc_length{opt.epsilon}, segment_after.end());
            const auto actual_step_right = after_boundary - boundary;
            if (actual_step_right < opt.epsilon * 0.5) {
                continue;
            }
            const auto q_prime_ab = segment_after.tangent(after_boundary);
            const auto q_double_prime_ab = segment_after.curvature(after_boundary);
            const auto [s_dot_max_acc_ab, _6] =
                compute_velocity_limits(q_prime_ab, q_double_prime_ab, opt.max_velocity, opt.max_acceleration, opt.epsilon);
            // This is d/ds s_dot_max_acc(s+)
            const auto slope_right = (s_dot_max_acc_ab - s_dot_max_acc_after) / actual_step_right;

            // Evaluate s_ddot_max at (s+, s_dot_max_acc(s+))
            const auto [_1, s_ddot_max_at_s_plus] = compute_acceleration_bounds(q_prime_after,
                                                                                q_double_prime_after,
                                                                                s_dot_max_acc_after,  // Evaluate at s+'s own limit
                                                                                opt.max_acceleration,
                                                                                opt.epsilon);

            is_switching_point = opt.epsilon.wrap(s_ddot_max_at_s_plus / s_dot_max_acc_after) <= opt.epsilon.wrap(slope_right);
        } else {
            // Should s_dot_max_acc_before and s_dot_max_acc_after be within epsilon of each other, then
            // there is no perceived discontinuity. Therefore, equation 38 cannot be fulfilled and we
            // proceed onwards.
            continue;
        }

        if (is_switching_point) {
            // Switching velocity is the minimum (where the discontinuity meets the feasible region)
            const auto s_dot_max_acc_switching_min = std::min(s_dot_max_acc_before, s_dot_max_acc_after);

            // But make sure we don't violate whichever side's velocity limit is constraining.
            const auto s_dot_max_vel_switching_min = std::min(s_dot_max_vel_before, s_dot_max_vel_after);

            // Only accept this as switching point if it is also feasible with respect to the velocity limits.
            if (opt.epsilon.wrap(s_dot_max_acc_switching_min) <= opt.epsilon.wrap(s_dot_max_vel_switching_min)) {
                return trajectory::phase_point{.s = boundary, .s_dot = s_dot_max_acc_switching_min};
            }
        }
    }

    // No valid switching point found before path end - return end of path as switching point
    return trajectory::phase_point{.s = cursor.path().length(), .s_dot = arc_velocity{0.0}};
}

// Selects between two switching points: returns the earlier one, or if at the same location
// (within epsilon), returns the one with lower velocity (more conservative).
[[gnu::pure]] trajectory::phase_point select_switching_point(const trajectory::phase_point& sp1,
                                                             const trajectory::phase_point& sp2,
                                                             class epsilon epsilon) {
    if (epsilon.wrap(sp1.s) == epsilon.wrap(sp2.s)) {
        // Same location (within epsilon) - use whichever has lower velocity (more conservative)
        return (sp1.s_dot < sp2.s_dot) ? sp1 : sp2;
    }

    // Return whichever comes first along the path
    return (sp1.s < sp2.s) ? sp1 : sp2;
}

// Searches for a velocity switching point where escape from velocity curve becomes possible.
// Implements both continuous (Kunz & Stilman equation 40, with `Correction 5`) and discontinuous
// (Kunz & Stilman equations 41-42, with `Correction 6`) cases from Section VII-B.
//
// Returns the switching point on velocity curve. If no escape point found, returns end of path.
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] trajectory::phase_point find_velocity_switching_point(path::cursor cursor, const trajectory::options& opt) {
    const auto path_length = cursor.path().length();

    std::optional<trajectory::phase_point> discontinuous_switching_point;
    std::optional<trajectory::phase_point> continuous_switching_point;

    // Phase 1: Check for discontinuous switching points at segment boundaries (Kunz & Stilman equations 41-42).
    // Section VII-B case 2: f''_i(s) being discontinuous is a necessary condition for
    // s_ddot_min(s, s_dot_max_vel(s)) being discontinuous.
    // We need to check ALL boundaries to find the earliest one, not return immediately.
    auto boundary_cursor = cursor;
    while (boundary_cursor.position() < path_length) {
        auto current_segment = *boundary_cursor;
        const auto boundary = current_segment.end();

        // Make sure we are not at the end of the path
        if (boundary >= path_length) {
            break;
        }

        // Only check boundaries ahead of our starting position
        if (boundary <= cursor.position()) {
            // This boundary is behind our search position, skip to next segment
            if (boundary < cursor.path().length()) {
                boundary_cursor.seek(boundary);  // Move to next segment
                // The *boundary_cursor dereference will now give the next segment
                continue;
            }
            break;  // We've reached the end
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
        const auto [s_dot_max_accel_before, s_dot_max_vel_before] =
            compute_velocity_limits(q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        const auto [s_dot_max_accel_after, s_dot_max_vel_after] =
            compute_velocity_limits(q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // If velocity limit is degenerate (near zero) on either side, this is a switching point where we must come to rest
        if (abs(s_dot_max_vel_before) < opt.epsilon || abs(s_dot_max_vel_after) < opt.epsilon) {
            // Must stop at this boundary (or nearly so)
            const auto switching_velocity = std::min(std::min(s_dot_max_vel_before, s_dot_max_vel_after), arc_velocity{0.0});
            discontinuous_switching_point = trajectory::phase_point{.s = boundary, .s_dot = switching_velocity};
            break;  // This is the first (most constraining) discontinuous point
        }

        // We can't call `compute_acceleration_bounds` unless this holds; this is a `Divergence 2`
        // type check.
        if (opt.epsilon.wrap(s_dot_max_accel_before) <= opt.epsilon.wrap(s_dot_max_vel_before)) {
            continue;
        }

        // We can't call `compute_acceleration_bounds` unless this holds; this is a `Divergence 2`
        // type check.
        if (opt.epsilon.wrap(s_dot_max_accel_after) <= opt.epsilon.wrap(s_dot_max_vel_after)) {
            continue;
        }

        // TODO(RSDK-12847): Investigate the correctness of Kunz & Stilman equations 41 and 42; document any findings.
        const auto curve_slope_before =
            compute_velocity_limit_derivative(q_prime_before, q_double_prime_before, opt.max_velocity, opt.epsilon);

        const auto curve_slope_after =
            compute_velocity_limit_derivative(q_prime_after, q_double_prime_after, opt.max_velocity, opt.epsilon);

        // Compute minimum accelerations at velocity limits on both sides
        const auto [s_ddot_min_before, s_ddot_max_before] =
            compute_acceleration_bounds(q_prime_before, q_double_prime_before, s_dot_max_vel_before, opt.max_acceleration, opt.epsilon);

        const auto [s_ddot_min_after, s_ddot_max_after] =
            compute_acceleration_bounds(q_prime_after, q_double_prime_after, s_dot_max_vel_after, opt.max_acceleration, opt.epsilon);

        // Check Kunz & Stilman equations 41 and 42:
        // (s_ddot_min(s-, s_dot_max_vel(s-)) / s_dot >= d/ds s_dot_max_vel(s-))
        // AND
        // (s_ddot_min(s+, s_dot_max_vel(s+)) / s_dot <= d/ds s_dot_max_vel(s+))
        //
        // Per the above noted `Correction 6` the LHS of the equations needed a s_dot denominator
        // since without it the units are m/s^2, while the RHS units are 1/s.
        //
        // The following two variables are the LHS values for equations 41 and 42.
        const auto trajectory_slope_before = s_ddot_min_before / s_dot_max_vel_before;
        const auto trajectory_slope_after = s_ddot_min_after / s_dot_max_vel_after;

        const bool condition_41 = (opt.epsilon.wrap(trajectory_slope_before) >= opt.epsilon.wrap(curve_slope_before));
        const bool condition_42 = (opt.epsilon.wrap(trajectory_slope_after) <= opt.epsilon.wrap(curve_slope_after));

        if (condition_41 && condition_42) {
            // Valid discontinuous switching point found
            // Switching velocity is the minimum of velocity limits on both sides
            // TODO(RSDK-12848): Investigate the correctness of taking the minimum of these two switching points.
            const auto s_dot_max_vel_switching_min = std::min(s_dot_max_vel_before, s_dot_max_vel_after);
            const auto s_dot_max_accel_switching_min = std::min(s_dot_max_accel_before, s_dot_max_accel_after);

            // Only accept the velocity switching point if it is feasible with respect to the acceleration
            // limit curve.
            if (opt.epsilon.wrap(s_dot_max_accel_switching_min) >= opt.epsilon.wrap(s_dot_max_vel_switching_min)) {
                // Record first discontinuous switching point, but don't return yet - need to check continuous cases too
                discontinuous_switching_point = trajectory::phase_point{.s = boundary, .s_dot = s_dot_max_vel_switching_min};
                break;  // First discontinuous point found, can stop searching for more discontinuous points
            }
        }
    }

    // TODO(RSDK-12980): The following multi-phase search isn't quite correct, because if we find a velocity
    // switching point but reject it as infeasible (e.g. the acceleration limit curve dominates there), we
    // don't keep looking.

    // Phase 2: Coarse forward search for continuous escape condition (Kunz & Stilman equation 40).
    // Walk along the velocity limit curve until we find a point where s_ddot_min/s_dot <= curve_slope,
    // indicating the trajectory can drop below the curve and resume normal acceleration.
    // If we already found a discontinuous switching point, we can bound the search to stop there.
    std::optional<arc_length> escape_region_start;
    auto previous_position = cursor.position();
    const auto search_limit = discontinuous_switching_point.has_value() ? discontinuous_switching_point->s : path_length;

    auto search_cursor = cursor;  // this creates a copy of cursor but preserves the pattern above of having a `boundary_cursor`
    while (search_cursor.position() < search_limit) {
        // Advance cursor by step size.
        //
        // NOTE: The use of `opt.delta` here is definitely cheating:
        // it is a duration, not a distance, hence the need to convert
        // to arc_length. However, we haven't found a better way to
        // identify what the right space delta is. Since `opt.delta`
        // is at least configurable, this doesn't seem to unreasonable
        // as a starting point.
        const auto next_position = search_cursor.position() + arc_length{opt.delta.count()};
        if (next_position > search_limit) {
            break;  // Exceeded search limit without finding escape
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

        // We can't call `compute_acceleration_bounds` unless this holds. Effectively, this is an
        // instance of `Divergence 2` type behavior.
        if (opt.epsilon.wrap(s_dot_max_acc) < opt.epsilon.wrap(s_dot_max_vel)) {
            previous_position = search_cursor.position();
            continue;
        }

        const auto curve_slope = compute_velocity_limit_derivative(q_prime, q_double_prime, opt.max_velocity, opt.epsilon);

        // Compute minimum acceleration at velocity limit (equation 40 condition)
        const auto [s_ddot_min, s_ddot_max] =
            compute_acceleration_bounds(q_prime, q_double_prime, s_dot_max_vel, opt.max_acceleration, opt.epsilon);

        // Escape condition: trajectory slope (s_ddot_min / s_dot) equal to curve slope.
        // This means applying minimum acceleration would cause trajectory to drop below velocity limit.
        const auto trajectory_slope = s_ddot_min / s_dot_max_vel;

        if (opt.epsilon.wrap(trajectory_slope) == opt.epsilon.wrap(curve_slope)) {
            // Found escape region - record where we first detected it
            escape_region_start = search_cursor.position();
            break;
        }

        // Update previous position for next iteration
        previous_position = search_cursor.position();
    }

    // Phase 3: Bisection refinement to find exact continuous switching point (if escape region found).
    if (escape_region_start) {
        // We know escape is possible somewhere in the last step [previous_position, escape_region_start].
        // Bisect over this interval to find the exact location where the escape condition becomes true.
        auto before = previous_position;
        auto after = *escape_region_start;

        // TODO(RSDK-12767): Eleminiate this hardcoded constant.
        constexpr int max_bisection_iterations = 100;
        for (int iteration = 0; iteration < max_bisection_iterations; ++iteration) {
            // Check convergence
            if (opt.epsilon.wrap(before) == opt.epsilon.wrap(after)) {
                break;
            }

            // Evaluate midpoint
            const auto mid = before + ((after - before) / 2.0);
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

            // This is an instance of `Divergence 2` type behavior, where we are eliminating
            // switching points that impossible w.r.t. the other curve.
            //
            // TODO(RSDK-12980): This also seems not quite right. How do we know which side of the bisection
            // to pursue when the limit curves can swap around like this.
            if (opt.epsilon.wrap(s_dot_max_acc) < opt.epsilon.wrap(s_dot_max_vel)) {
                before = mid;
                continue;
            }

            const auto curve_slope = compute_velocity_limit_derivative(q_prime, q_double_prime, opt.max_velocity, opt.epsilon);

            const auto [s_ddot_min, s_ddot_max] =
                compute_acceleration_bounds(q_prime, q_double_prime, s_dot_max_vel, opt.max_acceleration, opt.epsilon);

            const auto trajectory_slope = s_ddot_min / s_dot_max_vel;

            if (opt.epsilon.wrap(trajectory_slope) == opt.epsilon.wrap(curve_slope)) {
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

        // `Reject switching points where s_dot_max_vel > s_dot_max_acc.
        //
        // If the velocity switching point has s_dot > s_dot_max_acc, backward integration will immediately
        // hit the acceleration limit curve, causing the algorithm to fail. We should continue searching
        // forward until finding a switching point where s_dot_max_vel <= s_dot_max_acc + epsilon.
        if (opt.epsilon.wrap(s_dot_max_vel) <= opt.epsilon.wrap(s_dot_max_acc)) {
            continuous_switching_point = trajectory::phase_point{.s = after, .s_dot = s_dot_max_vel};
        }
    }

    // Phase 4: Return whichever switching point comes first.
    // If both exist, return the one with smaller s (earlier along the path).
    // If only one exists, return that one.
    // If neither exists, return end of path.
    if (discontinuous_switching_point && continuous_switching_point) {
        return select_switching_point(*discontinuous_switching_point, *continuous_switching_point, opt.epsilon);
    }

    if (discontinuous_switching_point) {
        return *discontinuous_switching_point;
    }

    if (continuous_switching_point) {
        return *continuous_switching_point;
    }

    // No switching point found - return end of path
    return trajectory::phase_point{.s = path_length, .s_dot = arc_velocity{0.0}};
}

// Unified switching point search that calls both acceleration and velocity searches.
// Per the paper, we must search for BOTH types of switching points
// and return whichever comes first along the path. The curves can cross, so even if we hit the
// acceleration curve, the velocity switching point might come before the acceleration switching point.
//
// When both switching points are at the same location (within epsilon), we return whichever has
// lower velocity, as this is more conservative and less likely to cause immediate limit curve hit
// during backward integration.
//
// TODO(RSDK-12760): Performance optimization - bound velocity search by acceleration result.
//
// TODO(RSDK-12819): It is not entirely clear that separately
// searching for (filtered) acceleration switching points and then
// separately searching for (filtered) velocity switching points and
// taking the earlier one is entirely equivalent to searching once on
// the combined limit curve, since there could be curve crossings,
// which represent discontinuities on the limit curve. How should we
// handle those?
[[gnu::pure]] trajectory::phase_point find_switching_point(path::cursor cursor, const trajectory::options& opt) {
    // Always search for both types of switching points
    auto accel_sp = find_acceleration_switching_point(cursor, opt);
    auto vel_sp = find_velocity_switching_point(cursor, opt);

    return select_switching_point(accel_sp, vel_sp, opt.epsilon);
}

}  // namespace

trajectory::integration_observer::integration_observer() = default;
trajectory::integration_observer::~integration_observer() = default;

trajectory::integration_event_observer::integration_event_observer() = default;
trajectory::integration_event_observer::~integration_event_observer() = default;

void trajectory::integration_event_observer::on_started_forward_integration(const trajectory& traj, started_forward_event event) {
    on_event(traj, std::move(event));
}

void trajectory::integration_event_observer::on_hit_limit_curve(const trajectory& traj, limit_hit_event event) {
    on_event(traj, std::move(event));
}

void trajectory::integration_event_observer::on_started_backward_integration(const trajectory& traj, started_backward_event event) {
    on_event(traj, std::move(event));
}

void trajectory::integration_event_observer::on_trajectory_extended(const trajectory& traj, splice_event event) {
    on_event(traj, std::move(event));
}

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
    integration_points_.push_back(
        {.time = seconds{0.0}, .s = arc_length{0.0}, .s_dot = arc_velocity{0.0}, .s_ddot = arc_acceleration{0.0}});
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
        const auto find_trajectory_intersection = [&](arc_length backward_s, arc_velocity backward_s_dot) -> std::optional<size_t> {
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
                if (traj.options_.epsilon.wrap(backward_s_dot) >= traj.options_.epsilon.wrap(pt0.s_dot)) {
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
            const auto s_range = pt1.s - pt0.s;
            if (s_range < traj.options_.epsilon) [[unlikely]] {
                // Points are too close - compare directly with pt0
                if (traj.options_.epsilon.wrap(backward_s_dot) >= traj.options_.epsilon.wrap(pt0.s_dot)) {
                    return std::distance(forward_points.begin(), pt0_it);
                }
                return std::nullopt;
            }

            const auto s_offset = backward_s - pt0.s;
            const auto interpolation_factor = s_offset / s_range;
            const auto forward_s_dot_interp = pt0.s_dot + (interpolation_factor * (pt1.s_dot - pt0.s_dot));

            // Intersection occurs if backward's s_dot exceeds forward's interpolated s_dot.
            // Backward integration starts with low s_dot and increases as s decreases, eventually
            // crossing above the forward trajectory's s_dot at the same s position.
            if (traj.options_.epsilon.wrap(backward_s_dot) >= traj.options_.epsilon.wrap(forward_s_dot_interp)) {
                // Intersection found - return index of pt0 (start of bracketing interval)
                return std::distance(forward_points.begin(), pt0_it);
            }

            return std::nullopt;
        };

        // State transition function - validates and performs state transitions based on events
        const auto transition = [](integration_state current_state, integration_event event) -> integration_state {
            // Handle self-transition uniformly for all states
            if (event == integration_event::k_none) [[likely]] {
                return current_state;
            }

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
        integration_event event = integration_event::k_none;

        // Create path cursor for querying geometry during integration
        path::cursor path_cursor = traj.path_.create_cursor();

        // Scratch buffer for backward integration points. Backward integration builds a trajectory
        // moving from switching point toward start. Points are accumulated here, then reversed and
        // spliced into the main trajectory when intersection with forward trajectory is found.
        //
        // TODO(RSDK-12769): Revisit how state is passed between integration states. Consider using stateful
        // events that carry context, rather than state machine-scoped variables.
        std::vector<integration_point> backward_points;

        // Observer latch for detecting state transitions. When a state change is detected,
        // this latch gets armed with the observer pointer. Each state case checks and clears
        // the latch on entry using std::exchange, calling the observer only on actual transitions.
        // Initialize with the observer so the initial state entry is reported.
        auto* observer_latch = traj.options_.observer;

        // Integration loop runs until we've covered the entire path.
        // Loop exits when last integration point reaches path end exactly.
        while (traj.integration_points_.back().s != traj.path_.length()) {
            switch (state) {
                case integration_state::k_forward_accelerating: {
                    // On entry to this state (detected via observer latch), notify observer
                    if (auto* observer = std::exchange(observer_latch, nullptr)) {
                        const auto& current_point = traj.integration_points_.back();
                        observer->on_started_forward_integration(traj, {.start = {current_point.s, current_point.s_dot}});
                    }

                    // Starting point is the last integration point (known to be feasible)
                    const auto& current_point = traj.integration_points_.back();

                    // Position cursor at current integration point.  This ensures each state is self-contained and
                    // doesn't depend on cursor position from previous state.
                    path_cursor.seek(current_point.s);

                    // Query path geometry at current position to compute maximum acceleration
                    const auto q_prime = path_cursor.tangent();
                    const auto q_double_prime = path_cursor.curvature();

                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, current_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    // Compute candidate next point via Euler integration with maximum acceleration
                    const auto [next_s, next_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_max, traj.options_.delta, traj.options_.epsilon);

                    // Forward integration should move "up and to the right" in phase plane
                    if ((next_s <= current_point.s) || (next_s_dot < current_point.s_dot)) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: forward integration must increase both s and s_dot"};
                    }

                    // Check if this step would take us past the end of the path
                    if (next_s >= traj.path_.length()) {
                        // Reached end - this is a switching point at rest. This becomes the initial point
                        // for backward integration to correct the over-optimistic forward trajectory.
                        integration_point switching_point{.time = current_point.time + traj.options_.delta,
                                                          .s = traj.path_.length(),
                                                          .s_dot = arc_velocity{0.0},
                                                          .s_ddot = arc_acceleration{0.0}};

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

                    if (s_dot_limit <= arc_velocity{0.0}) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive"};
                    }

                    // Check if the candidate point exceeds the limit curve
                    if (traj.options_.epsilon.wrap(next_s_dot) > traj.options_.epsilon.wrap(s_dot_limit)) {
                        // Candidate point would hit or exceed limit curve.
                        //
                        // TODO(RSDK-12708,RSDK-12709): For robustness (Section VIII-A of paper), we should:
                        // 1. Use bisection to find exact intersection with limit curve
                        // 2. Check if intersection point is a trajectory source or sink
                        // 3. Only transition if it's a sink (not a source due to numerical overshoot)
                        // For now, we transition immediately, which may be less robust with large dt.
                        //
                        // Note: The phase_point passed to observer is the INFEASIBLE candidate point
                        // that exceeded limits, not a feasible point on the limit curve.

                        if (traj.options_.observer) {
                            traj.options_.observer->on_hit_limit_curve(
                                traj, {.breach = {next_s, next_s_dot}, .s_dot_max_acc = s_dot_max_acc, .s_dot_max_vel = s_dot_max_vel});
                        }

                        // Determine which limit curve we hit by comparing the two curves.
                        // The lower curve is the active constraint at this position.
                        //
                        // TODO(RSDK-12762): When curves are within close to each other, it's arbitrary whether we treat
                        // this as hitting the velocity curve (and potentially following it) or the acceleration curve
                        // (and searching for a switching point). Current choice: if curves are within epsilon, treat as
                        // acceleration curve hit (search for switching point). This is conservative in the sense that
                        // searching is always safe, while curve following requires the curve to be
                        // well-defined. However, investigate whether there's a better heuristic (e.g., based on curve
                        // derivatives or previous state).
                        if ((traj.options_.epsilon.wrap(s_dot_max_acc) > traj.options_.epsilon.wrap(s_dot_max_vel))) {
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
                                             .s_ddot = arc_acceleration{0.0}};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Candidate point is feasible - accept it and advance. Observe the tricky behavior around the
                    // acceleration.  We need to update the acceleration of the current point: that's the acceleration
                    // we selected at our current location in the phase plane in order to get to the next point. That
                    // new point now has an indeterminate acceleration, which will not be computed until the next point,
                    // etc.
                    //
                    // TODO(RSDK-13002): Recording zero for the s_ddot value isn't great. We should find a
                    // better way so that invalid integration points are never present in the integration
                    // points vector, or, if they are, they shouldn't have a potentially reasonable value like
                    // 0.0.
                    //
                    // Note: Don't call maybe_on_trajectory_extended() here - forward points are tentative
                    // and may be discarded during backward integration. Only finalized segments trigger that callback.
                    integration_point next_point{.time = current_point.time + traj.options_.delta,
                                                 .s = next_s,
                                                 .s_dot = next_s_dot,
                                                 .s_ddot = arc_acceleration{0.0}};
                    traj.integration_points_.back().s_ddot = s_ddot_max;
                    traj.integration_points_.push_back(std::move(next_point));

                    // Continue in same state (no event)
                    // Cursor will be repositioned on next iteration entry
                    break;
                }

                case integration_state::k_forward_following_velocity_curve: {
                    // On entry to this state (detected via observer latch), notify observer we're
                    // starting forward integration along the velocity limit curve.
                    if (auto* observer = std::exchange(observer_latch, nullptr)) {
                        const auto& current_point = traj.integration_points_.back();
                        observer->on_started_forward_integration(traj, {.start = {current_point.s, current_point.s_dot}});
                    }

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

                    // TODO(RSDK-12763): Revisit whether we need epsilon checks before dividing by s_dot_max_vel here. During forward
                    // integration following the velocity curve, s_dot_max_vel should always be positive. However, if the
                    // velocity limit itself becomes very small (e.g., tight curvature with low velocity limits), the division
                    // could become numerically unstable. For now, throw if s_dot_max_vel is near zero as a canary.
                    if (s_dot_max_vel < traj.options_.epsilon) [[unlikely]] {
                        // TODO(RSDK-12768): Investigate whether we want a richer exception type for algorithm failures.
                        // A custom exception hierarchy could distinguish between different failure modes
                        // (infeasible path, numerical issues, constraint violations) for better error handling.
                        throw std::runtime_error{
                            "TOTG algorithm error: cannot evaluate curve exit conditions with near-zero velocity limit"};
                    }

                    // Check if acceleration limit curve has dropped below velocity limit curve.
                    // While following the velocity curve, the acceleration curve can drop below us, meaning
                    // the acceleration constraint becomes the active limit. This is symmetric to the check in
                    // k_forward_accelerating that detects hitting the velocity curve from below.
                    if (traj.options_.epsilon.wrap(s_dot_max_acc) < traj.options_.epsilon.wrap(s_dot_max_vel)) {
                        if (traj.options_.observer) {
                            traj.options_.observer->on_hit_limit_curve(traj,
                                                                       {.breach = {current_point.s, current_point.s_dot},
                                                                        .s_dot_max_acc = s_dot_max_acc,
                                                                        .s_dot_max_vel = s_dot_max_vel});
                        }

                        // Acceleration curve is now below velocity curve - we've effectively hit the acceleration
                        // limit curve from above. Need to search for switching point.
                        auto switching_point = find_switching_point(path_cursor, traj.options_);

                        // Initialize backward integration from switching point
                        integration_point sp{.time = current_point.time + traj.options_.delta,  // Placeholder, corrected during splice
                                             .s = switching_point.s,
                                             .s_dot = switching_point.s_dot,
                                             .s_ddot = arc_acceleration{0.0}};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Compute the slope of the velocity limit curve in phase plane. This is d/ds s_dot_max_vel(s),
                    // telling us how the velocity limit changes along the path.
                    //
                    // Selecting the velocity limit derivative here is `Correction 2`.
                    const auto curve_slope =
                        compute_velocity_limit_derivative(q_prime, q_double_prime, traj.options_.max_velocity, traj.options_.epsilon);

                    // Three cases (from Kunz & Stilman Section VI.3):
                    // 1. s_ddot_max / s_dot < curve_slope: Can escape (trajectory curves below limit)
                    // 2. s_ddot_min / s_dot > curve_slope: Must search (trapped on curve)
                    // 3. Otherwise: Follow curve tangentially
                    //
                    // Note: We use s_dot_max_vel in the slope calculations per Algorithm Step 3.

                    // Compute feasible acceleration bounds at the velocity limit per paper Algorithm Step 3.
                    // The paper specifies evaluating s_ddot_min/max at s_dot_max_vel(s), not at current trajectory velocity.
                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, s_dot_max_vel, traj.options_.max_acceleration, traj.options_.epsilon);

                    if (traj.options_.epsilon.wrap(s_ddot_max / current_point.s_dot) < traj.options_.epsilon.wrap(curve_slope)) {
                        // Maximum acceleration trajectory curves away from the limit curve (less steeply upward).
                        // The trajectory's slope (s_ddot_max / s_dot) is less than the limit curve's slope,
                        // meaning we're moving upward more slowly than the curve, effectively dropping below it.
                        // We can escape the curve and resume normal maximum acceleration integration.
                        event = integration_event::k_escaped_limit_curve;
                        break;
                    }

                    if (traj.options_.epsilon.wrap(s_ddot_min / current_point.s_dot) > traj.options_.epsilon.wrap(curve_slope)) {
                        // Trapped on limit curve with no way to escape via normal acceleration.
                        // Search forward for a switching point where backward integration can begin.
                        // The unified search checks for both acceleration discontinuities and velocity
                        // escape points, returning whichever comes first.
                        auto switching_point = find_switching_point(path_cursor, traj.options_);

                        // Initialize backward integration from switching point
                        integration_point sp{.time = current_point.time + traj.options_.delta,  // Placeholder, corrected during splice
                                             .s = switching_point.s,
                                             .s_dot = switching_point.s_dot,
                                             .s_ddot = arc_acceleration{0.0}};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Neither escape nor search conditions met - follow the limit curve tangentially.
                    // Compute the tangent acceleration: the rate of change that keeps us on the curve.
                    const auto s_ddot_curve = curve_slope * s_dot_max_vel;

                    // Validate that tangent acceleration is within feasible bounds.
                    // If the curve's tangent falls outside our acceleration capabilities, the trajectory
                    // is infeasible at this point (algorithm error - shouldn't reach here).
                    // TODO(RSDK-12768): Investigate whether we want a richer exception type for infeasible trajectories.
                    // Could provide diagnostic information about which constraints failed and where.
                    if (s_ddot_curve < s_ddot_min || s_ddot_curve > s_ddot_max) [[unlikely]] {
                        throw std::runtime_error{
                            "TOTG algorithm error: velocity curve tangent acceleration outside feasible bounds - "
                            "trajectory is infeasible"};
                    }

                    // Integrate forward along the curve with tangent acceleration
                    const auto [next_s, next_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_curve, traj.options_.delta, traj.options_.epsilon);

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
                        integration_point switching_point{.time = current_point.time + traj.options_.delta,
                                                          .s = traj.path_.length(),
                                                          .s_dot = arc_velocity{0.0},
                                                          .s_ddot = arc_acceleration{0.0}};

                        // Initialize backward integration with switching point as starting position
                        backward_points.push_back(std::move(switching_point));

                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Probe the candidate next point to check if it would exceed limit curves.
                    // Create a temporary cursor at the candidate position to query geometry there.
                    //
                    // TODO(RSDK-12994): Looking at the amount of duplication between the forward following
                    // limit case and the forward at max accel case, it is probably possible to unify them.
                    auto probe_cursor = path_cursor;
                    probe_cursor.seek(next_s);
                    const auto probe_q_prime = probe_cursor.tangent();
                    const auto probe_q_double_prime = probe_cursor.curvature();

                    // Check if the candidate next point would violate velocity or acceleration limits
                    const auto [s_dot_max_acc_probe, s_dot_max_vel_probe] = compute_velocity_limits(probe_q_prime,
                                                                                                    probe_q_double_prime,
                                                                                                    traj.options_.max_velocity,
                                                                                                    traj.options_.max_acceleration,
                                                                                                    traj.options_.epsilon);
                    const auto s_dot_limit = std::min(s_dot_max_acc_probe, s_dot_max_vel_probe);

                    if (s_dot_limit <= arc_velocity{0.0}) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive"};
                    }

                    // Check if the candidate point exceeds the limit curve
                    if (traj.options_.epsilon.wrap(next_s_dot) > traj.options_.epsilon.wrap(s_dot_limit)) {
                        // Candidate point would hit or exceed limit curve.
                        //
                        // TODO(RSDK-12708,RSDK-12709): For robustness (Section VIII-A of paper), we should:
                        // 1. Use bisection to find exact intersection with limit curve
                        // 2. Check if intersection point is a trajectory source or sink
                        // 3. Only transition if it's a sink (not a source due to numerical overshoot)
                        // For now, we transition immediately, which may be less robust with large dt.
                        //
                        // Note: The phase_point passed to observer is the INFEASIBLE candidate point
                        // that exceeded limits, not a feasible point on the limit curve.

                        if (traj.options_.observer) {
                            traj.options_.observer->on_hit_limit_curve(traj,
                                                                       {.breach = {next_s, next_s_dot},
                                                                        .s_dot_max_acc = s_dot_max_acc_probe,
                                                                        .s_dot_max_vel = s_dot_max_vel_probe});
                        }

                        // Hit limit curve. Search forward along the path for a switching point where
                        // backward integration can begin. The unified search checks for both acceleration
                        // discontinuities and velocity escape points, returning whichever comes first.
                        auto switching_point = find_switching_point(path_cursor, traj.options_);

                        // Initialize backward integration from switching point
                        integration_point sp{.time = current_point.time + traj.options_.delta,  // Placeholder, corrected during splice
                                             .s = switching_point.s,
                                             .s_dot = switching_point.s_dot,
                                             .s_ddot = arc_acceleration{0.0}};
                        backward_points.push_back(std::move(sp));

                        // Reuse k_reached_end event to transition to backward integration
                        event = integration_event::k_reached_end;
                        break;
                    }

                    // Accept the point and continue following the curve.
                    // The next iteration will re-evaluate whether we're still on the curve or can escape.
                    //
                    // NOTE: As above, we set the acceleration of the previous point here,
                    // not the acceleration of the point we just created.
                    integration_point next_point{.time = current_point.time + traj.options_.delta,
                                                 .s = next_s,
                                                 .s_dot = next_s_dot,
                                                 .s_ddot = arc_acceleration{0.0}};
                    traj.integration_points_.back().s_ddot = s_ddot_curve;
                    traj.integration_points_.push_back(std::move(next_point));

                    // No event - remain in same state and re-evaluate on next iteration
                    // The loop will check exit conditions again with updated position
                    break;
                }

                case integration_state::k_backward_decelerating: {
                    // Backward integration from a switching point toward the start, building a trajectory
                    // that decelerates to meet constraints. Uses minimum (negative) acceleration to move
                    // "up and left" in phase plane: s decreases, s_dot increases.

                    // On first entry to this state (detected by single point in backward_points),
                    // validate that the switching point is geometrically correct.
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
                    }

                    // On entry to this state (detected via observer latch), notify observer
                    if (auto* observer = std::exchange(observer_latch, nullptr)) {
                        const auto& switching_point = backward_points.back();
                        observer->on_started_backward_integration(traj, {.start = {switching_point.s, switching_point.s_dot}});
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

                    auto s_ddot_to_use = s_ddot_min;

                    // Minimum acceleration must be negative to produce backward motion (decreasing s)
                    // Allow small tolerance for numerical precision at near-zero acceleration points
                    if (s_ddot_to_use >= -traj.options_.epsilon) {
                        if (s_ddot_to_use > traj.options_.epsilon) {
                            // Clearly positive - this is an error
                            throw std::runtime_error{"TOTG algorithm error: backward integration requires negative minimum acceleration"};
                        }
                        // Near zero (degenerate switching point) - use zero acceleration per Kunz & Stilman Section VII-A-2.x`
                        // Forward progress is guaranteed as long as s_dot is non-zero.
                        s_ddot_to_use = arc_acceleration{0.0};
                    }

                    // Compute candidate next point via Euler integration with negative dt and minimum acceleration.
                    // Negative dt reverses time direction, reconstructing velocities that led to current point.
                    // With s_ddot_min < 0 and dt < 0, s_dot increases (up) while s decreases (left).
                    //
                    // TODO(RSDK-12981): There's no guarantee that the candidate we select here by going
                    // backwards with `s_ddot_to_use` as determined at the switching point would then
                    // integrate forwards from the candidate to the switching point.
                    const auto [candidate_s, candidate_s_dot] =
                        euler_step(current_point.s, current_point.s_dot, s_ddot_to_use, -traj.options_.delta, traj.options_.epsilon);

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
                        // Found intersection - the candidate point crossed above the forward trajectory in the phase plane. Note that we
                        // do not include it in the backward trajectory.

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

                        // TODO(RSDK-12982): We also need to 1) adjust the s_ddot of the last forward point,
                        // since it currently points to the old next point, not the new next point from the
                        // intersection with the backwards trajectory. We probably also need to do a better
                        // job adjusting timestamps. There is no guarantee that simply replaying the deltas
                        // like this really makes sense. We should compute some timestep between the last
                        // forward and first from backwards, use that, and then increment all the others by
                        // dt.

                        // Update trajectory duration to reflect new endpoint before invoking callbacks.
                        // Observers receiving the trajectory by reference may query its duration.
                        traj.duration_ = traj.integration_points_.back().time;

                        // Clear backward scratch buffer for potential reuse if algorithm continues
                        backward_points.clear();

                        // Notify observer that trajectory has been extended with finalized backward segment
                        if (traj.options_.observer) {
                            traj.options_.observer->on_trajectory_extended(traj, {});
                        }

                        event = integration_event::k_hit_forward_trajectory;
                        break;
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

                    if (s_dot_limit <= arc_velocity{0.0}) [[unlikely]] {
                        throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive during backward integration"};
                    }

                    // `Divergent Behavior 3`: Candidate exceeding limit curve is an algorithm error - trajectory is
                    // infeasible. Being at the limit (within epsilon) is allowed - only exceeding it is rejected.
                    if (traj.options_.epsilon.wrap(candidate_s_dot) > traj.options_.epsilon.wrap(s_dot_limit)) {
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

            // Processes the current event, applying it to the state machine. Computes state transition,
            // arms observer latch if state changes, updates state, and resets event to k_none.
            if (const auto new_state = transition(state, std::exchange(event, integration_event::k_none)); new_state != state) {
                state = new_state;
                observer_latch = traj.options_.observer;
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
    const auto dt = time_ - p0.time;
    const auto s_dot = p0.s_dot + (p0.s_ddot * dt);
    const auto s_ddot = p0.s_ddot;

    // Query the path geometry at the current arc length position. The path_cursor_ has
    // already been positioned by update_path_cursor_position_ in seek().
    const auto q = path_cursor_.configuration();
    const auto q_prime = path_cursor_.tangent();
    const auto q_double_prime = path_cursor_.curvature();

    // Convert from path space (s, s_dot, s_ddot) to joint space (q, q_dot, q_ddot) using the chain rule.
    //
    // This gives us the actual joint velocities and accelerations that result from
    // moving along the path at the computed path velocity and acceleration.

    // Kunz & Stilman equation 11
    const auto q_dot = q_prime * static_cast<double>(s_dot);

    // The second term captures the centripetal acceleration from
    // following a curved path. We cast s_dot to double here, loosing
    // dimensional safety, because we don't have a strong type for
    // v^2. That's OK, it is fairly clear what is going on.
    const auto s_dot_double = static_cast<double>(s_dot);

    // Kunz & Stilman equation 12
    const auto q_ddot = (q_prime * static_cast<double>(s_ddot)) + q_double_prime * (s_dot_double * s_dot_double);

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
    const auto dt = (t - p0.time);
    const auto s_interp = p0.s + (p0.s_dot * dt) + (0.5 * p0.s_ddot * dt * dt);

    // Clamp interpolated arc length to path bounds to handle floating point accumulation.
    // Even when t <= duration, s_interp can slightly exceed path.length() due to numerical
    // error in the kinematic integration, which would put the path cursor at sentinel.
    const auto s_clamped = std::min(s_interp, traj_->path_.length());

    // Position path cursor at clamped arc length
    path_cursor_.seek(s_clamped);
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
    // TODO(RSDK-12770): Optimize hint update by detecting seek direction (forward vs backward from current
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

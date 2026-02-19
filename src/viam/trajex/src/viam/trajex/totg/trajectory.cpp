#include <viam/trajex/totg/trajectory.hpp>

#include <cassert>
#include <cstddef>
#include <functional>
#include <iterator>
#include <numbers>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>

#include <boost/range/adaptors.hpp>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/private/phase_plane_slope.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/arc_operations.hpp>

namespace viam::trajex::totg {

// Verify that trajectory::sampled satisfies range concepts (using uniform_sampler as concrete type)
static_assert(std::ranges::range<trajectory::sampled<uniform_sampler>>);
static_assert(std::ranges::input_range<trajectory::sampled<uniform_sampler>>);
static_assert(std::ranges::view<trajectory::sampled<uniform_sampler>>);

namespace {

// Internal representation of a switching point.
struct switching_point {
    trajectory::phase_point point;
    trajectory::switching_point_kind kind;
    std::optional<arc_acceleration> forward_accel = std::nullopt;
    std::optional<arc_acceleration> backward_accel = std::nullopt;
};

// Computes maximum path velocity (s_dot) from joint velocity and acceleration limits.
// Two constraints apply: centripetal acceleration from path curvature (eq 31) and
// direct velocity limits (eq 36). Returns both so caller can take the minimum.
// See Kunz & Stilman equations 31 and 36.
[[gnu::pure]] trajectory::velocity_limits compute_velocity_limits(const xt::xarray<double>& q_prime,
                                                                  const xt::xarray<double>& q_double_prime,
                                                                  const xt::xarray<double>& q_dot_max,
                                                                  const xt::xarray<double>& q_ddot_max,
                                                                  class epsilon epsilon) {
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

    return {s_dot_max_accel, s_dot_max_vel};
}

// Computes the feasible range of path acceleration (s_ddot) given current path velocity (s_dot)
// and joint acceleration limits. The path acceleration must satisfy joint constraints in all DOF.
// See Kunz & Stilman equations 22-23.
[[gnu::pure]] trajectory::acceleration_bounds compute_acceleration_bounds(const xt::xarray<double>& q_prime,
                                                                          const xt::xarray<double>& q_double_prime,
                                                                          arc_velocity s_dot,
                                                                          const xt::xarray<double>& q_ddot_max,
                                                                          class epsilon epsilon) {
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

    return {s_ddot_min, s_ddot_max};
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
    // numerically singular case where the joint is barely moving along the path (q' ~= 0) but happens
    // to be the limiting constraint. The derivative d/ds(1/|q'|) becomes huge and ill-defined.
    //
    // Physical interpretation: When |q'| ~= 0, the velocity limit s_dot_max = q_dot_max / |q'| is
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

struct eq40_result {
    phase_plane_slope delta;
    arc_velocity s_dot_max_vel;
};

// Try to evaluate Eq. 40 delta:
//   Delta(s) = s_ddot_min(s, s_dot_max_vel(s)) / s_dot_max_vel(s) - d/ds s_dot_max_vel(s)
// Returns std::nullopt when Eq. 40 is not evaluable at this geometry (degenerate
// velocity limit, acceleration limit below velocity limit, singular derivative,
// or infeasible acceleration bounds).
[[nodiscard]] std::optional<eq40_result> try_compute_eq40_delta(path::cursor& cursor, arc_length s, const trajectory::options& opt) {
    cursor.seek(s);
    const auto q_prime = cursor.tangent();
    const auto q_double_prime = cursor.curvature();

    const auto [s_dot_max_acc, s_dot_max_vel] =
        compute_velocity_limits(q_prime, q_double_prime, opt.max_velocity, opt.max_acceleration, opt.epsilon);

    if (opt.epsilon.wrap(s_dot_max_vel) == opt.epsilon.wrap(arc_velocity{0.0})) {
        return std::nullopt;
    }

    if (opt.epsilon.wrap(s_dot_max_acc) < opt.epsilon.wrap(s_dot_max_vel)) {
        return std::nullopt;
    }

    phase_plane_slope curve_slope{0.0};
    try {
        curve_slope = compute_velocity_limit_derivative(q_prime, q_double_prime, opt.max_velocity, opt.epsilon);
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }

    std::optional<trajectory::acceleration_bounds> accel_bounds;
    try {
        accel_bounds = compute_acceleration_bounds(q_prime, q_double_prime, s_dot_max_vel, opt.max_acceleration, opt.epsilon);
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }

    const auto& [s_ddot_min, s_ddot_max] = *accel_bounds;
    const auto trajectory_slope = s_ddot_min / s_dot_max_vel;
    return eq40_result{.delta = trajectory_slope - curve_slope, .s_dot_max_vel = s_dot_max_vel};
}

// Performs a single Euler integration step in phase plane (s, s_dot).
// Given current position, velocity, and applied acceleration, computes the next state.
// Uses constant acceleration kinematic equations: v_new = v + a*dt, s_new = s + v*dt + 0.5*a*dt^2.
// This is direction-agnostic - caller determines whether dt is positive (forward) or negative (backward).
[[gnu::const]] trajectory::phase_point euler_step(
    arc_length s, arc_velocity s_dot, arc_acceleration s_ddot, trajectory::seconds dt, class epsilon epsilon) {
    const auto s_dot_new = s_dot + (s_ddot * dt);
    const auto s_new = s + (s_dot * dt) + (0.5 * s_ddot * dt * dt);

    if (epsilon.wrap(s_new) == epsilon.wrap(s)) [[unlikely]] {
        if (epsilon.wrap(s_dot_new) == epsilon.wrap(s_dot)) [[unlikely]] {
            throw std::runtime_error{"Euler step will not make sufficient progress in the phase plane"};
        }
    }

    return {s_new, s_dot_new};
}

[[gnu::const]] auto euler_step(trajectory::phase_point where, arc_acceleration s_ddot, trajectory::seconds dt, class epsilon epsilon) {
    return euler_step(where.s, where.s_dot, s_ddot, dt, epsilon);
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
[[gnu::pure]] switching_point find_acceleration_switching_point(path::cursor cursor, const trajectory::options& opt) {
    // Walk forward through segments, checking interior extrema first, then boundaries

    // Note: We search for VII-A Case 2 before Case 1 because Case 1 occurs when we switch between a circular segment
    // and a straight line segment while Case 2 manifests on the interior of a circular segment. Therefore if there exists a
    // Case 2 switching point it will be fulfilled before Case 1 and the earlier switching point is the one we wish to use.
    while (true) {
        auto current_segment = *cursor;

        // Examine the current segment for switching points where the path velocity limit curve, s_dot_max_acc(s),
        // is continuous, but not differentiable, per VII-A Case 2 in the paper (Kunz & Stilman equation 39). These
        // switching points occur on the interior of circular segments for joint extrema where f'_i(s) = 0.
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
                return switching_point{.point = {.s = *first_extremum, .s_dot = first_extremum_velocity},
                                       .kind = trajectory::switching_point_kind::k_nondifferentiable_extremum,
                                       .forward_accel = arc_acceleration{0.0},
                                       .backward_accel = arc_acceleration{0.0}};
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

        // Compute s_dot_max_acc on both sides
        const auto [s_dot_max_acc_before, s_dot_max_vel_before] =
            compute_velocity_limits(q_prime_before, q_double_prime_before, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // Move cursor to next segment
        cursor.seek(boundary);
        auto segment_after = *cursor;

        // Sample geometry AFTER we cross the boundary
        const auto q_prime_after = segment_after.tangent(boundary);
        const auto q_double_prime_after = segment_after.curvature(boundary);

        const auto [s_dot_max_acc_after, s_dot_max_vel_after] =
            compute_velocity_limits(q_prime_after, q_double_prime_after, opt.max_velocity, opt.max_acceleration, opt.epsilon);

        // All computation happens at the smaller one
        const auto s_dot_max_acc_switching_min = std::min(s_dot_max_acc_before, s_dot_max_acc_after);

        // Apply Kunz & Stilman equation 38
        // A discontinuity of s_dot_max_acc(s) is a switching point if and only if:
        // Case A: [s_dot_max_acc(s-) < s_dot_max_acc(s+) and (s_ddot_max(s-, s_dot_max_acc(s-))/s_dot_max_acc(s-)) >= d/ds
        // s_dot_max_acc(s-)] Case B: [s_dot_max_acc(s-) > s_dot_max_acc(s+) and (s_ddot_max(s+, s_dot_max_acc(s+))/s_dot_max_acc(s+)) <=
        // d/ds s_dot_max_acc(s+)]

        bool is_switching_point = false;
        switching_point sp{.point = {.s = boundary, .s_dot = s_dot_max_acc_switching_min},
                           .kind = trajectory::switching_point_kind::k_discontinuous_curvature};

        if (s_dot_max_acc_after > s_dot_max_acc_before) {
            // Case A: Positive step (limit increases)
            const auto before_boundary = std::max(boundary - arc_length{opt.epsilon}, current_segment.start());
            const auto actual_step_left = boundary - before_boundary;
            if (actual_step_left < opt.epsilon * 0.5) {
                continue;
            }
            const auto q_prime_bb = current_segment.tangent(before_boundary);
            const auto q_double_prime_bb = current_segment.curvature(before_boundary);

            const auto [s_dot_max_acc_bb, _1] =
                compute_velocity_limits(q_prime_bb, q_double_prime_bb, opt.max_velocity, opt.max_acceleration, opt.epsilon);
            // This is d/ds s_dot_max_acc(s-)
            const auto slope_left = (s_dot_max_acc_before - s_dot_max_acc_bb) / actual_step_left;

            // Evaluate s_ddot_min at (s-, s_dot_max_acc(s-))
            //
            // NOTE: `Correction 7`: For this to be a sink, ALL feasible accelerations lead into the
            // infeasible region, so we check against the MIN, not the max.

            const auto [s_ddot_min_bb, s_ddot_max_bb] =
                compute_acceleration_bounds(q_prime_bb, q_double_prime_bb, s_dot_max_acc_bb, opt.max_acceleration, opt.epsilon);

            const auto phase_slope_left = s_ddot_min_bb / s_dot_max_acc_bb;
            is_switching_point = phase_slope_left >= slope_left;

            sp.forward_accel = s_ddot_max_bb;
            sp.backward_accel = s_ddot_min_bb;

        } else if (s_dot_max_acc_after < s_dot_max_acc_before) {
            // Case B: Negative step (limit decreases)
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
            const auto [s_dot_max_acc_ab, _1] =
                compute_velocity_limits(q_prime_ab, q_double_prime_ab, opt.max_velocity, opt.max_acceleration, opt.epsilon);
            // This is d/ds s_dot_max_acc(s+)
            const auto slope_right = (s_dot_max_acc_ab - s_dot_max_acc_after) / actual_step_right;

            // Evaluate s_ddot_max at (s+, s_dot_max_acc(s+))
            const auto [s_ddot_min_ab, s_ddot_max_ab] =
                compute_acceleration_bounds(q_prime_ab, q_double_prime_ab, s_dot_max_acc_ab, opt.max_acceleration, opt.epsilon);

            const auto phase_slope_right = s_ddot_max_ab / s_dot_max_acc_ab;
            is_switching_point = phase_slope_right <= slope_right;

            sp.forward_accel = s_ddot_max_ab;
            sp.backward_accel = s_ddot_min_ab;

        } else {
            // Should s_dot_max_acc_before and s_dot_max_acc_after be within epsilon of each other, then
            // there is no perceived discontinuity. Therefore, equation 38 cannot be fulfilled and we
            // proceed onwards.
            //
            // TODO: Should this actually be an epsilon wrapped comparison for equality first, before we look for step
            // up / step down?
            continue;
        }

        if (is_switching_point) {
            // Make sure we don't violate whichever side's velocity limit is constraining.
            const auto s_dot_max_vel_switching_min = std::min(s_dot_max_vel_before, s_dot_max_vel_after);

            // Only accept this as switching point if it is also feasible with respect to the velocity limits.
            if (opt.epsilon.wrap(s_dot_max_acc_switching_min) <= opt.epsilon.wrap(s_dot_max_vel_switching_min)) {
                return sp;
            }
        }
    }

    // No valid switching point found before path end - return end of path as switching point
    return switching_point{.point = {.s = cursor.path().length(), .s_dot = arc_velocity{0.0}},
                           .kind = trajectory::switching_point_kind::k_path_end};
}

// Selects between two switching points: returns the earlier one, or if at the same location
// (within epsilon), returns the one with lower velocity (more conservative).
[[gnu::pure]] switching_point select_switching_point(const switching_point& sp1, const switching_point& sp2, class epsilon epsilon) {
    if (epsilon.wrap(sp1.point.s) == epsilon.wrap(sp2.point.s)) {
        // Same location (within epsilon) - use whichever has lower velocity (more conservative)
        return (sp1.point.s_dot < sp2.point.s_dot) ? sp1 : sp2;
    }

    // Return whichever comes first along the path
    return (sp1.point.s < sp2.point.s) ? sp1 : sp2;
}

// Phase 1: Search segment boundaries for a discontinuous velocity-limit switching point
// (Kunz & Stilman equations 41-42).
std::optional<switching_point> find_discontinuous_velocity_switching_point(path::cursor cursor,
                                                                           const trajectory::options& opt,
                                                                           arc_length path_length) {
    auto boundary_cursor = cursor;
    auto last_evaluated_boundary = arc_length{-std::numeric_limits<double>::infinity()};
    while (boundary_cursor.position() < path_length) {
        auto current_segment = *boundary_cursor;
        const auto boundary = current_segment.end();

        // Guard: if the boundary hasn't advanced past the last one we evaluated,
        // we're stuck on a zero-length segment. Break to avoid an infinite loop.
        if (opt.epsilon.wrap(boundary) <= opt.epsilon.wrap(last_evaluated_boundary) ||
            opt.epsilon.wrap(boundary) >= opt.epsilon.wrap(path_length)) {
            break;
        }

        // Only check boundaries ahead of our starting position
        if (opt.epsilon.wrap(boundary) <= opt.epsilon.wrap(cursor.position())) {
            boundary_cursor.seek(boundary);
            continue;
        }

        last_evaluated_boundary = boundary;

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

        // If velocity limit is degenerate (near zero) on either side, this is a switching point where we must come to rest.
        if (opt.epsilon.wrap(s_dot_max_vel_before) == opt.epsilon.wrap(arc_velocity{0.0}) ||
            opt.epsilon.wrap(s_dot_max_vel_after) == opt.epsilon.wrap(arc_velocity{0.0})) {
            const auto switching_velocity = std::min(std::min(s_dot_max_vel_before, s_dot_max_vel_after), arc_velocity{0.0});
            return switching_point{.point = {.s = boundary, .s_dot = switching_velocity},
                                   .kind = trajectory::switching_point_kind::k_discontinuous_velocity_limit};
        }

        // We can't call compute_acceleration_bounds at s_dot_max_vel if it exceeds s_dot_max_accel,
        // because the centripetal term would already exceed the acceleration budget at that velocity.
        if (opt.epsilon.wrap(s_dot_max_accel_before) <= opt.epsilon.wrap(s_dot_max_vel_before) ||
            opt.epsilon.wrap(s_dot_max_accel_after) <= opt.epsilon.wrap(s_dot_max_vel_after)) {
            continue;
        }

        // Evaluate condition 41 (before-side only) first to avoid computing after-side
        // derivatives and acceleration bounds when the before-side already disqualifies.
        phase_plane_slope curve_slope_before{0.0};
        try {
            curve_slope_before = compute_velocity_limit_derivative(q_prime_before, q_double_prime_before, opt.max_velocity, opt.epsilon);
        } catch (const std::runtime_error&) {
            continue;
        }

        std::optional<trajectory::acceleration_bounds> accel_before;
        try {
            accel_before =
                compute_acceleration_bounds(q_prime_before, q_double_prime_before, s_dot_max_vel_before, opt.max_acceleration, opt.epsilon);
        } catch (const std::runtime_error&) {
            continue;
        }
        const auto& [s_ddot_min_before, s_ddot_max_before] = *accel_before;
        const auto trajectory_slope_before = s_ddot_min_before / s_dot_max_vel_before;
        const bool condition_41 = opt.epsilon.wrap(trajectory_slope_before) >= opt.epsilon.wrap(curve_slope_before);
        if (!condition_41) {
            continue;
        }

        // Condition 41 passed -- now evaluate condition 42 (after-side).
        phase_plane_slope curve_slope_after{0.0};
        try {
            curve_slope_after = compute_velocity_limit_derivative(q_prime_after, q_double_prime_after, opt.max_velocity, opt.epsilon);
        } catch (const std::runtime_error&) {
            continue;
        }

        std::optional<trajectory::acceleration_bounds> accel_after;
        try {
            accel_after =
                compute_acceleration_bounds(q_prime_after, q_double_prime_after, s_dot_max_vel_after, opt.max_acceleration, opt.epsilon);
        } catch (const std::runtime_error&) {
            continue;
        }
        const auto& [s_ddot_min_after, s_ddot_max_after] = *accel_after;
        const auto trajectory_slope_after = s_ddot_min_after / s_dot_max_vel_after;
        const bool condition_42 = opt.epsilon.wrap(trajectory_slope_after) <= opt.epsilon.wrap(curve_slope_after);

        if (condition_42) {
            // Valid discontinuous switching point found.
            // Switching velocity is the minimum of velocity limits on both sides.
            // TODO(RSDK-12848): Investigate the correctness of taking the minimum of these two switching points.
            const auto s_dot_max_vel_switching_min = std::min(s_dot_max_vel_before, s_dot_max_vel_after);
            const auto s_dot_max_accel_switching_min = std::min(s_dot_max_accel_before, s_dot_max_accel_after);

            // Only accept the velocity switching point if it is feasible with respect to the acceleration
            // limit curve.
            if (opt.epsilon.wrap(s_dot_max_accel_switching_min) >= opt.epsilon.wrap(s_dot_max_vel_switching_min)) {
                return switching_point{.point = {.s = boundary, .s_dot = s_dot_max_vel_switching_min},
                                       .kind = trajectory::switching_point_kind::k_discontinuous_velocity_limit};
            }
        }
    }

    return std::nullopt;
}

struct eq40_escape_bracket {
    arc_length before;
    arc_length after;
    phase_plane_slope after_delta;
    arc_velocity after_s_dot_max_vel;
};

// Phase 2: Coarse forward scan for an Eq. 40 sign bracket:
// previous/current Delta values on opposite sides of zero.
std::optional<eq40_escape_bracket> find_eq40_escape_bracket(path::cursor& search_cursor,
                                                            arc_length search_position,
                                                            arc_length search_limit,
                                                            const trajectory::options& opt) {
    constexpr phase_plane_slope k_zero_delta{0.0};
    auto previous_position = search_position;
    bool has_previous_delta = false;
    phase_plane_slope previous_delta{0.0};
    search_cursor.seek(search_position);
    auto previous_segment_end = (*search_cursor).end();

    while (search_cursor.position() < search_limit) {
        const auto current_position = search_cursor.position();
        const auto result = try_compute_eq40_delta(search_cursor, current_position, opt);

        // If we crossed a segment boundary, reset the baseline. A delta sign change
        // across a boundary is a geometric discontinuity, not a continuous Eq. 40 escape.
        const auto current_segment_end = (*search_cursor).end();
        if (current_segment_end != previous_segment_end) {
            has_previous_delta = false;
            previous_segment_end = current_segment_end;
        }

        if (result.has_value()) {
            if (has_previous_delta) {
                const bool previous_positive = opt.epsilon.wrap(previous_delta) > opt.epsilon.wrap(k_zero_delta);
                const bool current_nonpositive = opt.epsilon.wrap(result->delta) <= opt.epsilon.wrap(k_zero_delta);
                if (previous_positive && current_nonpositive) {
                    return eq40_escape_bracket{.before = previous_position,
                                               .after = current_position,
                                               .after_delta = result->delta,
                                               .after_s_dot_max_vel = result->s_dot_max_vel};
                }
            }
            // When has_previous_delta is false (first evaluable point or after a non-evaluable gap),
            // we only record the value as a baseline. A non-positive Delta here means escape is already
            // possible, but the algorithm requires a sign *transition* (Delta > 0 -> Delta <= 0) to identify
            // the switching point where the trajectory first becomes able to leave the velocity curve.
            previous_delta = result->delta;
            has_previous_delta = true;
        } else {
            has_previous_delta = false;
        }

        previous_position = current_position;

        // Advance cursor by step size.
        //
        // NOTE: The use of `opt.delta` here is definitely cheating: it is a duration,
        // not a distance. However, `opt.delta` is configurable and works as a practical
        // starting point for now.
        const auto next_position = std::min(current_position + arc_length{opt.delta.count()}, search_limit);
        if (opt.epsilon.wrap(next_position) == opt.epsilon.wrap(current_position)) {
            break;
        }
        search_cursor.seek(next_position);
    }

    return std::nullopt;
}

// Phase 3: Refine the coarse Eq. 40 bracket via bisection and accept only if
// delta converges to zero (within epsilon).
std::optional<switching_point> refine_continuous_velocity_switching_point(path::cursor& search_cursor,
                                                                          const eq40_escape_bracket& bracket,
                                                                          const trajectory::options& opt) {
    constexpr phase_plane_slope k_zero_delta{0.0};

    // If the coarse step already landed on the root, no bisection needed.
    if (opt.epsilon.wrap(bracket.after_delta) == opt.epsilon.wrap(k_zero_delta)) {
        return switching_point{.point = {.s = bracket.after, .s_dot = bracket.after_s_dot_max_vel},
                               .kind = trajectory::switching_point_kind::k_velocity_escape};
    }

    // TODO(RSDK-12767): Eliminate this hardcoded constant.
    constexpr int max_bisection_iterations = 100;
    auto positive_side = bracket.before;
    auto nonpositive_side = bracket.after;
    auto best_s_dot_max_vel = bracket.after_s_dot_max_vel;

    for (int iteration = 0; iteration < max_bisection_iterations; ++iteration) {
        // Positions converged -- the sign change is localized within epsilon.
        if (opt.epsilon.wrap(positive_side) == opt.epsilon.wrap(nonpositive_side)) {
            return switching_point{.point = {.s = nonpositive_side, .s_dot = best_s_dot_max_vel},
                                   .kind = trajectory::switching_point_kind::k_velocity_escape};
        }

        const auto mid = positive_side + ((nonpositive_side - positive_side) / 2.0);
        const auto mid_result = try_compute_eq40_delta(search_cursor, mid, opt);
        if (!mid_result.has_value()) {
            // Midpoint is non-evaluable (degenerate/singular). Contract from the
            // positive side to preserve the known Delta <= 0 endpoint and approach the root from
            // the trapped (Delta > 0) direction.
            positive_side = mid;
            continue;
        }

        // Found the root -- return immediately.
        if (opt.epsilon.wrap(mid_result->delta) == opt.epsilon.wrap(k_zero_delta)) {
            return switching_point{.point = {.s = mid, .s_dot = mid_result->s_dot_max_vel},
                                   .kind = trajectory::switching_point_kind::k_velocity_escape};
        }

        if (opt.epsilon.wrap(mid_result->delta) < opt.epsilon.wrap(k_zero_delta)) {
            nonpositive_side = mid;
            best_s_dot_max_vel = mid_result->s_dot_max_vel;
        } else {
            positive_side = mid;
        }
    }

    // 100 iterations on a verified sign-change bracket yields 2^-100 precision
    // this path is effectively unreachable but kept as a defensive safety net.
    return std::nullopt;
}

// Phase 2+3: Coarse Eq. 40 bracket search + bisection refinement.
// Searches for the first valid continuous velocity-escape switching point in [cursor.position(), search_limit).
std::optional<switching_point> find_continuous_velocity_switching_point(path::cursor cursor,
                                                                        arc_length search_limit,
                                                                        const trajectory::options& opt) {
    auto search_position = cursor.position();
    while (opt.epsilon.wrap(search_position) < opt.epsilon.wrap(search_limit)) {
        const auto coarse_bracket = find_eq40_escape_bracket(cursor, search_position, search_limit, opt);
        if (!coarse_bracket.has_value()) {
            break;
        }

        const auto refined = refine_continuous_velocity_switching_point(cursor, *coarse_bracket, opt);
        if (refined.has_value()) {
            return refined;
        }

        search_position = coarse_bracket->after;
    }

    return std::nullopt;
}

// Searches for a velocity switching point where escape from velocity curve becomes possible.
// Implements both continuous (Kunz & Stilman equation 40, with `Correction 5`) and discontinuous
// (Kunz & Stilman equations 41-42, with `Correction 6`) cases from Section VII-B.
//
// Returns the switching point on velocity curve. If no escape point found, returns end of path.
// Takes path::cursor by value (cheap copy) to seed forward search from current position.
[[gnu::pure]] switching_point find_velocity_switching_point(path::cursor cursor, const trajectory::options& opt) {
    const auto path_length = cursor.path().length();

    // Phase 1: boundary discontinuity search.
    const auto discontinuous_switching_point = find_discontinuous_velocity_switching_point(cursor, opt, path_length);

    // Phase 2+3: continuous escape search, bounded by the discontinuous result if one was found.
    const auto search_limit = discontinuous_switching_point.has_value() ? discontinuous_switching_point->point.s : path_length;
    const auto continuous_switching_point = find_continuous_velocity_switching_point(cursor, search_limit, opt);

    // Phase 4: return whichever switching point comes first.
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
    return switching_point{.point = {.s = path_length, .s_dot = arc_velocity{0.0}}, .kind = trajectory::switching_point_kind::k_path_end};
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
[[gnu::pure]] switching_point find_switching_point(path::cursor cursor, const trajectory::options& opt) {
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

trajectory::trajectory(class path p, options opt) : path_{std::move(p)}, options_{std::move(opt)} {}

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
        // Algorithm from Kunz & Stilman Section VI: phase plane (s, s_dot) integration.

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
            const auto forward_s_dot_interp = lerp(pt0.s_dot, pt1.s_dot, interpolation_factor);

            // Intersection occurs if backward's s_dot exceeds forward's interpolated s_dot.
            // Backward integration starts with low s_dot and increases as s decreases, eventually
            // crossing above the forward trajectory's s_dot at the same s position.
            if (traj.options_.epsilon.wrap(backward_s_dot) >= traj.options_.epsilon.wrap(forward_s_dot_interp)) {
                // Intersection found - return index of pt0 (start of bracketing interval)
                return std::distance(forward_points.begin(), pt0_it);
            }

            return std::nullopt;
        };

        // TODO: If switching_point communicated s via cursor, this would be even cleaner.
        path::cursor path_cursor = traj.path_.create_cursor();
        const auto integrate_forward_from = [&](switching_point where) -> switching_point {
            // If we have no integration points yet, then this is the beginning of time. Otherwise, we are starting from
            // a switching point from which we started backwards integration. The backwards integration pass has put
            // that point into integration_points, but we want to overwrite it, or we will end up creating a duplicate
            // point. This is a little clunky, but we want backwards integration to put in the switching point so it can
            // "cap" the trajectory at the very end, without requiring another call into forward integration.
            auto current_time = [&]() {
                if (traj.integration_points_.empty()) {
                    return trajectory::seconds{0.0};
                }
                assert(traj.integration_points_.back().s == where.point.s);
                assert(traj.integration_points_.back().s_dot == where.point.s_dot);
                auto now = traj.integration_points_.back().time;
                traj.integration_points_.pop_back();
                traj.duration_ = traj.integration_points_.back().time;
                return now;
            }();
            auto current_point{std::move(where.point)};

            std::optional<switching_point_kind> current_kind{std::move(where.kind)};
            auto* first_forward_observer{traj.options_.observer};

            while (true) {
                path_cursor.seek(current_point.s);

                // Capture current segment before any seeking, so we can detect segment boundary crossings
                const auto current_segment = *path_cursor;

                // Select acceleration for this integration step based on local situation.
                const auto [s_ddot_desired, following] = [&]() -> std::pair<arc_acceleration, bool> {
                    if (std::exchange(current_kind, std::nullopt)) {
                        if (where.forward_accel) {
                            // TODO: is this false correct? What about vel sps?
                            return {*where.forward_accel, false};
                        }
                    }

                    const auto q_prime = path_cursor.tangent();
                    const auto q_double_prime = path_cursor.curvature();

                    // Check if we are currently at the velocity limit. If so, use tangent acceleration
                    // to follow the curve rather than max acceleration, which would immediately breach
                    // the limit and trigger bisection.
                    //
                    // TODO: In some cases, we may already know these limits for `current` from work
                    // done below on `next`, so we could eliminate this call.
                    const auto [_1, s_dot_max_vel] = compute_velocity_limits(
                        q_prime, q_double_prime, traj.options_.max_velocity, traj.options_.max_acceleration, traj.options_.epsilon);

                    // Find the upper limit for acceleration at this phase point.
                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, current_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    // If we are at the velocity limit, clamp our acceleration to follow it.
                    const bool at_velocity_limit =
                        traj.options_.epsilon.wrap(current_point.s_dot) == traj.options_.epsilon.wrap(s_dot_max_vel);

                    if (at_velocity_limit) {
                        const auto vel_curve_slope =
                            compute_velocity_limit_derivative(q_prime, q_double_prime, traj.options_.max_velocity, traj.options_.epsilon);

                        // The velocity curve could be moving up more steeply than our acceleration
                        // bounds allow, so we need to take the smaller value. We don't need to
                        // worry about the lower bound, because that condition should have been
                        // handled as the "trapped" condition below and forced a switching point
                        // search.
                        assert(vel_curve_slope > s_ddot_min);
                        const auto s_ddot_tangent = std::min(vel_curve_slope * current_point.s_dot, s_ddot_max);

                        return {s_ddot_tangent, true};
                    }

                    return {s_ddot_max, false};
                }();

                // Compute candidate next point via Euler integration with desired acceleration
                auto next_point = euler_step(current_point, s_ddot_desired, traj.options_.delta, traj.options_.epsilon);

                // Forward integration should move forward in the phase plane
                if (next_point.s <= current_point.s) [[unlikely]] {
                    throw std::runtime_error{"TOTG algorithm error: forward integration must increase s"};
                }

                // Don't integrate past the end of a segment. Instead, interpolate to the segment end. Otherwise, if we
                // had a very short subsequent segment, but which contained a more limiting constraint, we might
                // integrate right over it with a coarse `dt`, and tunnel through the infeasible region.
                if (const auto segment = *path_cursor; next_point.s > segment.end()) {
                    const auto delta_s_desired = next_point.s - current_point.s;
                    const auto delta_s_achieved = segment.end() - current_point.s;
                    next_point.s = segment.end();
                    next_point.s_dot = lerp(current_point.s_dot, next_point.s_dot, delta_s_achieved / delta_s_desired);
                }

                // Advance the path cursor to the arc length we just computed above. Note that this intentionally
                // moves us to the next segment if we are on a boundary. We want to evaluate limits against the constraints
                // as we will see them moving forward.
                path_cursor.seek(next_point.s);

                auto next_q_prime = path_cursor.tangent();
                auto next_q_double_prime = path_cursor.curvature();

                // Compute the velocity limits at the probe point.
                auto [next_s_dot_max_acc, next_s_dot_max_vel] = compute_velocity_limits(
                    next_q_prime, next_q_double_prime, traj.options_.max_velocity, traj.options_.max_acceleration, traj.options_.epsilon);

                // If we got here by following, but we clipped past the velocity limit curve, then
                // clamp to it so we don't bisect.
                //
                // TODO: Do we need to check against accel bounds to do this safely? Is it still worth it if we need to?
                if (following && next_point.s_dot > next_s_dot_max_vel) {
                    next_point.s_dot = next_s_dot_max_vel;
                }

                // If we hit the a limit curve, bisect until we have `next_point` in bounds,
                // `breach_point` out of bounds, and a separation less than our configured epsilon.
                const auto limit_hit_event = [&]() mutable -> std::optional<integration_observer::limit_hit_event> {
                    if (next_point.s_dot <= next_s_dot_max_acc && next_point.s_dot <= next_s_dot_max_vel) [[likely]] {
                        return std::nullopt;
                    }

                    auto breach_point = next_point;
                    next_point = current_point;

                    auto breach_s_dot_max_acc = next_s_dot_max_acc;
                    auto breach_s_dot_max_vel = next_s_dot_max_vel;

                    while (traj.options_.epsilon.wrap(next_point.s) != traj.options_.epsilon.wrap(breach_point.s)) {
                        const phase_point mid = {.s = midpoint(next_point.s, breach_point.s),
                                                 .s_dot = midpoint(next_point.s_dot, breach_point.s_dot)};

                        path_cursor.seek(mid.s);
                        const auto mid_q_prime = path_cursor.tangent();
                        const auto mid_q_double_prime = path_cursor.curvature();

                        // Compute the velocity limits at the midpoint.
                        const auto [midpoint_s_dot_max_acc, midpoint_s_dot_max_vel] =
                            compute_velocity_limits(mid_q_prime,
                                                    mid_q_double_prime,
                                                    traj.options_.max_velocity,
                                                    traj.options_.max_acceleration,
                                                    traj.options_.epsilon);

                        // Use > here, because we really want the breach point to be on the violating side of the curve.
                        if (mid.s_dot > midpoint_s_dot_max_acc || mid.s_dot > midpoint_s_dot_max_vel) {
                            breach_point = mid;
                            breach_s_dot_max_acc = midpoint_s_dot_max_acc;
                            breach_s_dot_max_vel = midpoint_s_dot_max_vel;
                        } else {
                            next_point = mid;
                            next_q_prime = mid_q_prime;
                            next_q_double_prime = mid_q_double_prime;
                            next_s_dot_max_acc = midpoint_s_dot_max_acc;
                            next_s_dot_max_vel = midpoint_s_dot_max_vel;
                        }
                    }

                    path_cursor.seek(breach_point.s);
                    const auto breach_segment = *path_cursor;

                    // If we violated across a segment boundary, we need to do a switching point search.
                    // Check if current_point and breach_point are in different segments.
                    if (current_segment.start() != breach_segment.start()) {
                        return integration_observer::limit_hit_event{
                            .breach = breach_point, .s_dot_max_acc = breach_s_dot_max_acc, .s_dot_max_vel = breach_s_dot_max_vel};
                    }

                    // Determine which limit was violated and classify as source or sink.
                    // If acceleration limit violated at all, it takes precedence - these curves can
                    // cross and we may violate both simultaneously.
                    //
                    // We analyze the trajectory at next_point (the last feasible point) to determine
                    // if it's heading into or away from the infeasible region.

                    if (breach_point.s_dot >= breach_s_dot_max_acc) {
                        // Check whether trajectory naturally escapes or drives deeper into infeasible region.

                        // Compute phase slope at next_point (feasible) using already-available data.
                        const auto [next_s_ddot_min, next_s_ddot_max] = compute_acceleration_bounds(
                            next_q_prime, next_q_double_prime, next_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);
                        const auto next_phase_slope = next_s_ddot_max / next_point.s_dot;

                        // Compute acceleration curve slope by looking at next_point and a point before it.
                        const auto before_next = next_point.s - arc_length{traj.options_.epsilon};
                        if (next_point.s - before_next < traj.options_.epsilon * 0.5) {
                            // Couldn't get good separation, we need to search for a switching point.
                            return integration_observer::limit_hit_event{
                                .breach = breach_point, .s_dot_max_acc = breach_s_dot_max_acc, .s_dot_max_vel = breach_s_dot_max_vel};
                        }

                        path_cursor.seek(before_next);
                        const auto before_next_q_prime = path_cursor.tangent();
                        const auto before_next_q_double_prime = path_cursor.curvature();

                        const auto [before_next_s_dot_max_acc, _1] = compute_velocity_limits(before_next_q_prime,
                                                                                             before_next_q_double_prime,
                                                                                             traj.options_.max_velocity,
                                                                                             traj.options_.max_acceleration,
                                                                                             traj.options_.epsilon);

                        const auto next_acc_curve_slope = (next_s_dot_max_acc - before_next_s_dot_max_acc) / (next_point.s - before_next);
                        if (next_phase_slope > next_acc_curve_slope) {
                            // The acceleration limit drops faster than we can follow. We need a switching
                            // point to start backward integration.
                            return integration_observer::limit_hit_event{
                                .breach = breach_point, .s_dot_max_acc = breach_s_dot_max_acc, .s_dot_max_vel = breach_s_dot_max_vel};
                        }
                        // The trajectory naturally escapes back to the feasible region. This was
                        // numerical overshoot from a source point.
                        return std::nullopt;
                    }

                    // We hit the velocity curve only. Check if we are trapped, can escape, or can follow.
                    //
                    // TODO: We might be able to avoid recomputing these since we could track them in the bisection loop
                    // like we do for next.
                    path_cursor.seek(breach_point.s);
                    const auto breach_q_prime = path_cursor.tangent();
                    const auto breach_q_double_prime = path_cursor.curvature();

                    const auto [breach_s_ddot_min, breach_s_ddot_max] = compute_acceleration_bounds(
                        breach_q_prime, breach_q_double_prime, breach_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);
                    const auto min_phase_slope = breach_s_ddot_min / breach_point.s_dot;
                    const auto max_phase_slope = breach_s_ddot_max / breach_point.s_dot;
                    const auto vel_curve_slope = compute_velocity_limit_derivative(
                        breach_q_prime, breach_q_double_prime, traj.options_.max_velocity, traj.options_.epsilon);

                    if (min_phase_slope > vel_curve_slope) {
                        // The velocity curve drops faster than we can follow while respecting the
                        // acceleration bounds. We are trapped and need a switching point.
                        return integration_observer::limit_hit_event{
                            .breach = breach_point, .s_dot_max_acc = breach_s_dot_max_acc, .s_dot_max_vel = breach_s_dot_max_vel};
                    }

                    if (max_phase_slope < vel_curve_slope) {
                        // The velocity curve rises faster than we can accelerate, so the trajectory
                        // naturally falls away from the limit. This was numerical overshoot. Accept
                        // the breach point as valid since we'll naturally escape from it, but clamp
                        // velocity to avoid starting in a forbidden region.
                        next_point = breach_point;
                        next_point.s_dot = breach_s_dot_max_vel;
                        return std::nullopt;
                    }

                    // The curve slope is within our acceleration authority, so we can follow it
                    // tangentially. Accept the breach point as our next position and clamp velocity
                    // to the limit so the next iteration detects "at velocity limit" and uses tangent
                    // acceleration rather than max acceleration, avoiding repeated bisection.
                    next_point = breach_point;
                    next_point.s_dot = breach_s_dot_max_vel;
                    return std::nullopt;
                }();

                const auto delta_s = next_point.s - current_point.s;
                const auto delta_s_dot = next_point.s_dot - current_point.s_dot;

                // When bisection converges to the same arc length, we're at a segment boundary.
                // The cursor has already advanced to the next segment during seek operations.
                if (delta_s == arc_length{0.0}) {
                    if (first_forward_observer != nullptr) {
                        throw std::runtime_error{"Cannot advance from initial forward position"};
                    }

                    if (limit_hit_event) {
                        // Stuck due to limit in new segment - record position for intersection detection.
                        traj.integration_points_.push_back({.time = current_time,
                                                            .s = current_point.s,
                                                            .s_dot = current_point.s_dot,
                                                            .s_ddot = arc_acceleration{std::numeric_limits<double>::quiet_NaN()}});

                        if (traj.options_.observer) {
                            traj.options_.observer->on_hit_limit_curve(traj, *limit_hit_event);
                        }

                        path_cursor.seek(current_point.s);
                        return find_switching_point(path_cursor, traj.options_);
                    }

                    // Crossed segment boundary without hitting limit - try again with new segment geometry.
                    continue;
                }

                const auto s_dot_average = midpoint(current_point.s_dot, next_point.s_dot);
                const auto dt = delta_s / s_dot_average;
                const auto s_ddot = delta_s_dot / dt;
                const auto next_time = current_time + dt;

                traj.integration_points_.push_back(
                    {.time = current_time, .s = current_point.s, .s_dot = current_point.s_dot, .s_ddot = s_ddot});

                if (auto* const observer = std::exchange(first_forward_observer, nullptr)) {
                    observer->on_started_forward_integration(traj, {.start = {current_point.s, current_point.s_dot}});
                }

                if (limit_hit_event) {
                    traj.integration_points_.push_back({.time = next_time,
                                                        .s = next_point.s,
                                                        .s_dot = next_point.s_dot,
                                                        .s_ddot = arc_acceleration{std::numeric_limits<double>::quiet_NaN()}});

                    if (traj.options_.observer) {
                        traj.options_.observer->on_hit_limit_curve(traj, *limit_hit_event);
                    }

                    path_cursor.seek(next_point.s);
                    return find_switching_point(path_cursor, traj.options_);
                }

                if (next_point.s == traj.path_.length()) {
                    traj.integration_points_.push_back({.time = next_time,
                                                        .s = next_point.s,
                                                        .s_dot = next_point.s_dot,
                                                        .s_ddot = arc_acceleration{std::numeric_limits<double>::quiet_NaN()}});

                    path_cursor.seek(next_point.s);
                    return {.point = {next_point.s, arc_velocity{0.0}},
                            .kind = switching_point_kind::k_path_end,
                            .forward_accel = arc_acceleration{0.0}};
                }

                current_point = next_point;
                current_time = next_time;
            }
        };

        // We could, in theory, accumulate backwards points into the `integration_points_` array. This has the downside
        // that observers have access to those points. As currently designed, it would probably be unobservable since
        // there aren't events that fire between the event to indicate that backwards integration has started, and the event
        // that fires after a splice. But if such events were added later for some reason, the state of the integration points
        // array might be quite confusing. For now, use a separate buffer. Scope it to the mutable lambda because we want
        // to keep the allocation, at the logical cost of needing to clear it on entry to the function.

        // The `where` parameter is `const` because we intend to return it so that it will feed back
        // into `integrate_forward`. Intentional or accidental alteration would likely result in a bug.
        auto integrate_backwards_from = [&, backwards_points = trajectory::integration_points{}](const switching_point where) mutable {
            // Clear out any old state.
            backwards_points.clear();

            // Copy the cursor. We don't want to change path_cursor, because that's where we will start forward integration from.
            //
            // TODO: This might be cleaner if the switching_point struct contained a path cursor.
            auto backwards_cursor = path_cursor;

            if (where.point.s < traj.integration_points_.back().s || where.point.s_dot >= traj.integration_points_.back().s_dot) {
                std::ostringstream oss;
                oss << "TOTG algorithm error: switching point must be below and not before last forward point "
                    << "(higher s, lower s_dot). "
                    << "Switching point: s=" << where.point.s << " s_dot=" << where.point.s_dot << ". "
                    << "Last forward point: s=" << traj.integration_points_.back().s << " s_dot=" << traj.integration_points_.back().s_dot;
                throw std::runtime_error{oss.str()};
            }

            if (traj.options_.observer) {
                traj.options_.observer->on_started_backward_integration(traj, {.start = where.point, .kind = where.kind});
            }

            auto current_point{where.point};
            std::optional<switching_point_kind> current_kind{where.kind};

            while (true) {
                backwards_cursor.seek(current_point.s);

                // Compute the maximum acceleration we are permitted at this phase point and switching point type.
                const auto s_ddot_desired = [&] {
                    if (std::exchange(current_kind, std::nullopt)) {
                        if (where.backward_accel) {
                            return *where.backward_accel;
                        }
                    };

                    auto q_prime = backwards_cursor.tangent();
                    auto q_double_prime = backwards_cursor.curvature();

                    auto [s_ddot_min, _1] = compute_acceleration_bounds(
                        q_prime, q_double_prime, current_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    // Minimum acceleration must be negative to produce backward motion (decreasing s)
                    // Allow small tolerance for numerical precision at near-zero acceleration points
                    if (s_ddot_min >= -traj.options_.epsilon) {
                        if (s_ddot_min > traj.options_.epsilon) {
                            // Clearly positive - this is an error
                            throw std::runtime_error{"TOTG algorithm error: backward integration requires negative minimum acceleration"};
                        }
                        // Probably numerical noise, just clamp to zero. Backward integration will
                        // make progress as long as s_dot is non-zero.
                        return arc_acceleration{0.0};
                    }

                    return s_ddot_min;
                }();

                // Compute candidate next point via Euler integration with negative dt and minimum acceleration.
                // Negative dt reverses time direction, reconstructing velocities that led to current point.
                // With s_ddot_min < 0 and dt < 0, s_dot increases (up) while s decreases (left).
                //
                // TODO(RSDK-12981): There's no guarantee that the candidate we select here by going
                // backwards with `s_ddot_to_use` as determined at the switching point would then
                // integrate forwards from the candidate to the switching point.
                const auto next_point =
                    euler_step(current_point.s, current_point.s_dot, s_ddot_desired, -traj.options_.delta, traj.options_.epsilon);

                // TODO: Do we need to avoid integrating across segment boundaries here too?

                // Backward integration must decrease s.
                if ((next_point.s >= current_point.s)) [[unlikely]] {
                    throw std::runtime_error{"TOTG algorithm error: backward integration must decrease s and not decrease s_dot"};
                }

                // It should not be possible to integrate backwards to the beginning of time without
                // intersecting the forward trajectory along the way.
                if (next_point.s <= arc_length{0.0}) {
                    throw std::runtime_error{
                        "TOTG algorithm error: backward integration reached start without intersecting "
                        "forward trajectory - trajectory is infeasible (would require non-zero initial velocity)"};
                }

                // Check if we have moved far enough backward in the phase plane that we are now at or before
                // the last forward point. If we have, it is time to find the exact intersection.
                //
                // NOTE: We do this check before checking to see if we have hit the velocity limit curve, since
                // we may be trying to intersect a forward trajectory that was following the curve. If we check
                // against the limit curve first, we might error out when we should have intersected.
                std::optional<size_t> intersection_index;
                if (next_point.s <= traj.integration_points_.back().s) {
                    intersection_index = find_trajectory_intersection(next_point.s, next_point.s_dot);
                }

                if (intersection_index) {
                    // Splice the backward trajectory into forward trajectory at the intersection point.
                    // Note that `next_point` will not be included in the backwards trajectory.

                    // Truncate forward trajectory after intersection point

                    const auto truncate_index = static_cast<ptrdiff_t>(*intersection_index + 1);
                    const auto truncate_begin = std::next(begin(traj.integration_points_), truncate_index);

                    // Only preserve pruned points if observer needs them for event reporting
                    if (traj.options_.observer) {
                        traj.frosts_.emplace_back(std::make_move_iterator(truncate_begin),
                                                  std::make_move_iterator(end(traj.integration_points_)));
                    }
                    traj.integration_points_.erase(truncate_begin, end(traj.integration_points_));

                    // Fix splice point kinematic consistency
                    //
                    // The last forward point has s_ddot computed for some point which we just removed. We need to
                    // recompute its s_ddot to connect to the first backward point, and compute the appropriate dt that
                    // makes this kinematically consistent.

                    auto& last_forward_point = traj.integration_points_.back();

                    // Access backwards points in reverse order (first backward point is closest to splice).
                    const auto backwards_points_reversed = std::ranges::reverse_view(std::as_const(backwards_points));
                    const auto& first_backward_point = backwards_points_reversed.front();

                    // Compute the position delta and the average velocity across the points.
                    const auto delta_s = first_backward_point.s - last_forward_point.s;
                    const auto mean_sdot = midpoint(last_forward_point.s_dot, first_backward_point.s_dot);

                    // Validate average velocity is non-zero to avoid division by zero.
                    if (traj.options_.epsilon.wrap(mean_sdot) == traj.options_.epsilon.wrap(arc_velocity{0.0})) {
                        throw std::runtime_error{"Splice point has near-zero average velocity - division by zero in dt computation"};
                    }

                    // Now we can compute the dt that moves us from the one point to the other.
                    const auto dt = delta_s / mean_sdot;
                    if (dt <= trajectory::seconds{0.0}) {
                        throw std::runtime_error{"Computed negative or zero dt at splice point - intersection detection error"};
                    }

                    // Compute the acceleration that connects last_forward_point to first_backward_point.
                    const auto computed_s_ddot = (first_backward_point.s_dot - last_forward_point.s_dot) / dt;

                    // Query the path geometry at last_forward_point to validate that our computed acceleration is
                    // actually feasible.
                    backwards_cursor.seek(last_forward_point.s);
                    const auto q_prime = backwards_cursor.tangent();
                    const auto q_double_prime = backwards_cursor.curvature();

                    const auto [s_ddot_min, s_ddot_max] = compute_acceleration_bounds(
                        q_prime, q_double_prime, last_forward_point.s_dot, traj.options_.max_acceleration, traj.options_.epsilon);

                    if (traj.options_.epsilon.wrap(computed_s_ddot) < traj.options_.epsilon.wrap(s_ddot_min) ||
                        traj.options_.epsilon.wrap(computed_s_ddot) > traj.options_.epsilon.wrap(s_ddot_max)) {
                        std::ostringstream oss;
                        oss << "Splice point requires infeasible acceleration: "
                            << "computed=" << static_cast<double>(computed_s_ddot) << ", bounds=[" << static_cast<double>(s_ddot_min)
                            << ", " << static_cast<double>(s_ddot_max) << "]";
                        throw std::runtime_error{oss.str()};
                    }

                    // Correct the acceleration value at the last forward point.
                    last_forward_point.s_ddot = computed_s_ddot;

                    // In the loop below, we need the time of the last forward point, but we are going
                    // to be writing to the vector that holds that point. Extract the time to a local variable
                    // so it doesn't get destroyed out from under us on a reallocation.
                    auto last_forward_point_time = last_forward_point.time;

                    // Reserve space to avoid reallocations during bulk append.
                    traj.integration_points_.reserve(traj.integration_points_.size() + backwards_points.size());

                    // Append backward trajectory with corrected timestamps.
                    const auto size = std::ranges::ssize(backwards_points_reversed);
                    for (std::remove_const_t<decltype(size)> i = 0; i < size; ++i) {
                        auto corrected = backwards_points_reversed[i];

                        // All points in the backwards array were computed with a uniform delta, but we need to account
                        // for the computed dt that happened at the splice.
                        corrected.time = (last_forward_point_time + dt) + (i * traj.options_.delta);

                        // For all but the last point we will be adding, we assume the acceleration that took us
                        // backward to that point is the acceleration we should use to move forward (erroneously, see
                        // RSDK-12981 above), so we need to take it from its neighbor. For the last point, we need to
                        // respect the switching point's forward acceleration if set, since we may not call forward
                        // integration again if at the end of path.
                        corrected.s_ddot = (i < size - 1)
                                               ? backwards_points_reversed[i + 1].s_ddot
                                               : where.forward_accel.value_or(arc_acceleration{std::numeric_limits<double>::quiet_NaN()});

                        traj.integration_points_.push_back(corrected);
                    }

                    // Update trajectory duration to reflect new endpoint before invoking callbacks.
                    // Observers receiving the trajectory by reference may query its duration.
                    traj.duration_ = traj.integration_points_.back().time;

                    // Notify observer that trajectory has been extended with finalized backward segment
                    if (traj.options_.observer) {
                        // Observer exists, so frosts_ must have been populated above
                        traj.options_.observer->on_trajectory_extended(traj, {.pruned = std::ranges::subrange(traj.frosts_.back())});
                    }

                    // Backwards integration has completed. Return the switching point so that forward
                    // integration can resume from there.
                    return where;
                }

                // Query geometry at candidate position to check if it would hit limit curves.
                // Backward integration hitting a limit curve indicates the trajectory is infeasible -
                // we cannot decelerate from the switching point without violating joint constraints.
                backwards_cursor.seek(next_point.s);
                const auto next_q_prime = backwards_cursor.tangent();
                const auto next_q_double_prime = backwards_cursor.curvature();

                const auto [s_dot_max_acc, s_dot_max_vel] = compute_velocity_limits(
                    next_q_prime, next_q_double_prime, traj.options_.max_velocity, traj.options_.max_acceleration, traj.options_.epsilon);
                const auto s_dot_limit = std::min(s_dot_max_acc, s_dot_max_vel);

                if (s_dot_limit <= arc_velocity{0.0}) [[unlikely]] {
                    throw std::runtime_error{"TOTG algorithm error: velocity limit curve is non-positive during backward integration"};
                }

                // `Divergent Behavior 3`: Candidate exceeding limit curve is an algorithm error - trajectory is
                // infeasible. Being at the limit (within epsilon) is allowed - only exceeding it is rejected.
                if (traj.options_.epsilon.wrap(next_point.s_dot) > traj.options_.epsilon.wrap(s_dot_limit)) {
                    throw std::runtime_error{"TOTG algorithm error: backward integration exceeded limit curve - trajectory is infeasible"};
                }

                // The point is feasible, so append it to the backwards trajectory points. Note that the timestamps
                // are zero-based from the switching point, but will be fixed up when we splice.
                const trajectory::seconds previous_time =
                    backwards_points.empty() ? trajectory::seconds{0.0} : backwards_points.back().time;
                backwards_points.push_back({.time = previous_time + traj.options_.delta,
                                            .s = current_point.s,
                                            .s_dot = current_point.s_dot,
                                            .s_ddot = s_ddot_desired});

                // Continue backward integration from the point we just added.
                current_point = next_point;
            }
        };

        switching_point sp = {.point = {arc_length{0}, arc_velocity{0}}, .kind = switching_point_kind::k_path_begin};
        while (sp.kind != switching_point_kind::k_path_end) {
            sp = integrate_backwards_from(integrate_forward_from(sp));
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

trajectory::velocity_limits trajectory::get_velocity_limits(const path::cursor& cursor) const {
    const auto tangent = cursor.tangent();
    const auto curvature = cursor.curvature();
    return compute_velocity_limits(tangent, curvature, options_.max_velocity, options_.max_acceleration, options_.epsilon);
}

trajectory::acceleration_bounds trajectory::get_acceleration_bounds(const path::cursor& cursor, arc_velocity s_dot) const {
    const auto tangent = cursor.tangent();
    const auto curvature = cursor.curvature();
    return compute_acceleration_bounds(tangent, curvature, s_dot, options_.max_acceleration, options_.epsilon);
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

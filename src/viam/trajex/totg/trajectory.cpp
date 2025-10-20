#include <viam/trajex/totg/trajectory.hpp>

#include <cassert>
#include <functional>
#include <numeric>
#include <ranges>
#include <stdexcept>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_length.hpp>

namespace viam::trajex::totg {

trajectory::trajectory(class path p, options opt, integration_points points)
    : path_{std::move(p)}, options_{std::move(opt)}, integration_points_{std::move(points)} {
    // Validate path is not empty
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

trajectory trajectory::create(class path p, options opt, integration_points test_points) {
    // Validate options
    if (opt.max_velocity.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_velocity DOF doesn't match path DOF"};
    }

    if (opt.max_acceleration.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_acceleration DOF doesn't match path DOF"};
    }

    if (opt.delta <= seconds{0.0}) {
        throw std::invalid_argument{"delta must be positive"};
    }

    if (opt.epsilon <= 0.0) {
        throw std::invalid_argument{"epsilon must be positive"};
    }

    if (test_points.empty()) {
        // Normal path: generate fake integration points for TOPP stub
        // TODO(acm): Replace this with real TOPP algorithm
        // - Set up phase plane (s, s_dot) with velocity/acceleration limits
        // - Forward integration from s=0 to find maximum velocity curve (MVC)
        // - Backward integration from s=length to find time-optimal trajectory

        // TEMPORARY STUB: Fake integration with zero velocity/acceleration
        const double path_length = static_cast<double>(p.length());

        // Calculate fake duration assuming average max velocity
        const double sum = std::accumulate(opt.max_velocity.begin(), opt.max_velocity.end(), 0.0);
        const double avg_max_vel = opt.max_velocity.size() > 0 ? sum / static_cast<double>(opt.max_velocity.size()) : 1.0;
        const double fake_duration = (avg_max_vel > 0.0) ? path_length / avg_max_vel : 1.0;

        // Generate integration points at regular intervals with zero velocity/acceleration
        const int num_steps = std::max(1, static_cast<int>(std::ceil(fake_duration / opt.delta.count())));
        const double actual_delta = fake_duration / static_cast<double>(num_steps);

        test_points.reserve(static_cast<size_t>(num_steps) + 1);

        auto step_indices = std::views::iota(0, num_steps + 1);
        for (int i : step_indices) {
            const double t = static_cast<double>(i) * actual_delta;
            // Explicitly set last point to exactly path.length() to avoid floating point error
            const arc_length s = (i == num_steps) ? p.length() : arc_length{path_length * (t / fake_duration)};
            test_points.push_back({
                .time = seconds{t},
                .s = s,
                .s_dot = 0.0,  // Zero velocity (fake)
                .s_ddot = 0.0  // Zero acceleration (fake)
            });
        }
    } else {
        // Test path: validate provided integration points
        // First point must be at t=0
        if (test_points.front().time != seconds{0.0}) {
            throw std::invalid_argument{"First integration point must have time == 0"};
        }

        // Pairwise validation using C++20 ranges
        auto pairs = std::views::iota(size_t{0}, test_points.size() - 1) |
                     std::views::transform([&](size_t i) { return std::pair{std::cref(test_points[i]), std::cref(test_points[i + 1])}; });

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

        // Validate last point's arc length
        if (test_points.back().s < arc_length{0.0} || test_points.back().s > p.length()) {
            throw std::invalid_argument{"Integration point arc lengths must be in [0, path.length()]"};
        }
    }

    // Construct trajectory with integration points
    // Duration is set automatically in constructor from last integration point
    return trajectory{std::move(p), std::move(opt), std::move(test_points)};
}

struct trajectory::sample trajectory::sample(trajectory::seconds t) const {
    if (t < trajectory::seconds{0.0} || t > duration_) {
        throw std::out_of_range{"Time out of trajectory bounds"};
    }

    // Create a cursor, seek it to the desired time, and sample
    auto c = create_cursor();
    c.seek(t);
    return c.sample();
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

// Note: trajectory::samples() is a template method defined in the header

// trajectory::cursor implementations

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
    // Validate cursor is not at sentinel
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot sample cursor at sentinel position"};
    }

    // Precondition: cursor must have valid hint (maintained by seek())
    assert(time_hint_ != traj_->get_integration_points().end());
    assert(time_hint_->time <= time_);

    // Use time_hint_ for O(1) lookup (positioned by seek())
    // time_hint_ points to the integration point at or before current time_
    const integration_point& p0 = *time_hint_;

    // Piecewise constant acceleration interpolation between integration points:
    // Position:     s(t) = s0 + s_dot0 * dt + 0.5 * s_ddot0 * dt^2  (computed in update_path_cursor_position_)
    // Velocity:     s_dot(t) = s_dot0 + s_ddot0 * dt
    // Acceleration: s_ddot(t) = s_ddot0  (constant between integration points)
    const double dt = (time_ - p0.time).count();
    const double s_dot = p0.s_dot + (p0.s_ddot * dt);
    const double s_ddot = p0.s_ddot;

    // Query path geometry at current path_cursor_ position
    // path_cursor_ is already positioned at correct arc length by seek()
    const auto q = path_cursor_.configuration();
    const auto q_prime = path_cursor_.tangent();
    const auto q_double_prime = path_cursor_.curvature();

    // Apply chain rule to compute joint velocities and accelerations
    // q_dot(t) = q'(s) * s_dot
    const auto q_dot = q_prime * s_dot;

    // q_ddot(t) = q'(s) * s_ddot + q''(s) * s_dot^2
    const auto q_ddot = q_prime * s_ddot + q_double_prime * (s_dot * s_dot);

    return {.time = time_, .configuration = q, .velocity = q_dot, .acceleration = q_ddot};
}

void trajectory::cursor::update_path_cursor_position_(seconds t) {
    // Postcondition: seek() must position time_hint_ correctly
    assert(time_hint_ != traj_->get_integration_points().end());
    assert(time_hint_->time <= t);

    // Interpolate arc length at time t using constant acceleration motion from time_hint_
    // s(t) = s0 + s_dot0 * dt + 0.5 * s_ddot0 * dt^2
    // This assumes piecewise constant acceleration between TOPP integration points

    const auto& p0 = *time_hint_;
    const double dt = (t - p0.time).count();
    const double s_interp = static_cast<double>(p0.s) + (p0.s_dot * dt) + (0.5 * p0.s_ddot * dt * dt);

    // By construction, s_interp cannot exceed path.length() when t <= duration
    assert(s_interp <= static_cast<double>(traj_->path_.length()));

    // Position path cursor at interpolated arc length
    path_cursor_.seek(arc_length{s_interp});
}

void trajectory::cursor::seek(seconds t) {
    // Overflow to ±infinity sentinels outside valid range
    if (t < seconds{0.0}) {
        time_ = seconds{-std::numeric_limits<double>::infinity()};
        return;  // Don't update hints at sentinel
    }
    if (t > traj_->duration()) {
        time_ = seconds{std::numeric_limits<double>::infinity()};
        return;  // Don't update hints at sentinel
    }

    // Update time
    time_ = t;

    // Precondition: time_hint_ must be valid (prevents UB in backward path)
    assert(time_hint_ != traj_->get_integration_points().end());

    // Smart hint update strategy for O(1) amortized performance
    // Maintains invariant: time_hint_->time <= time_ (verified in update_path_cursor_position_)
    // 1. Fast path: Check if current hint is still valid
    // 2. Check adjacent points (forward/backward by 1)
    // 3. Fallback: Binary search for large jumps

    const auto& points = traj_->get_integration_points();

    // Fast path: Check if current hint is still valid
    if (time_hint_ != points.end() && time_hint_->time <= t) {
        auto next_hint = time_hint_;
        ++next_hint;
        if (next_hint == points.end() || t < next_hint->time) {
            // Hint is still valid, O(1)
            update_path_cursor_position_(t);
            return;
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
                return;
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
            return;
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
}

void trajectory::cursor::seek_by(seconds dt) {
    seek(time_ + dt);
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

#include <viam/trajex/totg/trajectory.hpp>

#include <numeric>
#include <stdexcept>

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_length.hpp>

namespace viam::trajex::totg {

trajectory::trajectory(class path p) : path_{std::move(p)} {}

trajectory trajectory::create(class path p, const options& opt) {
    if (opt.max_velocity.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_velocity DOF doesn't match path DOF"};
    }
    if (opt.max_acceleration.shape(0) != p.dof()) {
        throw std::invalid_argument{"max_acceleration DOF doesn't match path DOF"};
    }
    if (opt.delta <= 0.0) {
        throw std::invalid_argument{"delta must be positive"};
    }
    if (opt.epsilon <= 0.0) {
        throw std::invalid_argument{"epsilon must be positive"};
    }

    trajectory traj{std::move(p)};

    // TODO(acm): Implement TOPP algorithm
    // - Set up phase plane (s, ṡ) with velocity/acceleration limits
    // - Forward integration from s=0 to find maximum velocity curve (MVC)
    // - Backward integration from s=length to find time-optimal trajectory
    // - Store time parameterization in trajectory

    // TEMPORARY STUB: Fake duration calculation for testing
    // Assume constant velocity = average of max velocities
    const double sum = std::accumulate(opt.max_velocity.begin(), opt.max_velocity.end(), 0.0);
    const double avg_max_vel = opt.max_velocity.size() > 0 ? sum / static_cast<double>(opt.max_velocity.size()) : 1.0;
    const double path_length = static_cast<double>(traj.path_.length());
    const double fake_duration = (avg_max_vel > 0.0) ? path_length / avg_max_vel : 1.0;
    traj.duration_ = seconds{fake_duration};

    return traj;
}

struct trajectory::sample trajectory::sample(trajectory::seconds t) const {
    if (t < trajectory::seconds{0.0} || t > duration_) {
        throw std::out_of_range{"Time out of trajectory bounds"};
    }

    // TODO(acm): Implement sampling
    // - Look up s(t) from time parameterization
    // - Query path for q(s), q̇(s), q̈(s)
    // - Apply chain rule to get q(t), q̇(t), q̈(t)

    // TEMPORARY STUB: Fake linear time-to-arc-length mapping
    // Pretend we move at constant velocity along path
    const double fraction = (duration_.count() > 0.0) ? t.count() / duration_.count() : 0.0;
    const arc_length s{static_cast<double>(path_.length()) * fraction};

    // Get configuration from path
    auto config = path_.configuration(s);

    // Fake velocity and acceleration (zeros for now)
    auto vel = xt::zeros<double>({config.shape(0)});
    auto accel = xt::zeros<double>({config.shape(0)});

    return {.time = t, .configuration = std::move(config), .velocity = vel, .acceleration = accel};
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

trajectory::cursor trajectory::create_cursor() const {
    return cursor{this};
}

// Note: trajectory::samples() is a template method defined in the header

// trajectory::cursor implementations

trajectory::cursor::cursor(const class trajectory* traj) : traj_{traj} {}

const trajectory& trajectory::cursor::trajectory() const noexcept {
    return *traj_;
}

trajectory::seconds trajectory::cursor::time() const noexcept {
    return time_;
}

struct trajectory::sample trajectory::cursor::sample() const {
    return traj_->sample(time_);
}

void trajectory::cursor::advance_to(seconds t) {
    time_ = t;
    // TODO(acm): Update hints when time parameterization is implemented
}

void trajectory::cursor::advance_by(seconds dt) {
    time_ += dt;
    // TODO(acm): Update hints when time parameterization is implemented
}

}  // namespace viam::trajex::totg

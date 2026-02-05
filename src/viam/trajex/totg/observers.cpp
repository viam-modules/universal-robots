#include <viam/trajex/totg/observers.hpp>

#include <stdexcept>

namespace viam::trajex::totg {

namespace {

class reentrancy_guard {
   public:
    explicit reentrancy_guard(bool& flag) : flag_(flag) {
        if (flag_) {
            throw std::runtime_error("Re-entrant call to composite_integration_observer detected");
        }
        flag_ = true;
    }

    ~reentrancy_guard() {
        flag_ = false;
    }

    // Non-copyable, non-movable
    reentrancy_guard(const reentrancy_guard&) = delete;
    reentrancy_guard& operator=(const reentrancy_guard&) = delete;
    reentrancy_guard(reentrancy_guard&&) = delete;
    reentrancy_guard& operator=(reentrancy_guard&&) = delete;

   private:
    bool& flag_;
};

}  // namespace

composite_integration_observer::composite_integration_observer() = default;
composite_integration_observer::~composite_integration_observer() = default;

std::shared_ptr<trajectory::integration_observer> composite_integration_observer::add_observer(
    std::shared_ptr<integration_observer> observer) {
    if (!observer) {
        throw std::invalid_argument("Cannot add null observer to composite_integration_observer");
    }
    const reentrancy_guard guard(dispatching_);
    observers_.push_back(observer);
    return observer;
}

void composite_integration_observer::on_started_forward_integration(const trajectory& traj, started_forward_event event) {
    const reentrancy_guard guard(dispatching_);
    for (auto& obs : observers_) {
        obs->on_started_forward_integration(traj, event);
    }
}

void composite_integration_observer::on_hit_limit_curve(const trajectory& traj, limit_hit_event event) {
    const reentrancy_guard guard(dispatching_);
    for (auto& obs : observers_) {
        obs->on_hit_limit_curve(traj, event);
    }
}

void composite_integration_observer::on_started_backward_integration(const trajectory& traj, started_backward_event event) {
    const reentrancy_guard guard(dispatching_);
    for (auto& obs : observers_) {
        obs->on_started_backward_integration(traj, event);
    }
}

void composite_integration_observer::on_trajectory_extended(const trajectory& traj, splice_event event) {
    const reentrancy_guard guard(dispatching_);
    for (auto& obs : observers_) {
        obs->on_trajectory_extended(traj, event);
    }
}

trajectory_integration_event_collector::trajectory_integration_event_collector() = default;
trajectory_integration_event_collector::~trajectory_integration_event_collector() = default;

void trajectory_integration_event_collector::on_event(const class trajectory&, event ev) {
    events_.push_back(std::move(ev));
}

const std::vector<trajectory_integration_event_collector::event>& trajectory_integration_event_collector::events() const {
    return events_;
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::cbegin() const noexcept {
    using std::cbegin;
    return cbegin(events());
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::cend() const noexcept {
    using std::cend;
    return cend(events());
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::begin() const noexcept {
    return cbegin();
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::end() const noexcept {
    return cend();
}

}  // namespace viam::trajex::totg

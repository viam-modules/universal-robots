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

void composite_integration_observer::add_observer_(std::shared_ptr<integration_observer> observer) {
    if (!observer) {
        throw std::invalid_argument("Cannot add null observer to composite_integration_observer");
    }
    const reentrancy_guard guard(dispatching_);
    observers_.push_back(std::move(observer));
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

void composite_integration_observer::on_failed(std::exception_ptr error,
                                               std::shared_ptr<const trajectory> partial_traj) noexcept {
    // No reentrancy_guard: it throws, which is forbidden in a noexcept context.
    // on_failed is a terminal event fired once during stack unwinding; re-entrant calls won't occur.
    for (auto& obs : observers_) {
        obs->on_failed(error, partial_traj);
    }
}

trajectory_integration_event_collector::trajectory_integration_event_collector() = default;
trajectory_integration_event_collector::~trajectory_integration_event_collector() = default;

void trajectory_integration_event_collector::on_event(const class trajectory&, event ev) {
    events_.push_back(std::move(ev));
}

void trajectory_integration_event_collector::on_failed(std::exception_ptr error,
                                                        std::shared_ptr<const trajectory> partial_traj) noexcept {
    try {
        if (error) {
            try {
                std::rethrow_exception(error);
            } catch (const std::exception& e) {
                failure_error_ = e.what();
            } catch (...) {
                failure_error_ = "unknown exception";
            }
        }
        failed_trajectory_ = std::move(partial_traj);
    } catch (...) {
        // Swallow anything that escapes the inner block (e.g. std::bad_alloc storing
        // the shared_ptr). Record what we have and move on - better than terminate.
        if (failure_error_.empty())
            failure_error_ = "unknown exception (on_failed capture failed)";
    }
}

bool trajectory_integration_event_collector::has_failure() const noexcept {
    return !failure_error_.empty();
}

const trajectory* trajectory_integration_event_collector::failed_trajectory() const noexcept {
    return failed_trajectory_.get();
}

const std::string& trajectory_integration_event_collector::failure_error() const noexcept {
    return failure_error_;
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

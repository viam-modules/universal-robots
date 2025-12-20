#include <stdexcept>
#include <viam/trajex/totg/observers.hpp>
#include "viam/trajex/totg/trajectory.hpp"

namespace viam::trajex::totg {

trajectory_integration_event_collector::trajectory_integration_event_collector() = default;
trajectory_integration_event_collector::~trajectory_integration_event_collector() = default;

void trajectory_integration_event_collector::on_event(const class trajectory& traj, event ev) {
    events_.push_back(std::move(ev));
}

const std::vector<trajectory_integration_event_collector::event>& trajectory_integration_event_collector::events() const {
    return events_;
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::cbebin() const noexcept {
    using std::begin;
    return begin(events());
}

trajectory_integration_event_collector::const_iterator trajectory_integration_event_collector::cend() const noexcept {
    using std::end;
    return end(events());
};

}  // namespace viam::trajex::totg

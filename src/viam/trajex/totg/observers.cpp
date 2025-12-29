#include <viam/trajex/totg/observers.hpp>

namespace viam::trajex::totg {

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

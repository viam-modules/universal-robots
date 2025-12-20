#pragma once

#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

class trajectory_integration_event_collector final : public trajectory::integration_event_observer {
   public:
    trajectory_integration_event_collector();
    ~trajectory_integration_event_collector();

    void on_event(const trajectory& traj, event ev) override;

    const std::vector<event>& events() const;

    using const_iterator = std::vector<event>::const_iterator;

    const_iterator cbebin() const noexcept;
    const_iterator cend() const noexcept;

   private:
    std::vector<event> events_;
};

}  // namespace viam::trajex::totg

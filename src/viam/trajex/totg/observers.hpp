#pragma once

#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

///
/// Observer that collects all integration events for later inspection.
///
/// Stores all trajectory integration events (forward starts, limit hits,
/// backward starts, splices) in order of occurrence. Useful for testing,
/// debugging, and analyzing trajectory generation behavior.
///
/// @code
/// trajectory_integration_event_collector collector;
/// auto traj = trajectory::create(path, {.observer = &collector});
/// for (const auto& ev : collector.events()) {
///     // Inspect each event
/// }
/// @endcode
///
class trajectory_integration_event_collector final : public trajectory::integration_event_observer {
   public:
    ///
    /// Constructs an empty event collector.
    ///
    trajectory_integration_event_collector();

    ///
    /// Destructor.
    ///
    ~trajectory_integration_event_collector();

    ///
    /// Appends an event to the collection.
    ///
    /// @param traj Trajectory being integrated
    /// @param ev Event to store
    ///
    void on_event(const trajectory& traj, event ev) override;

    ///
    /// Gets all collected events.
    ///
    /// @return Vector of events in order of occurrence
    ///
    const std::vector<event>& events() const;

    using const_iterator = std::vector<event>::const_iterator;

    ///
    /// Gets const iterator to beginning of events.
    ///
    /// @return Iterator to first event
    ///
    const_iterator cbegin() const noexcept;

    ///
    /// Gets const iterator to end of events.
    ///
    /// @return Iterator past last event
    ///
    const_iterator cend() const noexcept;

    ///
    /// Gets const iterator to beginning of events.
    ///
    /// @return Iterator to first event
    ///
    const_iterator begin() const noexcept;

    ///
    /// Gets const iterator to end of events.
    ///
    /// @return Iterator past last event
    ///
    const_iterator end() const noexcept;

   private:
    std::vector<event> events_;
};

}  // namespace viam::trajex::totg

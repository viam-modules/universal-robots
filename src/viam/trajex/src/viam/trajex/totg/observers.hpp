#pragma once

#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

///
/// Observer that dispatches integration events to multiple observers.
///
/// Allows attaching multiple observers to a single trajectory generation,
/// useful for combining expectation checking with data collection for
/// visualization or debugging.
///
/// @code
/// composite_integration_observer composite;
/// composite.add_observer(std::make_shared<some_integration_observer>());
/// composite.add_observer(std::make_shared<another_integration_observer>());
/// options.observer = &composite;
/// @endcode
///
class composite_integration_observer final : public trajectory::integration_observer {
   public:
    ///
    /// Constructs an empty composite observer.
    ///
    composite_integration_observer();

    ///
    /// Destructor.
    ///
    ~composite_integration_observer() override;

    ///
    /// Adds an observer to receive events.
    ///
    /// Observers are called in the order they are added. Returns the observer
    /// to allow capturing it when adding inline.
    ///
    /// @tparam T Observer type, must derive from integration_observer
    /// @param observer Observer to add
    /// @return The observer that was added
    /// @throws std::invalid_argument if observer is null
    /// @throws std::runtime_error if called while dispatching events (re-entrant call)
    ///
    template <std::derived_from<integration_observer> T>
    std::shared_ptr<T> add_observer(std::shared_ptr<T> observer);

    ///
    /// Dispatches forward integration start to all observers.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Forward integration start event
    ///
    void on_started_forward_integration(const trajectory& traj, started_forward_event event) override;

    ///
    /// Dispatches limit curve hit to all observers.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Limit curve hit event
    ///
    void on_hit_limit_curve(const trajectory& traj, limit_hit_event event) override;

    ///
    /// Dispatches backward integration start to all observers.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Backward integration start event
    ///
    void on_started_backward_integration(const trajectory& traj, started_backward_event event) override;

    ///
    /// Dispatches trajectory splice to all observers.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Trajectory splice event
    ///
    void on_trajectory_extended(const trajectory& traj, splice_event event) override;

    ///
    /// Dispatches failure notification to all observers.
    ///
    /// @param error Exception that caused the failure
    /// @param partial_traj Partial trajectory at time of failure, or null if unavailable
    ///
    void on_failed(std::exception_ptr error,
                   std::shared_ptr<const trajectory> partial_traj) noexcept override;

   private:
    void add_observer_(std::shared_ptr<integration_observer> observer);

    std::vector<std::shared_ptr<integration_observer>> observers_;
    bool dispatching_ = false;
};

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

    ///
    /// Called when trajectory generation fails. Stores partial trajectory and error
    /// message for later diagnostic use (e.g., writing failure JSON with limit curves).
    ///
    void on_failed(std::exception_ptr error,
                   std::shared_ptr<const trajectory> partial_traj) noexcept override;

    ///
    /// Returns true if trajectory generation failed and partial data is available.
    ///
    bool has_failure() const noexcept;

    ///
    /// Gets the partial trajectory from a failed generation, or null if unavailable.
    ///
    const trajectory* failed_trajectory() const noexcept;

    ///
    /// Gets the error message from a failed generation.
    ///
    const std::string& failure_error() const noexcept;

   private:
    std::vector<event> events_;
    std::shared_ptr<const trajectory> failed_trajectory_;
    std::string failure_error_;
};

template <std::derived_from<trajectory::integration_observer> T>
std::shared_ptr<T> composite_integration_observer::add_observer(std::shared_ptr<T> observer) {
    add_observer_(observer);
    return observer;
}

}  // namespace viam::trajex::totg

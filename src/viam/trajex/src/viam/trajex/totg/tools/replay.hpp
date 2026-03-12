#pragma once

#include <filesystem>
#include <iosfwd>
#include <memory>
#include <optional>

#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/tools/planner.hpp>
#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

///
/// Receiver for replay_planner. Holds the most recently generated
/// trajectory so callers can access it after execute() completes.
///
struct replay_receiver {
    std::optional<trajectory> traj;
};

///
/// Receiver for legacy_replay_planner. Holds the path and trajectory produced
/// by the legacy generator so callers can access them after execute() completes.
///
struct legacy_replay_receiver {
    std::optional<std::pair<Path, Trajectory>> result;
};

///
/// Replay planner for the TOTG algorithm.
///
/// Loads a canonical JSON replay record, runs TOTG with an attached event collector,
/// and provides the collector for downstream diagnostic JSON output.
///
/// Typical usage:
/// @code
///   auto p = replay_planner::create("failed.json");
///   auto outcome = p.execute([](const auto&, auto tx, const auto&) { return tx; });
///   write_trajectory_json(std::cout, p.collector(),
///                         outcome.receiver ? &outcome.receiver->traj.value() : nullptr);
/// @endcode
///
class replay_planner : public planner<replay_receiver> {
   public:
    ///
    /// Constructs a replay planner from a replay record stream.
    ///
    /// @param in Stream containing a canonical JSON replay record
    /// @throws std::runtime_error if the stream cannot be parsed or required fields are missing
    ///
    static replay_planner create(std::istream& in);

    ///
    /// Constructs a replay planner from a replay record file path.
    ///
    /// @param path Path to a canonical JSON replay record file
    /// @throws std::runtime_error if the file cannot be opened or parsed
    ///
    static replay_planner create(const std::filesystem::path& path);

    ///
    /// Returns the event collector populated during execute().
    ///
    const trajectory_integration_event_collector& collector() const noexcept;

   private:
    // The collector lives on the heap so its address is stable through moves.
    // mutable_config().observer holds a raw pointer to it; the pointer remains
    // valid as long as the planner is alive.
    std::unique_ptr<trajectory_integration_event_collector> collector_;

    explicit replay_planner(config cfg, std::unique_ptr<trajectory_integration_event_collector> collector);
};

///
/// Replay planner for the legacy trajectory algorithm.
///
/// Loads a canonical JSON replay record and runs the legacy generator. Primarily
/// useful for getting a debugger session on a known-bad trajectory.
///
class legacy_replay_planner : public planner<legacy_replay_receiver> {
   public:
    using planner<legacy_replay_receiver>::planner;

    ///
    /// Constructs a replay planner from a replay record stream.
    ///
    static legacy_replay_planner create(std::istream& in);

    ///
    /// Constructs a replay planner from a replay record file path.
    ///
    static legacy_replay_planner create(const std::filesystem::path& path);
};

}  // namespace viam::trajex::totg

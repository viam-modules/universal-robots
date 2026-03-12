#pragma once

#include <filesystem>
#include <iosfwd>
#include <memory>
#include <optional>

#include <viam/trajex/service/trajectory_planner.hpp>
#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex {

///
/// Receiver for trajex_replay_planner. Holds the most recently generated
/// trajectory so callers can access it after execute() completes.
///
struct trajex_replay_receiver {
    std::optional<totg::trajectory> traj;
};

///
/// Receiver for legacy_replay_planner. No-op accumulator; the legacy algorithm
/// is replayed for debugger access only.
///
struct legacy_replay_receiver {};

///
/// Replay planner for the trajex/TOTG algorithm.
///
/// Loads a canonical JSON replay record, runs TOTG with an attached event collector,
/// and provides the collector for downstream diagnostic JSON output.
///
/// Typical usage:
/// @code
///   auto planner = trajex_replay_planner::create("failed.json");
///   auto outcome = planner.execute([](const auto&, auto tx, auto) { return std::move(tx); });
///   write_trajectory_json(std::cout, planner.collector(),
///                         outcome.receiver ? &outcome.receiver->traj.value() : nullptr);
/// @endcode
///
class trajex_replay_planner : public trajectory_planner<trajex_replay_receiver> {
   public:
    ///
    /// Constructs a replay planner from a replay record stream.
    ///
    /// @param in Stream containing a canonical JSON replay record
    /// @throws std::runtime_error if the stream cannot be parsed or required fields are missing
    ///
    static trajex_replay_planner create(std::istream& in);

    ///
    /// Constructs a replay planner from a replay record file path.
    ///
    /// @param path Path to a canonical JSON replay record file
    /// @throws std::runtime_error if the file cannot be opened or parsed
    ///
    static trajex_replay_planner create(const std::filesystem::path& path);

    ///
    /// Returns the event collector populated during execute().
    ///
    const totg::trajectory_integration_event_collector& collector() const noexcept;

   private:
    // The collector lives on the heap so its address is stable through moves.
    // mutable_config().observer holds a raw pointer to it; the pointer remains
    // valid as long as the planner is alive.
    std::unique_ptr<totg::trajectory_integration_event_collector> collector_;

    explicit trajex_replay_planner(config cfg, std::unique_ptr<totg::trajectory_integration_event_collector> collector);
};

///
/// Replay planner for the legacy trajectory algorithm.
///
/// Loads a canonical JSON replay record and runs the legacy generator. Primarily
/// useful for getting a debugger session on a known-bad trajectory.
///
class legacy_replay_planner : public trajectory_planner<legacy_replay_receiver> {
   public:
    using trajectory_planner<legacy_replay_receiver>::trajectory_planner;

    ///
    /// Constructs a replay planner from a replay record stream.
    ///
    static legacy_replay_planner create(std::istream& in);

    ///
    /// Constructs a replay planner from a replay record file path.
    ///
    static legacy_replay_planner create(const std::filesystem::path& path);
};

}  // namespace viam::trajex

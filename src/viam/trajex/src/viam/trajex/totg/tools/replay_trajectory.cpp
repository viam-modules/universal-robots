// CLI tool: replay a canonical JSON trajectory replay record through the TOTG algorithm and
// emit diagnostic JSON for visualization.
//
// Usage:
//   trajex_replay_trajectory failed.json | python3 visualize.py
//   cat failed.json | trajex_replay_trajectory | python3 visualize.py

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <viam/trajex/totg/json_serialization.hpp>
#include <viam/trajex/totg/tools/replay_planners.hpp>

using viam::trajex::trajex_replay_planner;

int main(int argc, char* argv[]) try {
    trajex_replay_planner planner =
        (argc >= 2) ? trajex_replay_planner::create(std::filesystem::path(argv[1])) : trajex_replay_planner::create(std::cin);

    auto outcome = planner.execute([](const auto&, auto tx, const auto&) { return tx; });

    const viam::trajex::totg::trajectory* traj = nullptr;
    if (outcome.receiver && outcome.receiver->traj) {
        traj = &*outcome.receiver->traj;
    }

    viam::trajex::totg::write_trajectory_json(std::cout, planner.collector(), traj);
    return 0;
} catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
}

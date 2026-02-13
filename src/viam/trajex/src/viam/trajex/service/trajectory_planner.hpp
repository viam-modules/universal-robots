#pragma once

#include <chrono>
#include <exception>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <viam/trajex/service/colinearization.hpp>
#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>

#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>

namespace viam::trajex {

using totg::waypoint_accumulator;

///
/// Result of running a single algorithm across all segments.
///
/// The receiver is engaged when all segments complete successfully. If any
/// segment fails, the receiver is disengaged and the exception is captured.
///
template <typename Receiver>
struct algorithm_outcome {
    std::optional<Receiver> receiver;
    std::exception_ptr error;
};

///
/// Reusable trajectory planning workflow coordinator.
///
/// Captures the essential control flow for trajectory generation -- waypoint
/// preprocessing, segmentation, dual-algorithm execution, and result selection
/// -- while allowing each consumer (URArm, trajex service, Yaskawa) to inject
/// its own behaviors at well-defined customization points.
///
/// The template parameter `Receiver` is a user-defined type that accumulates
/// per-segment results during trajectory generation. Each enabled algorithm
/// gets its own receiver instance. Per-segment success callbacks fold results
/// into the receiver. The decider receives both receivers and picks the winner.
///
/// Single-use: construct, configure via builder methods, call execute(), discard.
///
/// @tparam Receiver Default-constructible type for accumulating per-segment results
///
template <typename Receiver>
class trajectory_planner {
   public:
    struct config {
        xt::xarray<double> velocity_limits;
        xt::xarray<double> acceleration_limits;
        double path_blend_tolerance;
        std::optional<double> colinearization_ratio;
        double max_trajectory_duration;
        bool segment_trajex = true;
    };

    explicit trajectory_planner(config cfg)
        : config_(std::move(cfg)) {}

    // -- Stash mechanism --

    ///
    /// Extends the lifetime of data created inside callbacks.
    ///
    /// Essential because waypoint_accumulator operates on views -- it does not
    /// own its data. When a callback creates an xt::xarray<double>, that array
    /// must outlive the waypoint_accumulator that views it. Stash stores it
    /// here. The returned shared_ptr can be used immediately; the data it
    /// points to remains valid until the planner is destroyed.
    ///
    template <typename T>
    std::shared_ptr<T> stash(T&& value) {
        auto ptr = std::make_shared<T>(std::forward<T>(value));
        stashed_.push_back(ptr);
        return ptr;
    }

    // -- Builder methods: workflow phases --

    ///
    /// Required. Returns the initial waypoint_accumulator.
    ///
    trajectory_planner& with_waypoint_provider(
        std::function<waypoint_accumulator(trajectory_planner&)> fn) {
        waypoint_provider_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Mutates the waypoint accumulator in place (e.g. deduplication).
    ///
    trajectory_planner& with_waypoint_preprocessor(
        std::function<void(trajectory_planner&, waypoint_accumulator&)> fn) {
        preprocessor_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Throws if waypoints violate some precondition.
    ///
    trajectory_planner& with_move_validator(
        std::function<void(trajectory_planner&, const waypoint_accumulator&)> fn) {
        validator_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Splits waypoints into segments. If omitted, the entire
    /// waypoint sequence is treated as a single segment.
    ///
    trajectory_planner& with_segmenter(
        std::function<std::vector<waypoint_accumulator>(trajectory_planner&, waypoint_accumulator)>
            fn) {
        segmenter_ = std::move(fn);
        return *this;
    }

    // -- Builder methods: algorithm registration --

    ///
    /// Enables the trajex/totg algorithm.
    ///
    trajectory_planner& with_trajex(
        std::function<void(Receiver&, const totg::trajectory&, std::chrono::duration<double>)>
            on_success,
        std::function<
            void(const Receiver&, const waypoint_accumulator&, const std::exception&)>
            on_failure = nullptr) {
        trajex_on_success_ = std::move(on_success);
        trajex_on_failure_ = std::move(on_failure);
        return *this;
    }

    ///
    /// Enables the legacy generator.
    ///
    trajectory_planner& with_legacy(
        std::function<void(Receiver&, const Trajectory&, std::chrono::duration<double>)>
            on_success,
        std::function<
            void(const Receiver&, const waypoint_accumulator&, const std::exception&)>
            on_failure = nullptr) {
        legacy_on_success_ = std::move(on_success);
        legacy_on_failure_ = std::move(on_failure);
        return *this;
    }

    // -- Query --

    ///
    /// Number of waypoints that survived preprocessing.
    ///
    /// Available after execute() has been called. Allows the decider to
    /// distinguish "nothing to do" (< 2 waypoints) from "both generators failed."
    ///
    std::size_t processed_waypoint_count() const noexcept {
        return processed_waypoint_count_;
    }

    // -- Terminal --

    ///
    /// Runs the full planning workflow and invokes the decider.
    ///
    /// @param decider Callable receiving (const trajectory_planner&,
    ///        algorithm_outcome<Receiver>, algorithm_outcome<Receiver>).
    ///        Return type is deduced.
    ///
    template <typename Decider>
    auto execute(Decider&& decider)
        -> std::invoke_result_t<Decider,
                                const trajectory_planner&,
                                algorithm_outcome<Receiver>,
                                algorithm_outcome<Receiver>> {
        if (!waypoint_provider_) {
            throw std::logic_error("waypoint provider not set");
        }

        // 1. Obtain waypoints
        auto wa = waypoint_provider_(*this);

        // 2. Preprocess
        if (preprocessor_) {
            preprocessor_(*this, wa);
        }

        // 3. Check waypoint count
        processed_waypoint_count_ = wa.size();
        if (processed_waypoint_count_ < 2) {
            return std::forward<Decider>(decider)(
                *this, algorithm_outcome<Receiver>{}, algorithm_outcome<Receiver>{});
        }

        // 4. Validate
        if (validator_) {
            validator_(*this, wa);
        }

        // 5. Segment
        // When segment_trajex is false, snapshot the current waypoints into owned
        // storage before the segmenter moves wa away. The snapshot is stashed for
        // lifetime, and a new accumulator views it.
        std::optional<std::vector<waypoint_accumulator>> unsegmented_for_trajex;
        if (!config_.segment_trajex && segmenter_) {
            auto snapshot = snapshot_waypoints_(wa);
            auto owned = stash(std::move(snapshot));
            unsegmented_for_trajex.emplace();
            unsegmented_for_trajex->push_back(waypoint_accumulator{*owned});
        }

        std::vector<waypoint_accumulator> segments;
        if (segmenter_) {
            segments = segmenter_(*this, std::move(wa));
        } else {
            segments.push_back(std::move(wa));
        }

        const auto& trajex_segments =
            unsegmented_for_trajex ? *unsegmented_for_trajex : segments;

        // 6. Run algorithms
        auto trajex_outcome = run_trajex_(trajex_segments);
        auto legacy_outcome = run_legacy_(segments);

        // 7. Decide
        return std::forward<Decider>(decider)(
            *this, std::move(trajex_outcome), std::move(legacy_outcome));
    }

   private:
    // -- Trajex algorithm loop --
    algorithm_outcome<Receiver> run_trajex_(
        const std::vector<waypoint_accumulator>& segments) {
        if (!trajex_on_success_) {
            return {};
        }

        algorithm_outcome<Receiver> outcome;
        outcome.receiver.emplace();
        double cumulative_duration = 0.0;

        for (const auto& segment : segments) {
            if (!outcome.receiver) {
                break;
            }

            try {
                auto path_opts = totg::path::options{}.set_max_deviation(
                    config_.path_blend_tolerance);

                auto p = totg::path::create(segment, path_opts);

                totg::trajectory::options topts;
                topts.max_velocity = config_.velocity_limits;
                topts.max_acceleration = config_.acceleration_limits;

                auto start = std::chrono::steady_clock::now();
                auto traj = totg::trajectory::create(std::move(p), std::move(topts));
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start);

                cumulative_duration += traj.duration().count();
                if (cumulative_duration > config_.max_trajectory_duration) {
                    throw std::runtime_error(
                        "trajectory duration exceeds maximum ("
                        + std::to_string(cumulative_duration) + "s > "
                        + std::to_string(config_.max_trajectory_duration) + "s)");
                }

                trajex_on_success_(*outcome.receiver, traj, elapsed);

            } catch (const std::exception& e) {
                outcome.error = std::current_exception();
                if (trajex_on_failure_) {
                    trajex_on_failure_(*outcome.receiver, segment, e);
                }
                outcome.receiver.reset();
            }
        }

        return outcome;
    }

    // -- Legacy algorithm loop --
    algorithm_outcome<Receiver> run_legacy_(
        const std::vector<waypoint_accumulator>& segments) {
        if (!legacy_on_success_) {
            return {};
        }

        algorithm_outcome<Receiver> outcome;
        outcome.receiver.emplace();
        double cumulative_duration = 0.0;

        for (const auto& segment : segments) {
            if (!outcome.receiver) {
                break;
            }

            try {
                // Convert waypoint_accumulator to list<Eigen::VectorXd>
                std::list<Eigen::VectorXd> eigen_waypoints;
                for (const auto& wp : segment) {
                    auto dof = static_cast<Eigen::Index>(segment.dof());
                    Eigen::VectorXd v(dof);
                    for (Eigen::Index j = 0; j < dof; ++j) {
                        v[j] = wp(static_cast<std::size_t>(j));
                    }
                    eigen_waypoints.push_back(std::move(v));
                }

                // Apply colinearization if configured
                if (config_.colinearization_ratio) {
                    apply_colinearization(
                        eigen_waypoints,
                        config_.path_blend_tolerance * *config_.colinearization_ratio);
                }

                // Construct legacy path and trajectory
                Path legacy_path(eigen_waypoints, config_.path_blend_tolerance);

                // Convert limits to Eigen
                auto vel_eigen = xt_to_eigen_(config_.velocity_limits);
                auto acc_eigen = xt_to_eigen_(config_.acceleration_limits);

                auto start = std::chrono::steady_clock::now();
                Trajectory traj(legacy_path, vel_eigen, acc_eigen);
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start);

                if (!traj.isValid()) {
                    throw std::runtime_error("legacy trajectory generator produced invalid result");
                }

                cumulative_duration += traj.getDuration();
                if (cumulative_duration > config_.max_trajectory_duration) {
                    throw std::runtime_error(
                        "trajectory duration exceeds maximum ("
                        + std::to_string(cumulative_duration) + "s > "
                        + std::to_string(config_.max_trajectory_duration) + "s)");
                }

                legacy_on_success_(*outcome.receiver, traj, elapsed);

            } catch (const std::exception& e) {
                outcome.error = std::current_exception();
                if (legacy_on_failure_) {
                    legacy_on_failure_(*outcome.receiver, segment, e);
                }
                outcome.receiver.reset();
            }
        }

        return outcome;
    }

    // Copy waypoint views into an owned 2D xarray
    static xt::xarray<double> snapshot_waypoints_(const waypoint_accumulator& wa) {
        std::vector<std::size_t> shape = {wa.size(), wa.dof()};
        xt::xarray<double> result(shape);
        for (std::size_t i = 0; i < wa.size(); ++i) {
            xt::view(result, i, xt::all()) = wa[i];
        }
        return result;
    }

    // Convert xt::xarray<double> to Eigen::VectorXd
    static Eigen::VectorXd xt_to_eigen_(const xt::xarray<double>& xa) {
        auto sz = static_cast<Eigen::Index>(xa.size());
        Eigen::VectorXd v(sz);
        for (Eigen::Index i = 0; i < sz; ++i) {
            v[i] = xa.flat(static_cast<std::size_t>(i));
        }
        return v;
    }

    config config_;
    std::size_t processed_waypoint_count_ = 0;

    // Stashed data for lifetime extension
    std::vector<std::shared_ptr<void>> stashed_;

    // Workflow callbacks
    std::function<waypoint_accumulator(trajectory_planner&)> waypoint_provider_;
    std::function<void(trajectory_planner&, waypoint_accumulator&)> preprocessor_;
    std::function<void(trajectory_planner&, const waypoint_accumulator&)> validator_;
    std::function<std::vector<waypoint_accumulator>(trajectory_planner&, waypoint_accumulator)>
        segmenter_;

    // Trajex callbacks
    std::function<void(Receiver&, const totg::trajectory&, std::chrono::duration<double>)>
        trajex_on_success_;
    std::function<void(const Receiver&, const waypoint_accumulator&, const std::exception&)>
        trajex_on_failure_;

    // Legacy callbacks
    std::function<void(Receiver&, const Trajectory&, std::chrono::duration<double>)>
        legacy_on_success_;
    std::function<void(const Receiver&, const waypoint_accumulator&, const std::exception&)>
        legacy_on_failure_;
};

}  // namespace viam::trajex

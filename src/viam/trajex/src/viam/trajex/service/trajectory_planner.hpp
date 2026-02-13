#pragma once

#include <chrono>
#include <exception>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <ranges>
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

///
/// Non-template base holding config and types shared across all trajectory_planner
/// instantiations regardless of Receiver type.
///
struct trajectory_planner_base {
    struct config {
        xt::xarray<double> velocity_limits;
        xt::xarray<double> acceleration_limits;
        double path_blend_tolerance = 0.0;
        std::optional<double> colinearization_ratio;
        double max_trajectory_duration = 0.0;
        bool segment_trajex = true;
    };
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
class trajectory_planner : public trajectory_planner_base {
   public:
    using config = trajectory_planner_base::config;

    ///
    /// Result of running a single algorithm across all segments.
    ///
    /// The receiver is engaged when all segments complete successfully. If any
    /// segment fails, the receiver is disengaged and the exception is captured.
    ///
    struct algorithm_outcome {
        std::optional<Receiver> receiver;
        std::exception_ptr error;
    };

    /// @name Handler type aliases
    /// @{
    using waypoint_provider_fn = std::function<totg::waypoint_accumulator(trajectory_planner&)>;

    using waypoint_preprocessor_fn = std::function<void(trajectory_planner&, totg::waypoint_accumulator&)>;

    using move_validator_fn = std::function<void(trajectory_planner&, const totg::waypoint_accumulator&)>;

    using segmenter_fn = std::function<std::vector<totg::waypoint_accumulator>(trajectory_planner&, totg::waypoint_accumulator)>;

    using trajex_success_fn =
        std::function<void(Receiver&, const totg::waypoint_accumulator&, const totg::trajectory&, std::chrono::duration<double>)>;

    using legacy_success_fn =
        std::function<void(Receiver&, const totg::waypoint_accumulator&, const Path&, const Trajectory&, std::chrono::duration<double>)>;

    using algorithm_failure_fn = std::function<void(const Receiver&, const totg::waypoint_accumulator&, const std::exception&)>;
    /// @}

    explicit trajectory_planner(config cfg) : config_(std::move(cfg)) {}

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

    /// @name Workflow phase registration
    /// @{

    ///
    /// Required. Returns the initial waypoint_accumulator.
    ///
    trajectory_planner& with_waypoint_provider(waypoint_provider_fn fn) {
        waypoint_provider_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Mutates the waypoint accumulator in place (e.g. deduplication).
    ///
    trajectory_planner& with_waypoint_preprocessor(waypoint_preprocessor_fn fn) {
        preprocessor_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Throws if waypoints violate some precondition.
    ///
    trajectory_planner& with_move_validator(move_validator_fn fn) {
        validator_ = std::move(fn);
        return *this;
    }

    ///
    /// Optional. Splits waypoints into segments. If omitted, the entire
    /// waypoint sequence is treated as a single segment.
    ///
    trajectory_planner& with_segmenter(segmenter_fn fn) {
        segmenter_ = std::move(fn);
        return *this;
    }

    /// @}

    /// @name Algorithm registration
    /// @{

    ///
    /// Enables the trajex/totg algorithm.
    ///
    trajectory_planner& with_trajex(trajex_success_fn on_success, algorithm_failure_fn on_failure = nullptr) {
        trajex_on_success_ = std::move(on_success);
        trajex_on_failure_ = std::move(on_failure);
        return *this;
    }

    ///
    /// Enables the legacy generator.
    ///
    trajectory_planner& with_legacy(legacy_success_fn on_success, algorithm_failure_fn on_failure = nullptr) {
        legacy_on_success_ = std::move(on_success);
        legacy_on_failure_ = std::move(on_failure);
        return *this;
    }

    /// @}

    ///
    /// Number of waypoints that survived preprocessing.
    ///
    /// Available after execute() has been called. Allows the decider to
    /// distinguish "nothing to do" (< 2 waypoints) from "both generators failed."
    ///
    std::size_t processed_waypoint_count() const noexcept {
        return processed_waypoint_count_;
    }

    ///
    /// Runs the full planning workflow and invokes the decider.
    ///
    /// @param decider Callable receiving (const trajectory_planner&,
    ///        algorithm_outcome, algorithm_outcome). Return type is deduced.
    ///
    template <typename Decider>
    auto execute(Decider&& decider) -> std::invoke_result_t<Decider, const trajectory_planner&, algorithm_outcome, algorithm_outcome> {
        if (!waypoint_provider_) {
            throw std::logic_error("waypoint provider not set");
        }

        auto accumulator = waypoint_provider_(*this);

        if (preprocessor_) {
            preprocessor_(*this, accumulator);
        }

        processed_waypoint_count_ = accumulator.size();
        if (processed_waypoint_count_ < 2) {
            return std::forward<Decider>(decider)(*this, algorithm_outcome{}, algorithm_outcome{});
        }

        if (validator_) {
            validator_(*this, accumulator);
        }

        // When segment_trajex is false, copy the accumulator before the segmenter
        // consumes it. The copy is used as a single unsegmented input for trajex.
        std::optional<std::vector<totg::waypoint_accumulator>> unsegmented_for_trajex;
        if (!config_.segment_trajex && segmenter_) {
            unsegmented_for_trajex.emplace();
            unsegmented_for_trajex->push_back(accumulator);
        }

        std::vector<totg::waypoint_accumulator> segments;
        if (segmenter_) {
            segments = segmenter_(*this, std::move(accumulator));
        } else {
            segments.push_back(std::move(accumulator));
        }

        const auto& trajex_segments = unsegmented_for_trajex ? *unsegmented_for_trajex : segments;

        auto trajex_outcome = run_trajex_(trajex_segments);
        auto legacy_outcome = run_legacy_(segments);

        return std::forward<Decider>(decider)(*this, std::move(trajex_outcome), std::move(legacy_outcome));
    }

   private:
    algorithm_outcome run_trajex_(const std::vector<totg::waypoint_accumulator>& segments) {
        if (!trajex_on_success_) {
            return {};
        }

        algorithm_outcome outcome;
        outcome.receiver.emplace();
        double cumulative_duration = 0.0;

        for (const auto& segment : segments) {
            if (!outcome.receiver) {
                break;
            }

            try {
                auto path_opts = totg::path::options{}.set_max_deviation(config_.path_blend_tolerance);

                auto p = totg::path::create(segment, path_opts);

                totg::trajectory::options topts;
                topts.max_velocity = config_.velocity_limits;
                topts.max_acceleration = config_.acceleration_limits;

                auto start = std::chrono::steady_clock::now();
                auto traj = totg::trajectory::create(std::move(p), std::move(topts));
                auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);

                cumulative_duration += traj.duration().count();
                if (cumulative_duration > config_.max_trajectory_duration) {
                    throw std::runtime_error("trajectory duration exceeds maximum (" + std::to_string(cumulative_duration) + "s > " +
                                             std::to_string(config_.max_trajectory_duration) + "s)");
                }

                trajex_on_success_(*outcome.receiver, segment, traj, elapsed);

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

    algorithm_outcome run_legacy_(const std::vector<totg::waypoint_accumulator>& segments) {
        if (!legacy_on_success_) {
            return {};
        }

        algorithm_outcome outcome;
        outcome.receiver.emplace();
        double cumulative_duration = 0.0;

        for (const auto& segment : segments) {
            if (!outcome.receiver) {
                break;
            }

            try {
                auto start = std::chrono::steady_clock::now();

                // Zero-copy Eigen::Map transform of waypoint views, then copy
                // into the list the legacy Path constructor requires.
                constexpr auto to_eigen = [](const auto& view) {
                    const auto* const data = view.data() + view.data_offset();
                    const auto size = static_cast<Eigen::Index>(view.shape(0));
                    return Eigen::Map<const Eigen::VectorXd>(data, size);
                };
                auto transformed = segment | std::views::transform(to_eigen);
                std::list<Eigen::VectorXd> eigen_waypoints(std::begin(transformed), std::end(transformed));

                if (config_.colinearization_ratio) {
                    apply_colinearization(eigen_waypoints, config_.path_blend_tolerance * *config_.colinearization_ratio);
                }

                Path legacy_path(eigen_waypoints, config_.path_blend_tolerance);

                auto vel_eigen = Eigen::Map<const Eigen::VectorXd>(config_.velocity_limits.data(),
                                                                   static_cast<Eigen::Index>(config_.velocity_limits.size()));
                auto acc_eigen = Eigen::Map<const Eigen::VectorXd>(config_.acceleration_limits.data(),
                                                                   static_cast<Eigen::Index>(config_.acceleration_limits.size()));

                Trajectory traj(legacy_path, vel_eigen, acc_eigen);
                auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start);

                if (!traj.isValid()) {
                    throw std::runtime_error("legacy trajectory generator produced invalid result");
                }

                cumulative_duration += traj.getDuration();
                if (cumulative_duration > config_.max_trajectory_duration) {
                    throw std::runtime_error("trajectory duration exceeds maximum (" + std::to_string(cumulative_duration) + "s > " +
                                             std::to_string(config_.max_trajectory_duration) + "s)");
                }

                legacy_on_success_(*outcome.receiver, segment, legacy_path, traj, elapsed);

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

    config config_;
    std::size_t processed_waypoint_count_ = 0;
    std::vector<std::shared_ptr<void>> stashed_;

    waypoint_provider_fn waypoint_provider_;
    waypoint_preprocessor_fn preprocessor_;
    move_validator_fn validator_;
    segmenter_fn segmenter_;

    trajex_success_fn trajex_on_success_;
    algorithm_failure_fn trajex_on_failure_;

    legacy_success_fn legacy_on_success_;
    algorithm_failure_fn legacy_on_failure_;
};

}  // namespace viam::trajex

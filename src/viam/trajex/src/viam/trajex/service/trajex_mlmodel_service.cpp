#include <viam/trajex/service/trajex_mlmodel_service.hpp>

#include <algorithm>
#include <iterator>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <boost/variant/get.hpp>

#include <viam/sdk/log/logging.hpp>

#include <viam/trajex/service/sampling_utils.hpp>
#include <viam/trajex/service/trajectory_planner.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>
#include <viam/trajex/types/hertz.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

namespace viam::trajex {

namespace {

namespace vsdk = ::viam::sdk;

// Result type for the trajectory planner. Accumulates samples as flat vectors
// for zero-copy output packing.
struct service_result {
    std::size_t dof = 0;
    std::vector<double> times;
    std::vector<double> configurations;
    std::vector<double> velocities;
    std::optional<std::vector<double>> accelerations;
    double total_duration = 0.0;
};

// Holds owned output data and the named_tensor_views that reference it.
// Returned via an aliasing shared_ptr so the views remain valid for the caller.
struct infer_result {
    service_result data;
    trajex_mlmodel_service::named_tensor_views views;
};

// Extract a required tensor_view<double> from the input map.
const auto& get_double_tensor(const trajex_mlmodel_service::named_tensor_views& inputs, const std::string& name) {
    const auto it = inputs.find(name);
    if (it == inputs.end()) {
        throw std::invalid_argument("missing required input tensor: " + name);
    }
    const auto* view = boost::get<trajex_mlmodel_service::tensor_view<double>>(&it->second);
    if (!view) {
        throw std::invalid_argument("input tensor '" + name + "' has wrong type (expected float64)");
    }
    return *view;
}

// Extract a required tensor_view<int64_t> from the input map.
const auto& get_int64_tensor(const trajex_mlmodel_service::named_tensor_views& inputs, const std::string& name) {
    const auto it = inputs.find(name);
    if (it == inputs.end()) {
        throw std::invalid_argument("missing required input tensor: " + name);
    }
    const auto* view = boost::get<trajex_mlmodel_service::tensor_view<std::int64_t>>(&it->second);
    if (!view) {
        throw std::invalid_argument("input tensor '" + name + "' has wrong type (expected int64)");
    }
    return *view;
}

// Extract a scalar double from a shape-[1] tensor.
double get_scalar_double(const trajex_mlmodel_service::named_tensor_views& inputs, const std::string& name) {
    const auto& view = get_double_tensor(inputs, name);
    if (view.size() != 1) {
        throw std::invalid_argument("input tensor '" + name + "' must be a scalar (shape [1])");
    }
    return view.flat(0);
}

}  // namespace

trajex_mlmodel_service::trajex_mlmodel_service(vsdk::Dependencies deps, vsdk::ResourceConfig config) : MLModelService(config.name()) {
    reconfigure(deps, config);
}

void trajex_mlmodel_service::reconfigure(const vsdk::Dependencies&, const vsdk::ResourceConfig& cfg) {
    config new_config;

    // Parse generator_sequence
    auto seq_attr = cfg.attributes().find("generator_sequence");
    if (seq_attr != cfg.attributes().end()) {
        const auto* arr = seq_attr->second.get<std::vector<vsdk::ProtoValue>>();
        if (!arr || arr->empty()) {
            throw std::invalid_argument("generator_sequence must be a non-empty array of strings");
        }
        new_config.generator_sequence.clear();
        for (const auto& elem : *arr) {
            const auto* str = elem.get<std::string>();
            if (!str) {
                throw std::invalid_argument("generator_sequence entries must be strings");
            }
            if (*str != "totg" && *str != "legacy") {
                throw std::invalid_argument("generator_sequence: unknown algorithm '" + *str + "' (expected 'totg' or 'legacy')");
            }
            new_config.generator_sequence.push_back(*str);
        }
    }

    // Parse segment_for_totg
    auto seg_attr = cfg.attributes().find("segment_for_totg");
    if (seg_attr != cfg.attributes().end()) {
        const auto* val = seg_attr->second.get<bool>();
        if (!val) {
            throw std::invalid_argument("segment_for_totg must be a boolean");
        }
        new_config.segment_for_totg = *val;
    }

    const std::unique_lock lock{config_mutex_};
    config_ = std::move(new_config);
}

std::shared_ptr<trajex_mlmodel_service::named_tensor_views> trajex_mlmodel_service::infer(const named_tensor_views& inputs,
                                                                                          const vsdk::ProtoStruct&) {
    // Snapshot config under the read lock, then release
    config local_config;
    {
        const std::shared_lock lock{config_mutex_};
        local_config = config_;
    }

    // Parse inputs
    const auto& waypoints_view = get_double_tensor(inputs, "waypoints_rads");
    if (waypoints_view.dimension() != 2) {
        throw std::invalid_argument("waypoints_rads must be 2-dimensional [n_waypoints, n_dof]");
    }

    const auto& velocity_limits_view = get_double_tensor(inputs, "velocity_limits_rads_per_sec");
    const auto& acceleration_limits_view = get_double_tensor(inputs, "acceleration_limits_rads_per_sec2");

    // Derive DOF from velocity limits and validate consistency
    const auto dof = velocity_limits_view.size();
    if (acceleration_limits_view.size() != dof) {
        throw std::invalid_argument("acceleration_limits size (" + std::to_string(acceleration_limits_view.size()) +
                                    ") does not match velocity_limits size (" + std::to_string(dof) + ")");
    }
    if (waypoints_view.shape(1) != dof) {
        throw std::invalid_argument("waypoints DOF (" + std::to_string(waypoints_view.shape(1)) + ") does not match limits DOF (" +
                                    std::to_string(dof) + ")");
    }

    const double path_tolerance = get_scalar_double(inputs, "path_tolerance_delta_rads");
    const double colinearization_ratio_val = get_scalar_double(inputs, "path_colinearization_ratio");
    const std::optional<double> colinearization_ratio =
        colinearization_ratio_val > 0.0 ? std::optional{colinearization_ratio_val} : std::nullopt;
    const double dedup_tolerance = get_scalar_double(inputs, "waypoint_deduplication_tolerance_rads");

    const auto& sampling_freq_view = get_int64_tensor(inputs, "trajectory_sampling_freq_hz");
    if (sampling_freq_view.size() != 1) {
        throw std::invalid_argument("trajectory_sampling_freq_hz must be a scalar (shape [1])");
    }
    const auto sampling_freq = static_cast<double>(sampling_freq_view.flat(0));

    // Copy inputs into owned xtensor arrays for the planner
    xt::xarray<double> velocity_limits(velocity_limits_view);
    xt::xarray<double> acceleration_limits(acceleration_limits_view);

    // Build the planner
    auto planner = trajectory_planner<service_result>({
        .velocity_limits = std::move(velocity_limits),
        .acceleration_limits = std::move(acceleration_limits),
        .path_blend_tolerance = path_tolerance,
        .colinearization_ratio = colinearization_ratio,
        .segment_trajex = local_config.segment_for_totg,
    });

    planner
        .with_waypoint_provider([&](auto& p) {
            // TODO(zero-copy): waypoint_accumulator should accept tensor views
            // directly to avoid this copy for large waypoint sets.
            auto wp = p.stash(xt::xarray<double>(waypoints_view));
            return totg::waypoint_accumulator{*wp};
        })
        .with_waypoint_preprocessor([dedup_tolerance](auto&, totg::waypoint_accumulator& accumulator) {
            accumulator = totg::deduplicate_waypoints(accumulator, dedup_tolerance);
        })
        .with_segmenter([](auto&, totg::waypoint_accumulator accumulator) { return totg::segment_at_reversals(std::move(accumulator)); });

    // Register algorithms based on configured sequence
    for (const auto& algo : local_config.generator_sequence) {
        if (algo == "totg") {
            planner.with_trajex([&, dof](service_result& acc, const totg::waypoint_accumulator&, const totg::trajectory& traj, auto) {
                acc.dof = dof;
                acc.total_duration += traj.duration().count();
                if (!acc.accelerations) {
                    acc.accelerations.emplace();
                }

                auto sampler = totg::uniform_sampler::quantized_for_trajectory(traj, types::hertz{sampling_freq});
                for (const auto& sample : traj.samples(sampler) | std::views::drop(1)) {
                    acc.times.push_back(sample.time.count());
                    std::ranges::copy(sample.configuration, std::back_inserter(acc.configurations));
                    std::ranges::copy(sample.velocity, std::back_inserter(acc.velocities));
                    std::ranges::copy(sample.acceleration, std::back_inserter(*acc.accelerations));
                }
            });
        } else if (algo == "legacy") {
            planner.with_legacy(
                [&, dof](service_result& acc, const totg::waypoint_accumulator&, const Path&, const Trajectory& traj, auto) {
                    acc.dof = dof;
                    acc.total_duration += traj.getDuration();

                    for_each_sample(traj.getDuration(), sampling_freq, [&](double t, double) {
                        acc.times.push_back(t);
                        auto p = traj.getPosition(t);
                        auto v = traj.getVelocity(t);
                        for (Eigen::Index j = 0; j < p.size(); ++j) {
                            acc.configurations.push_back(p[j]);
                            acc.velocities.push_back(v[j]);
                        }
                    });
                });
        }
    }

    // Execute the planner
    auto planner_result = planner.execute([](const auto& p, auto trajex, auto legacy) -> std::optional<service_result> {
        if (trajex.receiver) {
            return std::move(trajex.receiver);
        }
        if (legacy.receiver) {
            return std::move(legacy.receiver);
        }

        // Legacy errors are authoritative (stable reference algorithm)
        if (legacy.error) {
            std::rethrow_exception(legacy.error);
        }
        if (trajex.error) {
            std::rethrow_exception(trajex.error);
        }

        if (p.processed_waypoint_count() < 2) {
            return std::nullopt;
        }

        throw std::logic_error("trajectory generation returned neither results nor an error");
    });

    if (!planner_result) {
        // No waypoints to process -- return empty output
        auto result_holder = std::make_shared<infer_result>();
        auto* views = &result_holder->views;
        return {std::move(result_holder), views};
    }

    // Pack output using aliasing shared_ptr for zero-copy output
    auto result_holder = std::make_shared<infer_result>();
    result_holder->data = std::move(*planner_result);

    const auto n_samples = result_holder->data.times.size();
    const auto output_dof = result_holder->data.dof;

    result_holder->views.emplace("sample_times_sec", make_tensor_view(result_holder->data.times.data(), n_samples, {n_samples}));

    result_holder->views.emplace(
        "configurations_rads",
        make_tensor_view(result_holder->data.configurations.data(), n_samples * output_dof, {n_samples, output_dof}));

    result_holder->views.emplace("velocities_rads_per_sec",
                                 make_tensor_view(result_holder->data.velocities.data(), n_samples * output_dof, {n_samples, output_dof}));

    if (result_holder->data.accelerations) {
        result_holder->views.emplace(
            "accelerations_rads_per_sec2",
            make_tensor_view(result_holder->data.accelerations->data(), n_samples * output_dof, {n_samples, output_dof}));
    }

    auto* views = &result_holder->views;
    return {std::move(result_holder), views};
}

struct trajex_mlmodel_service::metadata trajex_mlmodel_service::metadata(const vsdk::ProtoStruct&) {
    const std::shared_lock lock{config_mutex_};
    return {
        .name = "trajex",
        .type = "other",
        .description = "Time-optimal trajectory generation via TOTG",
        .inputs =
            {
                {.name = "waypoints_rads",
                 .description = "Joint configurations (in radians) [n_waypoints, n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1, -1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "waypoint_deduplication_tolerance_rads",
                 .description = "Waypoint deduplication tolerance (in radians) [scalar]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "path_tolerance_delta_rads",
                 .description = "Path tolerance delta (in radians) [scalar]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "path_colinearization_ratio",
                 .description = "Path colinearization ratio [scalar]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "velocity_limits_rads_per_sec",
                 .description = "Max joint velocities (in radians per second) [n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "acceleration_limits_rads_per_sec2",
                 .description = "Max joint accelerations (in radians per second squared) [n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "trajectory_sampling_freq_hz",
                 .description = "Trajectory sampling frequency in Hz [scalar]",
                 .data_type = tensor_info::data_types::k_int64,
                 .shape = {1},
                 .associated_files = {},
                 .extra = {}},
            },
        .outputs =
            {
                {.name = "sample_times_sec",
                 .description = "Time values for each sample (in seconds) [n_samples]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "configurations_rads",
                 .description = "Joint configurations over time (in radians) [n_samples, n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1, -1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "velocities_rads_per_sec",
                 .description = "Joint velocities over time (in radians per second) [n_samples, n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1, -1},
                 .associated_files = {},
                 .extra = {}},
                {.name = "accelerations_rads_per_sec2",
                 .description = "Joint accelerations over time (in radians per second squared) [n_samples, n_dof]",
                 .data_type = tensor_info::data_types::k_float64,
                 .shape = {-1, -1},
                 .associated_files = {},
                 .extra = {}},
            },
    };
}

}  // namespace viam::trajex

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/mlmodel.hpp>

// TODO: Include trajectory_planner.hpp once the service links Eigen and
// viam::trajex::totg-legacy (required by the planner's legacy algorithm path).
// #include <viam/trajex/service/trajectory_planner.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>

namespace {

namespace vsdk = ::viam::sdk;

class trajex_mlmodel_service : public vsdk::MLModelService, public vsdk::Reconfigurable {
   public:
    trajex_mlmodel_service(vsdk::Dependencies deps, vsdk::ResourceConfig config) : MLModelService(config.name()) {
        reconfigure(deps, config);
    }

    void reconfigure(const vsdk::Dependencies&, const vsdk::ResourceConfig&) override {
        // TODO: Extract configuration (e.g., default sample rate)
    }

    std::shared_ptr<named_tensor_views> infer(const named_tensor_views& inputs, const vsdk::ProtoStruct&) override {
        // TODO: Parse input tensors from `inputs`:
        //   - waypoints_rads [n_waypoints, n_dof] -> xt::xarray<double>
        //   - velocity_limits_rads_per_sec [n_dof] -> xt::xarray<double>
        //   - acceleration_limits_rads_per_sec2 [n_dof] -> xt::xarray<double>
        //   - path_tolerance_delta_rads [1] -> double
        //   - path_colinearization_ratio [1] -> optional<double>
        //   - waypoint_deduplication_tolerance_rads [1] -> double
        //   - trajectory_sampling_freq_hz [1] -> double
        (void)inputs;
        throw std::runtime_error("trajex_mlmodel_service::infer not yet implemented: "
                                 "input tensor parsing and output packing are TODO");

        // Planner framework (compiles once input parsing is wired up):
        //
        // struct service_result {
        //     std::vector<viam::trajex::totg::trajectory::sample> samples;
        //     double total_duration = 0.0;
        // };
        //
        // auto result = viam::trajex::trajectory_planner<service_result>({
        //     .velocity_limits = velocity_limits,
        //     .acceleration_limits = acceleration_limits,
        //     .path_blend_tolerance = path_tolerance,
        //     .colinearization_ratio = std::nullopt,
        //     .max_trajectory_duration = 300.0,
        // })
        // .with_waypoint_provider([&](auto& planner) {
        //     auto wp = planner.stash(std::move(waypoints));
        //     return viam::trajex::totg::waypoint_accumulator{*wp};
        // })
        // .with_waypoint_preprocessor([&](auto&, auto& wa) {
        //     wa = viam::trajex::totg::deduplicate_waypoints(wa, dedup_tolerance);
        // })
        // .with_trajex(
        //     [&](service_result& acc, const viam::trajex::totg::trajectory& traj, auto elapsed) {
        //         acc.total_duration += traj.duration().count();
        //         // TODO: sample trajectory, push into acc.samples
        //     }
        // )
        // // TODO: .with_legacy(...) once service links viam::trajex::totg-legacy
        // .execute([](const auto& planner, auto trajex, auto legacy) -> std::optional<service_result> {
        //     if (planner.processed_waypoint_count() < 2) return std::nullopt;
        //     if (trajex.receiver) return std::move(trajex.receiver);
        //     if (legacy.receiver) return std::move(legacy.receiver);
        //     if (trajex.error) std::rethrow_exception(trajex.error);
        //     if (legacy.error) std::rethrow_exception(legacy.error);
        //     throw std::runtime_error("trajectory generation failed");
        // });
        //
        // TODO: Pack result into output named_tensor_views and return
    }

    struct metadata metadata(const vsdk::ProtoStruct&) override {
        // Describe the expected tensor shapes for this service.
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
};

int serve(const std::string& socket_path) try {
    vsdk::Instance inst;

    auto registration =
        std::make_shared<vsdk::ModelRegistration>(vsdk::API::get<vsdk::MLModelService>(),
                                                  vsdk::Model{"viam", "mlmodelservice", "trajex"},
                                                  [](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
                                                      return std::make_shared<trajex_mlmodel_service>(std::move(deps), std::move(config));
                                                  });

    vsdk::Registry::get().register_model(registration);

    auto module_service = std::make_shared<vsdk::ModuleService>(socket_path);
    module_service->add_model_from_registry(registration->api(), registration->model());
    module_service->serve();

    return EXIT_SUCCESS;
} catch (const std::exception& ex) {
    std::cerr << "ERROR: " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "ERROR: Unknown exception" << std::endl;
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "usage: trajex-service /path/to/unix/socket" << std::endl;
        return EXIT_FAILURE;
    }
    return serve(argv[1]);
}

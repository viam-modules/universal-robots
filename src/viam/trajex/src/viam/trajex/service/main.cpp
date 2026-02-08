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

#include <viam/trajex/totg/trajectory.hpp>

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

    std::shared_ptr<named_tensor_views> infer(const named_tensor_views&, const vsdk::ProtoStruct&) override {
        // TODO: Parse input tensors (waypoints, velocity_limits, acceleration_limits),
        // run TOTG, pack output tensors (positions, velocities, accelerations, timestamps).
        throw std::runtime_error("trajex_mlmodel_service::infer not yet implemented");
    }

    struct metadata metadata(const vsdk::ProtoStruct&) override {
        // Describe the expected tensor shapes for this service.
        return {
            .name = "trajex",
            .type = "trajex_totg",
            .description = "Time-optimal trajectory generation via TOTG",
            .inputs =
                {
                    {.name = "waypoints",
                     .description = "Joint configurations [n_waypoints, n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1, -1}},
                    {.name = "velocity_limits",
                     .description = "Max joint velocities [n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1}},
                    {.name = "acceleration_limits",
                     .description = "Max joint accelerations [n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1}},
                },
            .outputs =
                {
                    {.name = "positions",
                     .description = "Joint positions over time [n_samples, n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1, -1}},
                    {.name = "velocities",
                     .description = "Joint velocities over time [n_samples, n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1, -1}},
                    {.name = "accelerations",
                     .description = "Joint accelerations over time [n_samples, n_dof]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1, -1}},
                    {.name = "timestamps",
                     .description = "Time values for each sample [n_samples]",
                     .data_type = tensor_info::data_types::k_float64,
                     .shape = {-1}},
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

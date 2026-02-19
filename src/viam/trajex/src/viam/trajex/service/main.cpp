#include <iostream>
#include <memory>
#include <string>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/services/mlmodel.hpp>

#include <viam/trajex/service/trajex_mlmodel_service.hpp>

namespace {

namespace vsdk = ::viam::sdk;

int serve(const std::string& socket_path) try {
    const vsdk::Instance inst;

    auto registration = std::make_shared<vsdk::ModelRegistration>(
        vsdk::API::get<vsdk::MLModelService>(),
        vsdk::Model{"viam", "trajex", "mlmodel"},
        [](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
            return std::make_shared<viam::trajex::trajex_mlmodel_service>(std::move(deps), std::move(config));
        },
        &viam::trajex::trajex_mlmodel_service::validate);

    vsdk::Registry::get().register_model(registration);

    auto module_service = std::make_shared<vsdk::ModuleService>(socket_path);
    module_service->add_model_from_registry(registration->api(), registration->model());
    module_service->serve();

    return EXIT_SUCCESS;
} catch (const std::exception& ex) {
    std::cerr << "ERROR: " << ex.what() << '\n';
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "ERROR: Unknown exception" << '\n';
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "usage: trajex-service /path/to/unix/socket" << '\n';
        return EXIT_FAILURE;
    }
    return serve(argv[1]);
}

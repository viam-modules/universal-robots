#pragma once

#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/mlmodel.hpp>

namespace viam::trajex {

namespace vsdk = ::viam::sdk;

class trajex_mlmodel_service : public vsdk::MLModelService, public vsdk::Reconfigurable {
   public:
    trajex_mlmodel_service(vsdk::Dependencies deps, vsdk::ResourceConfig config);

    void reconfigure(const vsdk::Dependencies&, const vsdk::ResourceConfig&) override;

    std::shared_ptr<named_tensor_views> infer(const named_tensor_views& inputs, const vsdk::ProtoStruct& extra) override;

    struct metadata metadata(const vsdk::ProtoStruct& extra) override;

   private:
    struct config {
        std::vector<std::string> generator_sequence = {"totg", "legacy"};
        bool segment_for_totg = true;
    };

    mutable std::shared_mutex config_mutex_;
    config config_;
};

}  // namespace viam::trajex

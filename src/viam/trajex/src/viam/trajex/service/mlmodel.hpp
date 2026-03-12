#pragma once

#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/mlmodel.hpp>

namespace viam::trajex::service {

class mlmodel final : public ::viam::sdk::MLModelService, public ::viam::sdk::Reconfigurable {
   public:
    mlmodel(::viam::sdk::Dependencies deps, ::viam::sdk::ResourceConfig config);

    void reconfigure(const ::viam::sdk::Dependencies&, const ::viam::sdk::ResourceConfig&) override;

    std::shared_ptr<named_tensor_views> infer(const named_tensor_views& inputs, const ::viam::sdk::ProtoStruct& extra) override;

    struct metadata metadata(const ::viam::sdk::ProtoStruct& extra) override;

    static std::vector<std::string> validate(const ::viam::sdk::ResourceConfig& cfg);

   private:
    struct config {
        std::vector<std::string> generator_sequence = {"totg", "legacy"};
        bool segment_for_trajex = false;
    };

    mutable std::shared_mutex config_mutex_;
    config config_;
};

}  // namespace viam::trajex::service

#define BOOST_TEST_MODULE trajex_service_test

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <boost/test/included/unit_test.hpp>
#pragma GCC diagnostic pop

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/services/mlmodel.hpp>

#include <viam/trajex/service/trajex_mlmodel_service.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

namespace {

namespace vsdk = ::viam::sdk;
using viam::trajex::trajex_mlmodel_service;

vsdk::ResourceConfig make_config(const vsdk::ProtoStruct& attrs = {}) {
    return vsdk::ResourceConfig{
        "mlmodel",      // type (must match api subtype)
        "trajex_test",  // name
        "rdk",          // namespace (must match api namespace)
        attrs,
        "rdk:service:mlmodel",
        vsdk::Model{"viam", "mlmodelservice", "trajex"},
    };
}

// Build a named_tensor_views map with a simple two-point 3-DOF trajectory input
trajex_mlmodel_service::named_tensor_views make_simple_inputs() {
    // We need the data to outlive the views, so use static storage for test convenience
    static const std::vector<double> waypoints_data = {
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
    };
    static const std::vector<double> vel_limits = {1.0, 1.0, 1.0};
    static const std::vector<double> acc_limits = {1.0, 1.0, 1.0};
    static const std::vector<double> path_tolerance = {0.001};
    static const std::vector<double> colinearization_ratio = {0.0};  // disabled
    static const std::vector<double> dedup_tolerance = {1e-6};
    static const std::vector<std::int64_t> sampling_freq = {100};

    trajex_mlmodel_service::named_tensor_views inputs;

    inputs.emplace("waypoints_rads", trajex_mlmodel_service::make_tensor_view(waypoints_data.data(), 6, {2, 3}));
    inputs.emplace("velocity_limits_rads_per_sec", trajex_mlmodel_service::make_tensor_view(vel_limits.data(), 3, {3}));
    inputs.emplace("acceleration_limits_rads_per_sec2", trajex_mlmodel_service::make_tensor_view(acc_limits.data(), 3, {3}));
    inputs.emplace("path_tolerance_delta_rads", trajex_mlmodel_service::make_tensor_view(path_tolerance.data(), 1, {1}));
    inputs.emplace("path_colinearization_ratio", trajex_mlmodel_service::make_tensor_view(colinearization_ratio.data(), 1, {1}));
    inputs.emplace("waypoint_deduplication_tolerance_rads", trajex_mlmodel_service::make_tensor_view(dedup_tolerance.data(), 1, {1}));
    inputs.emplace("trajectory_sampling_freq_hz", trajex_mlmodel_service::make_tensor_view(sampling_freq.data(), 1, {1}));

    return inputs;
}

}  // namespace

BOOST_AUTO_TEST_SUITE(trajex_service_config_tests)

BOOST_AUTO_TEST_CASE(default_config) {
    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config());
    // Should not throw -- default config is ["totg", "legacy"]
    auto result = service->infer(make_simple_inputs(), {});
    BOOST_CHECK(result);
}

BOOST_AUTO_TEST_CASE(totg_only_config) {
    vsdk::ProtoStruct attrs;
    attrs.emplace("generator_sequence", std::vector<vsdk::ProtoValue>{vsdk::ProtoValue{"totg"}});

    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config(attrs));
    auto result = service->infer(make_simple_inputs(), {});
    BOOST_CHECK(result);

    // Should have accelerations (trajex produces them)
    BOOST_CHECK(result->count("accelerations_rads_per_sec2") > 0);
}

BOOST_AUTO_TEST_CASE(legacy_only_config) {
    vsdk::ProtoStruct attrs;
    attrs.emplace("generator_sequence", std::vector<vsdk::ProtoValue>{vsdk::ProtoValue{"legacy"}});

    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config(attrs));
    auto result = service->infer(make_simple_inputs(), {});
    BOOST_CHECK(result);

    // Should NOT have accelerations (legacy doesn't produce them)
    BOOST_CHECK(result->count("accelerations_rads_per_sec2") == 0);
}

BOOST_AUTO_TEST_CASE(unknown_algorithm_rejects) {
    vsdk::ProtoStruct attrs;
    attrs.emplace("generator_sequence", std::vector<vsdk::ProtoValue>{vsdk::ProtoValue{"unknown"}});

    BOOST_CHECK_THROW(std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config(attrs)), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(empty_sequence_rejects) {
    vsdk::ProtoStruct attrs;
    attrs.emplace("generator_sequence", std::vector<vsdk::ProtoValue>{});

    BOOST_CHECK_THROW(std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config(attrs)), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajex_service_input_validation_tests)

BOOST_AUTO_TEST_CASE(missing_tensor_throws) {
    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config());
    trajex_mlmodel_service::named_tensor_views empty_inputs;
    BOOST_CHECK_THROW(service->infer(empty_inputs, {}), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(dof_mismatch_throws) {
    static const std::vector<double> waypoints_data = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    static const std::vector<double> vel_limits = {1.0, 1.0};  // 2-DOF, waypoints are 3-DOF
    static const std::vector<double> acc_limits = {1.0, 1.0};
    static const std::vector<double> path_tolerance = {0.001};
    static const std::vector<double> colinearization_ratio = {0.0};
    static const std::vector<double> dedup_tolerance = {1e-6};
    static const std::vector<std::int64_t> sampling_freq = {100};

    trajex_mlmodel_service::named_tensor_views inputs;
    inputs.emplace("waypoints_rads", trajex_mlmodel_service::make_tensor_view(waypoints_data.data(), 6, {2, 3}));
    inputs.emplace("velocity_limits_rads_per_sec", trajex_mlmodel_service::make_tensor_view(vel_limits.data(), 2, {2}));
    inputs.emplace("acceleration_limits_rads_per_sec2", trajex_mlmodel_service::make_tensor_view(acc_limits.data(), 2, {2}));
    inputs.emplace("path_tolerance_delta_rads", trajex_mlmodel_service::make_tensor_view(path_tolerance.data(), 1, {1}));
    inputs.emplace("path_colinearization_ratio", trajex_mlmodel_service::make_tensor_view(colinearization_ratio.data(), 1, {1}));
    inputs.emplace("waypoint_deduplication_tolerance_rads", trajex_mlmodel_service::make_tensor_view(dedup_tolerance.data(), 1, {1}));
    inputs.emplace("trajectory_sampling_freq_hz", trajex_mlmodel_service::make_tensor_view(sampling_freq.data(), 1, {1}));

    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config());
    BOOST_CHECK_THROW(service->infer(inputs, {}), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(trajex_service_inference_tests)

BOOST_AUTO_TEST_CASE(simple_trajectory_produces_valid_output) {
    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config());
    auto result = service->infer(make_simple_inputs(), {});

    BOOST_REQUIRE(result);
    BOOST_CHECK(result->count("sample_times_sec") > 0);
    BOOST_CHECK(result->count("configurations_rads") > 0);
    BOOST_CHECK(result->count("velocities_rads_per_sec") > 0);

    // Verify output types are float64
    BOOST_CHECK_EQUAL(vsdk::MLModelService::tensor_info::tensor_views_to_data_type(result->at("sample_times_sec")),
                      vsdk::MLModelService::tensor_info::data_types::k_float64);
    BOOST_CHECK_EQUAL(vsdk::MLModelService::tensor_info::tensor_views_to_data_type(result->at("configurations_rads")),
                      vsdk::MLModelService::tensor_info::data_types::k_float64);
}

BOOST_AUTO_TEST_CASE(single_waypoint_returns_empty) {
    static const std::vector<double> single_wp = {0.0, 0.0, 0.0};
    static const std::vector<double> vel_limits = {1.0, 1.0, 1.0};
    static const std::vector<double> acc_limits = {1.0, 1.0, 1.0};
    static const std::vector<double> path_tolerance = {0.001};
    static const std::vector<double> colinearization_ratio = {0.0};
    static const std::vector<double> dedup_tolerance = {1e-6};
    static const std::vector<std::int64_t> sampling_freq = {100};

    trajex_mlmodel_service::named_tensor_views inputs;
    inputs.emplace("waypoints_rads", trajex_mlmodel_service::make_tensor_view(single_wp.data(), 3, {1, 3}));
    inputs.emplace("velocity_limits_rads_per_sec", trajex_mlmodel_service::make_tensor_view(vel_limits.data(), 3, {3}));
    inputs.emplace("acceleration_limits_rads_per_sec2", trajex_mlmodel_service::make_tensor_view(acc_limits.data(), 3, {3}));
    inputs.emplace("path_tolerance_delta_rads", trajex_mlmodel_service::make_tensor_view(path_tolerance.data(), 1, {1}));
    inputs.emplace("path_colinearization_ratio", trajex_mlmodel_service::make_tensor_view(colinearization_ratio.data(), 1, {1}));
    inputs.emplace("waypoint_deduplication_tolerance_rads", trajex_mlmodel_service::make_tensor_view(dedup_tolerance.data(), 1, {1}));
    inputs.emplace("trajectory_sampling_freq_hz", trajex_mlmodel_service::make_tensor_view(sampling_freq.data(), 1, {1}));

    auto service = std::make_shared<trajex_mlmodel_service>(vsdk::Dependencies{}, make_config());
    auto result = service->infer(inputs, {});

    // Single waypoint: nothing to do, should return empty views
    BOOST_REQUIRE(result);
    BOOST_CHECK(result->empty());
}

BOOST_AUTO_TEST_SUITE_END()

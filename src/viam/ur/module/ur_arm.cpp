#include "ur_arm.hpp"

#include "trajectory_logger.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

#include <json/json.h>

#include <grpcpp/server_context.h>

#include <boost/format.hpp>
#include <boost/io/ostream_joiner.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <viam/sdk/components/component.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>

#include <viam/sdk/rpc/grpc_context_observer.hpp>
#include <viam/trajex/totg/tools/planner.hpp>
#include <viam/trajex/totg/totg.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>
#include <viam/trajex/types/angles.hpp>
#include <viam/trajex/types/hertz.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xadapt.hpp>
#include <xtensor/containers/xarray.hpp>
#include <xtensor/core/xmath.hpp>
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xadapt.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#endif

#include "rust_utils.hpp"
#include "ur_arm_model.hpp"
#include "ur_arm_state.hpp"
#include "utils.hpp"

namespace {

constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

constexpr double k_min_duration_secs = 0.1;
constexpr double k_max_duration_secs = 3600.0;
constexpr double k_min_sampling_freq_hz = 1.0;
constexpr double k_max_sampling_freq_hz = 500.0;

pose ur_vector_to_pose(urcl::vector6d_t vec) {
    const double norm = std::hypot(vec[3], vec[4], vec[5]);
    if (std::isnan(norm) || (norm == 0)) {
        throw std::invalid_argument("Cannot normalize with NaN or zero norm");
    }

    const auto q = rust_utils::quaternion_from_axis_angle(vec[3] / norm, vec[4] / norm, vec[5] / norm, norm);
    const auto ov = rust_utils::ov_from_quaternion(q.get());
    const auto components = rust_utils::ov_components(ov.get());

    auto position = coordinates{1000 * vec[0], 1000 * vec[1], 1000 * vec[2]};
    auto orientation = pose_orientation{components[0], components[1], components[2]};
    auto theta = radians_to_degrees(components[3]);

    return {position, orientation, theta};
}

urcl::vector6d_t pose_to_ur_vector(const pose& p) {
    if (!std::isfinite(p.theta)) {
        throw std::invalid_argument("pose_to_ur_vector: theta is infinite or NaN");
    }

    urcl::vector6d_t v{};

    // convert millimeters to meters
    v[0] = p.coordinates.x / 1000.0;
    v[1] = p.coordinates.y / 1000.0;
    v[2] = p.coordinates.z / 1000.0;

    const double theta_rad = degrees_to_radians(p.theta);

    // If angle ~ 0, UR expects the axis-angle "vector" to be zero.
    constexpr double k_angle_epsilon = 1e-12;
    if (std::abs(theta_rad) < k_angle_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
        return v;
    }

    // convert viam's orientation to axis angles which is what universal robots use
    // viam orientation vector -> quaternion -> axis angle -> universal robots orientation vector
    const auto ov = rust_utils::make_orientation_vector(p.orientation.o_x, p.orientation.o_y, p.orientation.o_z, theta_rad);
    if (!ov) {
        throw std::runtime_error("pose_to_ur_vector: failed to create orientation vector");
    }

    const auto q = rust_utils::quaternion_from_ov(ov.get());
    if (!q) {
        throw std::runtime_error("pose_to_ur_vector: failed to create quaternion");
    }

    const auto aa = rust_utils::axis_angle_from_quaternion(q.get());
    if (!aa) {
        throw std::runtime_error("pose_to_ur_vector: failed to compute axis-angle");
    }

    const double ax = aa[0];
    const double ay = aa[1];
    const double az = aa[2];
    const double th = aa[3];

    // robustness to tiny axes/angles
    const double n = std::hypot(ax, ay, az);
    constexpr double k_axis_epsilon = 1e-8;
    if (n < k_angle_epsilon || std::abs(th) < k_axis_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
    } else {
        // universal robots orientation vector: r = axis * theta
        v[3] = ax * th;
        v[4] = ay * th;
        v[5] = az * th;
    }

    return v;
}

std::vector<std::string> validate_config_(const ResourceConfig& cfg) {
    if (!find_config_attribute<std::string>(cfg, "host")) {
        throw std::invalid_argument("attribute `host` is required");
    }

    parse_and_validate_joint_limits(cfg, "speed_degs_per_sec");
    parse_and_validate_joint_limits(cfg, "acceleration_degs_per_sec2");

    if (cfg.attributes().contains("max_speed_degs_per_sec")) {
        parse_and_validate_joint_limits(cfg, "max_speed_degs_per_sec");
    }

    if (cfg.attributes().contains("max_acceleration_degs_per_sec2")) {
        parse_and_validate_joint_limits(cfg, "max_acceleration_degs_per_sec2");
    }

    auto max_duration = find_config_attribute<double>(cfg, "max_trajectory_duration_secs");
    if (max_duration && (*max_duration < k_min_duration_secs || *max_duration > k_max_duration_secs)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `max_trajectory_duration_secs` should be between %1% and %2%, it is: %3% seconds") %
                       k_min_duration_secs % k_max_duration_secs % *max_duration));
    }

    auto sampling_freq = find_config_attribute<double>(cfg, "trajectory_sampling_freq_hz");
    if (sampling_freq && (*sampling_freq < k_min_sampling_freq_hz || *sampling_freq > k_max_sampling_freq_hz)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `trajectory_sampling_freq_hz` should be between %1% and %2%, it is: %3% Hz") %
                       k_min_sampling_freq_hz % k_max_sampling_freq_hz % *sampling_freq));
    }

    auto threshold = find_config_attribute<double>(cfg, "reject_move_request_threshold_deg");
    constexpr double k_min_threshold = 0.0;
    constexpr double k_max_threshold = 360.0;
    if (threshold && (*threshold < k_min_threshold || *threshold > k_max_threshold)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `reject_move_request_threshold_deg` should be between %1% and %2%, it is: %3% degrees") %
                       k_min_threshold % k_max_threshold % *threshold));
    }

    auto frequency = find_config_attribute<double>(cfg, "robot_control_freq_hz");
    constexpr double k_max_frequency = 1000.;
    if (frequency && (*frequency <= 0. || *frequency >= k_max_frequency)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `robot_control_freq_hz` should be a positive number less than %1%, it is: %2% Hz") %
                       k_max_frequency % *frequency));
    }

    auto path_tolerance_deg = find_config_attribute<double>(cfg, "path_tolerance_delta_deg");
    if (path_tolerance_deg && (*path_tolerance_deg <= 0 || *path_tolerance_deg > 12)) {
        throw std::invalid_argument(boost::str(
            boost::format("attribute `path_tolerance_delta_deg` must be > 0 and <= 12, it is: %1% degrees") % *path_tolerance_deg));
    }

    auto colinearization_ratio = find_config_attribute<double>(cfg, "path_colinearization_ratio");
    if (colinearization_ratio && (*colinearization_ratio < 0 || *colinearization_ratio > 2)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `path_colinearization_ratio` must be >= 0 and <= 2, it is: %1%") % *colinearization_ratio));
    }

    auto segmentation_threshold = find_config_attribute<double>(cfg, "segmentation_threshold");
    if (segmentation_threshold && (*segmentation_threshold <= 0 || *segmentation_threshold > 0.01)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `segmentation_threshold` must be > 0 and <= 0.01, it is: %1%") % *segmentation_threshold));
    }

    find_config_attribute<bool>(cfg, "enable_new_trajectory_planner");
    find_config_attribute<bool>(cfg, "prefer_precomputed_accelerations");
    find_config_attribute<bool>(cfg, "segment_for_trajex");

    // Validate telemetry_output_path is a string if present
    const auto telemetry_output_path = find_config_attribute<std::string>(cfg, "telemetry_output_path");

    // Also validate that `csv_output_path`, if present, is a string.
    //
    // TODO(RSDK-12929): When `csv_output_path` is removed, actively reject it (don't ignore it).
    const auto csv_output_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    if (csv_output_path) {
        if (telemetry_output_path) {
            throw std::invalid_argument("Only one of `csv_output_path` (deprecated) or `telemetry_output_path` may be specified");
        }
    }

    // Validate telemetry_output_path_append_traceid: accepts bool (backward compat) or template string containing {trace_id}
    {
        const auto it = cfg.attributes().find("telemetry_output_path_append_traceid");
        if (it != cfg.attributes().end()) {
            if (const auto* s = it->second.get<std::string>()) {
                if (s->find("{trace_id}") == std::string::npos) {
                    throw std::invalid_argument("`telemetry_output_path_append_traceid` template string must contain `{trace_id}`");
                }
            } else if (!it->second.get<bool>()) {
                throw std::invalid_argument(
                    "`telemetry_output_path_append_traceid` must be a boolean or a template string containing `{trace_id}`");
            }
        }
    }

    return {};
}

template <typename Callable>
auto make_scope_guard(Callable&& cleanup) {
    struct guard {
       public:
        explicit guard(Callable&& cleanup) : cleanup_(std::move(cleanup)) {}
        void deactivate() {
            cleanup_ = [] {};
        }
        ~guard() {
            cleanup_();
        }

       private:
        std::function<void()> cleanup_;
    };
    return guard{std::forward<Callable>(cleanup)};
}

// Converts a stream of SDK `trajectory_point` batches into URCL spline-point
// batches. The PV-vs-PVA choice is fixed from the first point and held for the
// life of the stream; per-point timesteps are differenced out of the
// stream-global cumulative time as points are converted.
class trajectory_point_converter {
   public:
    trajectory_point_converter(const URArm::trajectory_point& first, bool prefer_pva)
        : use_pva_{first.constraints && first.constraints->accelerations.has_value() && prefer_pva} {}

    trajectory_samples convert(const std::vector<URArm::trajectory_point>& batch) {
        auto samples = use_pva_ ? trajectory_samples{std::vector<trajectory_sample_point_pva>{}}
                                : trajectory_samples{std::vector<trajectory_sample_point_pv>{}};
        std::visit(
            [&](auto& points) {
                using point_type = typename std::decay_t<decltype(points)>::value_type;
                points.reserve(batch.size());
                for (const auto& p : batch) {
                    points.push_back(to_spline_point_<point_type>(p));
                }
            },
            samples);
        return samples;
    }

   private:
    // Copies `src` into `dst`, converting each joint value from degrees to radians.
    // `dst` fixes the expected joint count; a size mismatch is a client input error.
    static void to_radians_into_(urcl::vector6d_t& dst, const std::vector<double>& src, const char* field) {
        if (src.size() != dst.size()) {
            throw std::invalid_argument(std::string("trajectory point ") + field + " has the wrong number of joints");
        }
        std::ranges::transform(src, dst.begin(), [](double deg) { return degrees_to_radians(deg); });
    }

    // The `trajectory_point` contract carries joint positions, velocities, and
    // accelerations in degrees (matching the unary `move_through_joint_positions`
    // API); URCL wants radians. The SDK stub already validated time ordering and
    // per-point arity, so here we only guard DOF and the presence of the
    // constraints this arm requires.
    template <typename Point>
    Point to_spline_point_(const URArm::trajectory_point& p) {
        if (!p.constraints) {
            throw std::invalid_argument("trajectory point missing constraints (velocities required)");
        }
        Point pt;
        pt.timestep = std::chrono::duration<float>(p.time - cumulative_time_).count();
        cumulative_time_ = p.time;
        to_radians_into_(pt.p, p.positions, "positions");
        to_radians_into_(pt.v, p.constraints->velocities, "velocities");
        if constexpr (requires { pt.a; }) {
            if (!p.constraints->accelerations) {
                throw std::invalid_argument("PVA stream point missing accelerations");
            }
            to_radians_into_(pt.a, *p.constraints->accelerations, "accelerations");
        }
        return pt;
    }

    bool use_pva_;
    std::chrono::microseconds cumulative_time_{0};
};

}  // namespace

std::string failed_trajectory_filename(const std::string& path, const std::string& resource_name, const boost::uuids::uuid& move_id) {
    constexpr char kFailedTrajectoryJsonNameTemplate[] = "/%1%_%2%_failed_trajectory.json";
    auto fmt = boost::format(path + kFailedTrajectoryJsonNameTemplate);
    return (fmt % boost::uuids::to_string(move_id) % resource_name).str();
}

const ModelFamily& URArm::model_family() {
    // TODO: If ModelFamily had a constexpr constructor, we wouldn't need
    // this function at all and could just inline it into the class definition.
    static const auto family = ModelFamily{"viam", "universal-robots"};
    return family;
}

Model URArm::model(std::string model_name) {
    return {model_family(), std::move(model_name)};
}

std::vector<std::shared_ptr<ModelRegistration>> URArm::create_model_registrations() {
    const auto arm = API::get<Arm>();
    const auto registration_factory = [&](const UrModelDescriptor& d) {
        const auto model = URArm::model(d.sdk_name);
        return std::make_shared<ModelRegistration>(
            arm,
            model,
            // NOLINTNEXTLINE(performance-unnecessary-value-param): Signature is fixed by ModelRegistration.
            [model](auto deps, auto config) { return std::make_unique<URArm>(model, deps, config); },
            [](auto const& config) { return validate_config_(config); });
    };

    auto registrations = UrModelDescriptor::all() | boost::adaptors::transformed(registration_factory);
    return {std::make_move_iterator(begin(registrations)), std::make_move_iterator(end(registrations))};
}

URArm::URArm(Model model, const Dependencies& deps, const ResourceConfig& cfg) : Arm(cfg.name()), arm_model_(std::move(model)) {
    VIAM_SDK_LOG(info) << "instantiating URArm driver for arm model: " << arm_model_.sdk_model().to_string();
    const std::unique_lock wlock(config_mutex_);
    // TODO: prevent multiple calls to configure_logger
    configure_logger(cfg);
    configure_(wlock, deps, cfg);
}

void URArm::configure_(const std::unique_lock<std::shared_mutex>& lock, const Dependencies&, const ResourceConfig& cfg) {
    if (current_state_) {
        throw std::logic_error("URArm::configure_ was called for a currently configured instance");
    }

    // If we fail to make it through the startup sequence, execute the shutdown code. The
    // shutdown code must be prepared to be called from any intermediate state that this
    // function may have constructed due to partial execution.
    auto failure_handler = make_scope_guard([&] {
        VIAM_SDK_LOG(warn) << "URArm startup failed - shutting down";
        shutdown_(lock);
    });

    VIAM_SDK_LOG(debug) << "URArm starting up";
    current_state_ = state_::create(arm_model_, name(), cfg, ports_);

    // state_::create blocks until the state machine reaches a connected state
    // (its constructor loops on upgrade_downgrade_ until current_state_ is no
    // longer state_disconnected_), so by the time we reach this point the
    // primary client is up and we can read from it.
    VIAM_SDK_LOG(info) << "Fetching calibrated DH parameters from controller";
    // Force the eager calibration fetch at startup so any failure to obtain
    // KinematicsInfo from the controller surfaces here rather than on the
    // first `get_kinematics()` call. We deliberately do not pre-build the
    // JSON: the first `get_kinematics()` caller will trigger that work
    // under `std::call_once` in the future payload.
    const auto kin_info = current_state_->get_calibrated_kinematics_info(std::chrono::seconds{5});

    std::stringstream s;
    s << "Calibrated KinematicsInfo received: a=[";
    boost::copy(kin_info.dh_a_, boost::io::make_ostream_joiner(s, ", "));
    s << "], d=[";
    boost::copy(kin_info.dh_d_, boost::io::make_ostream_joiner(s, ", "));
    s << "], alpha=[";
    boost::copy(kin_info.dh_alpha_, boost::io::make_ostream_joiner(s, ", "));
    s << "], theta=[";
    boost::copy(kin_info.dh_theta_, boost::io::make_ostream_joiner(s, ", "));
    s << "]";
    VIAM_SDK_LOG(info) << s.str();

    VIAM_SDK_LOG(info) << "URArm startup complete";
    failure_handler.deactivate();
}

template <template <typename> typename lock_type>
void URArm::check_configured_(const lock_type<std::shared_mutex>&) {
    if (!current_state_) {
        std::ostringstream buffer;
        buffer << "Arm is not currently configured; reconfiguration likely failed";
        throw std::runtime_error(buffer.str());
    }
}

std::vector<double> URArm::get_joint_positions(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    auto joint_rads = get_joint_positions_rad_(rlock);
    auto joint_position_degree = joint_rads | boost::adaptors::transformed(radians_to_degrees<const double&>);
    return {std::begin(joint_position_degree), std::end(joint_position_degree)};
}

vector6d_t URArm::get_joint_positions_rad_(const std::shared_lock<std::shared_mutex>&) {
    return current_state_->read_joint_positions();
}

void URArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    const auto id = current_state_->allocate_move_id();

    const std::array<std::size_t, 2> shape = {1, positions.size()};
    const auto waypoint_deg = xt::adapt(positions.data(), positions.size(), xt::no_ownership(), shape);
    const xt::xarray<double> waypoint_rad = viam::trajex::degrees_to_radians(waypoint_deg);

    move_joint_space_(std::move(rlock), waypoint_rad, MoveOptions{}, id);
}

void URArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                         const MoveOptions& options,
                                         const viam::sdk::ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    const auto id = current_state_->allocate_move_id();

    if (positions.empty()) {
        return;
    }

    xt::xarray<double> waypoints_rad = xt::zeros<double>({positions.size(), positions[0].size()});
    for (size_t i = 0; i < positions.size(); ++i) {
        const std::array<std::size_t, 1> shape = {positions[i].size()};
        auto row_deg = xt::adapt(positions[i].data(), positions[i].size(), xt::no_ownership(), shape);
        xt::view(waypoints_rad, i, xt::all()) = viam::trajex::degrees_to_radians(row_deg);
    }

    move_joint_space_(std::move(rlock), waypoints_rad, options, id);
}

void URArm::check_streamed_start_pose_(const trajectory_point& first, const std::shared_lock<std::shared_mutex>& config_rlock) {
    // The streamed analog of the unary path's move validator. The stream's first
    // point is the trajectory's starting state at t=0; if the arm is not actually
    // there, the leading near-zero-duration segment would command a discontinuous
    // jump. The unary path is intrinsically safe because it seeds planning with
    // the measured position; streaming is not, so we require the threshold rather
    // than skip the check when it is unset.
    const auto& threshold = current_state_->get_reject_move_request_threshold_rad();
    if (!threshold) {
        throw std::invalid_argument("streamed moves require reject_move_request_threshold_deg to be configured");
    }
    const auto current = get_joint_positions_rad_(config_rlock);
    if (first.positions.size() != current.size()) {
        throw std::invalid_argument("trajectory point joint dimensionality mismatch");
    }
    const auto max_diff = std::transform_reduce(
        first.positions.begin(),
        first.positions.end(),
        current.begin(),
        0.0,
        [](auto a, auto b) { return std::max(a, b); },
        [](auto commanded_deg, auto actual_rad) { return std::abs(degrees_to_radians(commanded_deg) - actual_rad); });
    if (max_diff > *threshold) {
        std::stringstream err_string;
        err_string << "rejecting streamed move: first trajectory position [(";
        boost::copy(first.positions, boost::io::make_ostream_joiner(err_string, ", "));
        err_string << ")] and current joint position [(";
        boost::copy(boost::adaptors::transform(current, radians_to_degrees<const double&>),
                    boost::io::make_ostream_joiner(err_string, ", "));
        err_string << ")] differ by " << viam::trajex::radians_to_degrees(max_diff) << " > " << viam::trajex::radians_to_degrees(*threshold)
                   << " degrees";
        VIAM_SDK_LOG(error) << err_string.str();
        throw std::invalid_argument(err_string.str());
    }

    // TODO(RSDK-14273): passing this check admits up to `threshold` of slop between the
    // commanded first position and the arm's actual position, which the unary path
    // avoids by seeding the trajectory with the measured position. Determine
    // whether that slop needs handling (e.g. substituting the measured position for
    // the first point) once URScript's spline-start behavior is characterized.
}

URArm::stream_outcome URArm::move_through_joint_positions_streamed(
    const std::function<boost::optional<std::vector<trajectory_point>>()>& batch_source,
    const std::function<bool(trajectory_update)>& update_handler,
    const viam::sdk::ProtoStruct&) {
    // How the producer side of the stream ended: one of three clean exits, or a
    // producer-side error carried as an exception to rethrow. The unlocked teardown
    // below turns each into a return or a throw.
    enum class exit_reason { k_completed, k_halted_by_update_handler, k_worker_finished_early };
    struct loop_result {
        std::future<void> future;
        std::variant<exit_reason, std::exception_ptr> outcome;
    };

    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    const auto id = current_state_->allocate_move_id();

    VIAM_SDK_LOG(debug) << "move_streamed: start id " << id.uuid;
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(debug) << "move_streamed: end id " << id.uuid; });

    // Locked phase: claim the slot and drive the stream. The read lock is moved in
    // here so it releases when this returns, before we wait on completion. We hold
    // it across the blocking `batch_source()` waits to keep `current_state_` valid; a
    // stalled client is broken out by gRPC context cancellation / RPC teardown.
    auto result = [&, rlock = std::move(rlock)]() -> loop_result {
        // Capture the gRPC server context's cancellation status for the worker
        // visitor's top-of-tick async-cancel check. Same idiom as `move_joint_space_`.
        auto async_cancellation_monitor = [observer = GrpcContextObserver::current()]() {
            if (!observer) {
                return false;
            }
            return observer->context().IsCancelled();
        };

        // TODO(RSDK-14267): realtime telemetry is not yet implemented for streamed
        // trajectories, so streaming runs without a `RealtimeTrajectoryLogger`.
        auto future = current_state_->start_move_request(id, nullptr, std::move(async_cancellation_monitor));

        try {
            const auto reason = [&]() {
                // The converter is created from the first non-empty batch's first
                // point; it fixes the PV-vs-PVA choice and carries cumulative time.
                std::optional<trajectory_point_converter> converter;
                while (const auto batch = batch_source()) {
                    if (batch->empty()) {
                        // The dispatcher contract filters these out, but be defensive.
                        continue;
                    }

                    // A worker-side fault completes the future ahead of end-of-stream.
                    // Notice it here so we stop within a batch rather than feed a slot
                    // that is already gone; the completion path reports the outcome.
                    if (future.wait_for(std::chrono::seconds{0}) == std::future_status::ready) {
                        return exit_reason::k_worker_finished_early;
                    }

                    if (!converter) {
                        // TODO(RSDK-14274): Re-enable this check when it doesn't break move_to_joint_positions.
                        // check_streamed_start_pose_(batch->front(), rlock);
                        converter.emplace(batch->front(), current_state_->prefer_precomputed_accelerations());
                    }

                    // `extend`/`close` return false when our move is no longer the active
                    // one -- the worker finished it out from under us.
                    if (!current_state_->extend_move_request(id.uuid, converter->convert(*batch))) {
                        return exit_reason::k_worker_finished_early;
                    }

                    if (!update_handler({})) {
                        // The update handler asked us to stop. On the server side that
                        // only happens when the gRPC call is being torn down (client
                        // cancel or dead transport), which is the same as an async
                        // cancel, so we cancel the move.
                        current_state_->cancel_move_request(id.uuid);
                        return exit_reason::k_halted_by_update_handler;
                    }
                }

                // End of stream. Close; a false return means the worker raced us to
                // completion, handled the same as an early finish.
                return current_state_->close_move_request(id.uuid) ? exit_reason::k_completed : exit_reason::k_worker_finished_early;
            }();
            return {std::move(future), reason};
        } catch (...) {
            // A producer-side error (start-pose reject, malformed point). Cancel so
            // the worker doesn't wait for data that will never arrive; the unlocked
            // phase waits for the cancel to land before rethrowing.
            current_state_->cancel_move_request(id.uuid);
            return {std::move(future), std::current_exception()};
        }
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of the returned future.

    if (const auto* error = std::get_if<std::exception_ptr>(&result.outcome)) {
        // A producer-side error. Wait for our cancel to take effect so a retry
        // can't race the dying move, then rethrow the original error.
        try {
            result.future.get();
        } catch (...) {  // NOLINT(bugprone-empty-catch): Intentional
        }
        std::rethrow_exception(*error);
    }

    switch (std::get<exit_reason>(result.outcome)) {
        case exit_reason::k_completed:
            result.future.get();
            return stream_outcome::k_completed;
        case exit_reason::k_halted_by_update_handler:
            // We cancelled at the handler's request, so the future completing with a
            // cancellation error is expected, not a fault. Swallow it.
            try {
                result.future.get();
            } catch (...) {  // NOLINT(bugprone-empty-catch): Intentional
            }
            return stream_outcome::k_halted_by_update_handler;
        case exit_reason::k_worker_finished_early:
            // The worker completed before end-of-stream. In practice a fault, which
            // `get()` rethrows. A clean completion here would mean the arm finished a
            // motion we never told it was done with; surface that, don't hide it.
            result.future.get();
            throw std::runtime_error("arm reported trajectory completion before end-of-stream");
    }

    // Unreachable: the switch covers every `exit_reason`.
    throw std::logic_error("move_through_joint_positions_streamed: unhandled exit_reason");
}

pose URArm::get_end_position(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return ur_vector_to_pose(current_state_->read_tcp_pose());
}

bool URArm::is_moving() {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return current_state_->is_moving();
}

ProtoStruct URArm::get_status() {
    return {};
}

void URArm::move_to_position(const pose& p, const ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    const auto id = current_state_->allocate_move_id();
    move_tool_space_(std::move(rlock), p, id);
}

viam::sdk::KinematicsData URArm::get_kinematics(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    // Every supported arm goes through the calibrated-DH path: at configure
    // time we fetch DH from the controller, and `get_dh_kinematics_json`
    // synthesizes the SVA document on first call. There is no static-file
    // fallback.
    auto dh_json = current_state_->get_dh_kinematics_json(std::chrono::milliseconds{100});
    VIAM_SDK_LOG(info) << "get_kinematics: serving synthesized SVA JSON built from calibrated DH (" << dh_json.size() << " bytes)";
    return viam::sdk::KinematicsDataSVA(std::vector<unsigned char>(dh_json.begin(), dh_json.end()));
}

// Unknown arm models should return an empty map. Arm models that do not have all expected parts in the map should return as much as they
// have
std::map<std::string, mesh> URArm::get_3d_models(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    const auto& parts_to_load = arm_model_.descriptor().mesh_parts;

    std::map<std::string, mesh> result_model_parts;
    constexpr char threeDModelFileTemplate[] = "3d_models/%1%/%2%.glb";

    for (const auto& part : parts_to_load) {
        const std::filesystem::path model_file_path =
            current_state_->resource_root() / str(boost::format(threeDModelFileTemplate) % arm_model_.sdk_name() % part);

        // Open the file in binary mode
        std::ifstream model_file(model_file_path, std::ios::binary);
        if (!model_file) {
            throw std::runtime_error(str(boost::format("unable to open 3d model file '%1%'") % model_file_path));
        }

        // Read the entire file into a vector without computing size ahead of time
        std::vector<char> temp_bytes(std::istreambuf_iterator<char>(model_file), {});
        if (model_file.bad()) {
            throw std::runtime_error(str(boost::format("error reading 3d model file '%1%'") % model_file_path));
        }

        // Convert to unsigned char vector
        std::vector<unsigned char> temp_bytes_unsigned(temp_bytes.begin(), temp_bytes.end());
        result_model_parts.emplace(part, mesh{"model/gltf-binary", std::move(temp_bytes_unsigned)});
    }

    return result_model_parts;
}

void URArm::stop(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    stop_(rlock);
}

ProtoStruct URArm::do_command(const ProtoStruct& command) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    ProtoStruct resp = ProtoStruct{};

    // NOTE: Changes to these values will not be effective for any
    // trajectory currently being planned, and will only affect
    // trajectory planning initiated after these values have been
    // changed. Note that torn reads are also possible, since
    // `::move_` loads from these values independently.
    constexpr char k_vel_key[] = "set_vel";
    constexpr char k_acc_key[] = "set_acc";
    constexpr char k_vel_degs_key[] = "set_vel_degs_per_sec";
    constexpr char k_acc_degs_key[] = "set_accel_degs_per_sec2";
    constexpr char k_get_tcp_forces_base_key[] = "get_tcp_forces_base";
    constexpr char k_get_tcp_forces_tool_key[] = "get_tcp_forces_tool";
    constexpr char k_clear_pstop[] = "clear_pstop";
    constexpr char k_zero_ftsensor[] = "zero_ftsensor";
    constexpr char k_is_controllable[] = "is_controllable_state";
    constexpr char k_get_state_description[] = "get_state_description";
    constexpr char k_get_calibrated_dh_params[] = "get_calibrated_dh_params";

    // Cache TCP state to ensure atomic read of pose and forces from same timestamp
    std::optional<decltype(current_state_->read_tcp_state_snapshot())> cached_tcp_state;

    // cache state descriptors to ensure atomic read from the same timestamp
    struct controlled_info {
        bool controlled;
        std::string description;
    };
    std::optional<controlled_info> cached_controlled_info;

    const auto to_array = [](const vector6d_t& v) {
        std::vector<ProtoValue> arr;
        arr.reserve(v.size());
        for (size_t i = 0; i < v.size(); ++i) {
            arr.push_back(ProtoValue{v[i]});
        }
        return arr;
    };

    const auto add_limits_response = [&resp, &to_array](const std::string& key, const vector6d_t& limits_deg) {
        resp.emplace(key, to_array(limits_deg));
    };

    for (const auto& kv : command) {
        if (kv.first == k_vel_key || kv.first == k_vel_degs_key) {
            const auto limits_rad = parse_and_validate_joint_limits(kv.second, kv.first);
            auto result = current_state_->set_velocity_limits(limits_rad);
            add_limits_response(kv.first, radians_to_degrees(result));
        } else if (kv.first == k_acc_key || kv.first == k_acc_degs_key) {
            const auto limits_rad = parse_and_validate_joint_limits(kv.second, kv.first);
            auto result = current_state_->set_acceleration_limits(limits_rad);
            add_limits_response(kv.first, radians_to_degrees(result));
        } else if (kv.first == k_get_tcp_forces_base_key) {
            if (!cached_tcp_state) {
                cached_tcp_state = current_state_->read_tcp_state_snapshot();
            }
            const auto& tcp_force = cached_tcp_state->forces_at_base;
            ProtoStruct tcp_forces_base;
            tcp_forces_base.emplace("Fx_N", tcp_force[0]);
            tcp_forces_base.emplace("Fy_N", tcp_force[1]);
            tcp_forces_base.emplace("Fz_N", tcp_force[2]);
            tcp_forces_base.emplace("TRx_Nm", tcp_force[3]);
            tcp_forces_base.emplace("TRy_Nm", tcp_force[4]);
            tcp_forces_base.emplace("TRz_Nm", tcp_force[5]);
            resp.emplace("tcp_forces_base", std::move(tcp_forces_base));
        } else if (kv.first == k_get_tcp_forces_tool_key) {
            if (!cached_tcp_state) {
                cached_tcp_state = current_state_->read_tcp_state_snapshot();
            }
            const auto tcp_force = convert_tcp_force_to_tool_frame(cached_tcp_state->pose, cached_tcp_state->forces_at_base);
            ProtoStruct tcp_forces_tool;
            tcp_forces_tool.emplace("Fx_N", tcp_force[0]);
            tcp_forces_tool.emplace("Fy_N", tcp_force[1]);
            tcp_forces_tool.emplace("Fz_N", tcp_force[2]);
            tcp_forces_tool.emplace("TRx_Nm", tcp_force[3]);
            tcp_forces_tool.emplace("TRy_Nm", tcp_force[4]);
            tcp_forces_tool.emplace("TRz_Nm", tcp_force[5]);
            resp.emplace("tcp_forces_tool", std::move(tcp_forces_tool));
        } else if (kv.first == k_clear_pstop) {
            current_state_->clear_pstop();
            resp.emplace(k_clear_pstop, "protective stop cleared");
        } else if (kv.first == k_zero_ftsensor) {
            current_state_->zero_ftsensor();
            resp.emplace(k_zero_ftsensor, "force-torque sensor zeroed");
        } else if (kv.first == k_is_controllable) {
            if (!cached_controlled_info) {
                cached_controlled_info = controlled_info{};
                cached_controlled_info->controlled = current_state_->is_current_state_controlled(&cached_controlled_info->description);
            }
            resp.emplace(k_is_controllable, cached_controlled_info->controlled);
        } else if (kv.first == k_get_state_description) {
            if (!cached_controlled_info) {
                cached_controlled_info = controlled_info{};
                cached_controlled_info->controlled = current_state_->is_current_state_controlled(&cached_controlled_info->description);
            }
            resp.emplace(k_get_state_description, cached_controlled_info->description);
        } else if (kv.first == k_get_calibrated_dh_params) {
            const auto kin_info = current_state_->get_calibrated_kinematics_info(std::chrono::seconds{5});
            ProtoStruct dh;
            dh.emplace("a", to_array(kin_info.dh_a_));
            dh.emplace("d", to_array(kin_info.dh_d_));
            dh.emplace("alpha", to_array(kin_info.dh_alpha_));
            dh.emplace("theta", to_array(kin_info.dh_theta_));
            resp.emplace("calibrated_dh_params", std::move(dh));
        } else {
            throw std::runtime_error("unsupported do_command key: " + kv.first);
        }
    }

    return resp;
}

void URArm::move_tool_space_(std::shared_lock<std::shared_mutex> config_rlock, pose p, const URArm::move_id& id) {
    auto our_config_rlock = std::move(config_rlock);

    auto async_cancellation_monitor = [observer = GrpcContextObserver::current()]() {
        if (!observer) {
            return false;
        }

        return observer->context().IsCancelled();
    };

    VIAM_SDK_LOG(debug) << "move tool space: start id " << id.uuid << " p: " << p;
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(info) << "move tool space: end id " << id.uuid; });

    // get current pose
    auto current_pose = current_state_->read_tcp_pose();

    // convert viam pose to universal robots pose
    auto target_pose = pose_to_ur_vector(p);

    // if we are already at the desired pose, there is nothing to do
    constexpr double k_position_tolerance_m = 1e-3;  // 1 mm
    bool already_there = true;
    for (size_t i = 0; i != 3; ++i) {  // check XYZ
        if (std::abs(current_pose[i] - target_pose[i]) > k_position_tolerance_m) {
            already_there = false;
            break;
        }
    }
    constexpr double k_orientation_tolerance_rad = 1e-3;  // ~0.057 degrees
    for (size_t i = 3; i != 6 && already_there; ++i) {    // check orientation (rx, ry, rz)
        if (std::abs(current_pose[i] - target_pose[i]) > k_orientation_tolerance_rad) {
            already_there = false;
            break;
        }
    }
    if (already_there) {
        VIAM_SDK_LOG(debug) << "Already at desired pose; skipping movement.";
        return;
    }

    const std::string& telemetry_path = current_state_->telemetry_output_path();

    auto logger = std::make_unique<RealtimeTrajectoryLogger>(
        telemetry_path, id.uuid, arm_model_.sdk_model().to_string(), current_state_->resource_name());
    logger->set_velocity_limits(current_state_->get_velocity_limits());
    logger->set_acceleration_limits(current_state_->get_acceleration_limits());

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), logger = std::move(logger)]() mutable {
        return current_state_->start_move_request(id, std::move(logger), std::move(async_cancellation_monitor), pose_sample{target_pose});
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
}

void URArm::move_joint_space_(std::shared_lock<std::shared_mutex> config_rlock,
                              const xt::xarray<double>& waypoints,
                              const MoveOptions& options,
                              const URArm::move_id& id) {
    auto our_config_rlock = std::move(config_rlock);

    auto async_cancellation_monitor = [observer = GrpcContextObserver::current()]() {
        if (!observer) {
            return false;
        }

        auto result = observer->context().IsCancelled();
        return result;
    };

    VIAM_SDK_LOG(debug) << "move: start id " << id.uuid << " waypoints size " << waypoints.shape()[0];
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(debug) << "move: end id " << id.uuid; });

    // Prepare velocity/acceleration limits, applying per-move overrides if provided
    auto velocity_limits_data = current_state_->get_velocity_limits();
    if (options.max_vel_degs_per_sec) {
        apply_move_limit(velocity_limits_data, *options.max_vel_degs_per_sec);
        velocity_limits_data = current_state_->clamp_velocity_limits(velocity_limits_data);
    }

    auto acceleration_limits_data = current_state_->get_acceleration_limits();
    if (options.max_acc_degs_per_sec2) {
        apply_move_limit(acceleration_limits_data, *options.max_acc_degs_per_sec2);
        acceleration_limits_data = current_state_->clamp_acceleration_limits(acceleration_limits_data);
    }

    struct segment_accumulator {
        std::optional<trajectory_samples> samples;
        double total_duration = 0.0;
        std::chrono::microseconds total_generation_time{};
        size_t total_waypoints = 0;
        double total_arc_length = 0.0;
        size_t segment_count = 0;
    };

    VIAM_SDK_LOG(debug) << "move: compute_trajectory start " << id.uuid;

    auto planner = viam::trajex::totg::planner<segment_accumulator>({
        .velocity_limits = xt::adapt(velocity_limits_data),
        .acceleration_limits = xt::adapt(acceleration_limits_data),
        .path_blend_tolerance = current_state_->get_path_tolerance_delta_rads(),
        .colinearization_ratio = current_state_->get_path_colinearization_ratio(),
        .segment_totg = current_state_->segment_for_trajex(),
    });

    std::optional<viam::trajex::totg::waypoint_accumulator> captured_waypoints;

    planner
        .with_waypoint_provider([&](auto& p) {
            auto curr_joint_pos = get_joint_positions_rad_(our_config_rlock);
            const std::array<std::size_t, 2> shape{1, curr_joint_pos.size()};
            auto current_pos = p.stash(xt::xarray<double>{xt::adapt(curr_joint_pos, shape)});

            viam::trajex::totg::waypoint_accumulator accumulator{*current_pos};
            accumulator.add_waypoints(waypoints);

            captured_waypoints = accumulator;
            return accumulator;
        })

        .with_waypoint_preprocessor([&](auto&, viam::trajex::totg::waypoint_accumulator& accumulator) {
            accumulator =
                viam::trajex::totg::deduplicate_waypoints(accumulator, current_state_->get_waypoint_deduplication_tolerance_rad());
        })

        .with_move_validator([&](auto&, const viam::trajex::totg::waypoint_accumulator& accumulator) {
            if (const auto& threshold = current_state_->get_reject_move_request_threshold_rad()) {
                auto current_joint_position = accumulator.begin();
                auto first_waypoint = std::next(current_joint_position);
                const auto delta = *first_waypoint - *current_joint_position;
                const auto max_diff = xt::norm_linf(delta)();

                if (max_diff > *threshold) {
                    std::stringstream err_string;
                    err_string << "rejecting move request : difference between starting trajectory position [(";
                    boost::copy(boost::adaptors::transform(*first_waypoint, radians_to_degrees<const double&>),
                                boost::io::make_ostream_joiner(err_string, ", "));
                    err_string << ")] and joint position [(";
                    boost::copy(boost::adaptors::transform(*current_joint_position, radians_to_degrees<const double&>),
                                boost::io::make_ostream_joiner(err_string, ", "));
                    err_string << ")] is above threshold " << viam::trajex::radians_to_degrees(max_diff) << " > "
                               << viam::trajex::radians_to_degrees(*threshold);
                    VIAM_SDK_LOG(error) << err_string.str();
                    throw std::runtime_error(err_string.str());
                }
            }
        })

        .with_segmenter([&](auto&, viam::trajex::totg::waypoint_accumulator accumulator) {
            return viam::trajex::totg::segment_at_reversals(std::move(accumulator), current_state_->get_segmentation_threshold());
        });

    if (current_state_->use_new_trajectory_planner()) {
        planner.with_totg(
            [&](const auto&,
                segment_accumulator& acc,
                const viam::trajex::totg::waypoint_accumulator& segment,
                viam::trajex::totg::trajectory&& traj,
                auto elapsed) {
                acc.total_generation_time += elapsed;
                acc.total_duration += traj.duration().count();
                acc.total_waypoints += segment.size();
                acc.total_arc_length += static_cast<double>(traj.path().length());
                acc.segment_count++;

                auto sampler = viam::trajex::totg::uniform_sampler::quantized_for_trajectory(
                    traj, viam::trajex::types::hertz{current_state_->get_trajectory_sampling_freq_hz()});

                double previous_time = 0.0;
                if (!acc.samples) {
                    acc.samples = current_state_->prefer_precomputed_accelerations()
                                      ? trajectory_samples{std::vector<trajectory_sample_point_pva>{}}
                                      : trajectory_samples{std::vector<trajectory_sample_point_pv>{}};
                }
                std::visit(
                    [&](auto& dest) {
                        using PointType = typename std::decay_t<decltype(dest)>::value_type;
                        // TODO(RSDK-14268): investigate whether dropping the first sample is still correct
                        // now that the trajectory is seeded with the arm's measured position.
                        // It also predates PVA support, and dropping a t=0 point that carries a
                        // departure acceleration is very likely wrong.
                        for (const auto& sample : traj.samples(sampler) | std::views::drop(1)) {
                            const double current_time = sample.time.count();
                            const float timestep = boost::numeric_cast<float>(current_time - previous_time);
                            PointType point;
                            for (size_t i = 0; i < point.p.size(); ++i) {
                                point.p[i] = sample.configuration(i);
                                point.v[i] = sample.velocity(i);
                                if constexpr (requires { point.a; }) {
                                    point.a[i] = sample.acceleration(i);
                                }
                            }
                            point.timestep = timestep;
                            dest.push_back(point);
                            previous_time = current_time;
                        }
                    },
                    *acc.samples);

                const auto sample_count = std::visit([](const auto& v) { return v.size(); }, *acc.samples);
                VIAM_SDK_LOG(info) << "trajex/totg segment generated successfully"
                                   << ", waypoints: " << segment.size() << ", duration: " << traj.duration().count()
                                   << "s, samples: " << sample_count << ", arc length: " << traj.path().length();
            },
            [&](const auto& planner,
                const segment_accumulator& acc,
                const viam::trajex::totg::waypoint_accumulator& seg,
                const std::exception& e) {
                VIAM_SDK_LOG(warn) << "trajectory generation with trajex failed with an exception"
                                   << ", waypoints: " << acc.total_waypoints << ", exception: " << e.what();
                const std::string json_content = planner.serialize_for_replay(seg, e.what());
                const auto filename = failed_trajectory_filename(
                    current_state_->telemetry_output_path(), current_state_->resource_name() + "_trajex", id.uuid);
                std::ofstream json_file(filename);
                json_file << json_content;
            });
    }

    planner.with_legacy(
        [&](const auto&,
            segment_accumulator& acc,
            const viam::trajex::totg::waypoint_accumulator& segment,
            Path&& legacy_path,
            Trajectory&& traj,
            auto elapsed) {
            acc.total_generation_time += elapsed;
            acc.total_duration += traj.getDuration();
            acc.total_waypoints += segment.size();
            acc.total_arc_length += legacy_path.getLength();
            acc.segment_count++;

            if (!acc.samples) {
                acc.samples = std::vector<trajectory_sample_point_pv>{};
            }
            auto& pv_samples = std::get<std::vector<trajectory_sample_point_pv>>(*acc.samples);
            viam::trajex::totg::legacy::for_each_sample(
                traj.getDuration(), current_state_->get_trajectory_sampling_freq_hz(), [&](const double t, const double step) {
                    auto p_eigen = traj.getPosition(t);
                    auto v_eigen = traj.getVelocity(t);
                    pv_samples.push_back(
                        trajectory_sample_point_pv{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                                   {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                                   boost::numeric_cast<float>(step)});
                });
        },
        [&](const auto& planner, const segment_accumulator&, const viam::trajex::totg::waypoint_accumulator& seg, const std::exception& e) {
            VIAM_SDK_LOG(error) << "trajectory generation with legacy failed with an exception: " << e.what();
            const std::string json_content = planner.serialize_for_replay(seg, e.what());
            const auto filename =
                failed_trajectory_filename(current_state_->telemetry_output_path(), current_state_->resource_name(), id.uuid);
            std::ofstream json_file(filename);
            json_file << json_content;
        });

    auto result = planner.execute([&](const auto& p, auto trajex, auto legacy) -> std::optional<segment_accumulator> {
        // Log trajex summary if it ran
        if (trajex.receiver) {
            const auto sample_count = std::visit([](const auto& v) { return v.size(); }, *trajex.receiver->samples);
            VIAM_SDK_LOG(info) << "trajex/totg trajectory generated successfully"
                               << ", total waypoints: " << trajex.receiver->total_waypoints
                               << ", total duration: " << trajex.receiver->total_duration << "s, total samples: " << sample_count
                               << ", total arc length: " << trajex.receiver->total_arc_length
                               << ", generation_time: " << std::chrono::duration<double>(trajex.receiver->total_generation_time).count()
                               << "s, number of segments: " << trajex.receiver->segment_count;
        }

        // Log legacy summary. When the new planner is active, log at info for comparison
        // visibility; otherwise log at debug (routine operation).
        if (legacy.receiver && current_state_->use_new_trajectory_planner()) {
            const auto sample_count = std::visit([](const auto& v) { return v.size(); }, *legacy.receiver->samples);
            VIAM_SDK_LOG(info) << "legacy trajectory generated successfully"
                               << ", waypoints: " << legacy.receiver->total_waypoints << ", duration: " << legacy.receiver->total_duration
                               << "s, samples: " << sample_count
                               << ", generation_time: " << std::chrono::duration<double>(legacy.receiver->total_generation_time).count()
                               << "s, number of segments: " << legacy.receiver->segment_count;
        } else if (legacy.receiver) {
            const auto sample_count = std::visit([](const auto& v) { return v.size(); }, *legacy.receiver->samples);
            VIAM_SDK_LOG(debug) << "legacy trajectory generated successfully"
                                << ", waypoints: " << legacy.receiver->total_waypoints << ", duration: " << legacy.receiver->total_duration
                                << "s, samples: " << sample_count
                                << ", generation_time: " << std::chrono::duration<double>(legacy.receiver->total_generation_time).count()
                                << "s, number of segments: " << legacy.receiver->segment_count;
        }

        // Prefer trajex when enabled and successful
        if (trajex.receiver) {
            return std::move(trajex.receiver);
        }

        // Otherwise return the legacy results.
        if (legacy.receiver) {
            return std::move(legacy.receiver);
        }

        // Legacy is the authoritative algorithm, so its errors take priority.
        if (legacy.error) {
            std::rethrow_exception(legacy.error);
        }

        // Trajex errors are only fatal when legacy isn't registered (future:
        // legacy retired). When both run and both fail, legacy's error above
        // takes precedence.
        if (trajex.error) {
            std::rethrow_exception(trajex.error);
        }

        // No results and no errors: if the input had fewer than 2 waypoints after
        // preprocessing, no algorithms ran, so there was nothing to do.
        if (p.processed_waypoint_count() < 2) {
            VIAM_SDK_LOG(debug) << "arm is already at the desired joint positions";
            return std::nullopt;
        }

        // Unreachable while at least one algorithm is registered and
        // we didn't have degenerate waypoints: the planner guarantees
        // each algorithm produces either a receiver or an error.
        throw std::logic_error("trajectory generation returned neither results nor an error");
    });

    {
        const auto& timing = planner.timing();
        const auto to_ms = [](std::chrono::microseconds d) { return static_cast<double>(d.count()) / 1000.0; };
        const auto format_optional = [&to_ms](const std::optional<std::chrono::microseconds>& d) {
            return d ? std::to_string(to_ms(*d)) + "ms" : std::string("n/a");
        };
        VIAM_SDK_LOG(debug) << "move: phase timing (ms):"
                            << " provisioning=" << to_ms(timing.waypoint_provisioning) << "ms"
                            << " preprocessing=" << format_optional(timing.waypoint_preprocessing)
                            << " validation=" << format_optional(timing.move_validation)
                            << " segmentation=" << format_optional(timing.segmentation)
                            << " colinearization=" << format_optional(timing.colinearization)
                            << " totg_total_gen=" << format_optional(timing.totg_generation_total)
                            << " legacy_total_gen=" << format_optional(timing.legacy_generation_total);
    }

    if (!result) {
        VIAM_SDK_LOG(debug) << "move: compute_trajectory end " << id.uuid << " (no result)";
        return;
    } else if (result->total_duration > current_state_->get_max_trajectory_duration_secs()) {
        throw std::runtime_error("total trajectory duration exceeds maximum allowed duration");
    } else if (result->total_duration < k_min_timestep_sec) {
        VIAM_SDK_LOG(debug) << "move: compute_trajectory end " << id.uuid << " (duration too small)";
        return;
    } else {
        const auto sample_count = std::visit([](const auto& v) { return v.size(); }, *result->samples);
        VIAM_SDK_LOG(debug) << "move: compute_trajectory end " << id.uuid << " samples.size() " << sample_count;
    }

    const std::string& telemetry_path = current_state_->telemetry_output_path();

    auto logger = std::make_unique<RealtimeTrajectoryLogger>(
        telemetry_path, id.uuid, arm_model_.sdk_model().to_string(), current_state_->resource_name());
    logger->set_velocity_limits(current_state_->get_velocity_limits());
    logger->set_acceleration_limits(current_state_->get_acceleration_limits());
    if (captured_waypoints) {
        logger->set_waypoints(*captured_waypoints);
    }
    logger->set_planned_trajectory(*result->samples);

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), logger = std::move(logger)]() mutable {
        return current_state_->start_move_request(
            id, std::move(logger), std::move(async_cancellation_monitor), std::move(*result->samples));
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.

    try {
        VIAM_SDK_LOG(debug) << "move: trajectory enqueued to arm: id " << id.uuid;
        trajectory_completion_future.get();
        VIAM_SDK_LOG(debug) << "move: trajectory completed on arm: id " << id.uuid << " (success)";
    } catch (...) {
        VIAM_SDK_LOG(debug) << "move: trajectory failed on arm: id " << id.uuid << " (failure)";
        throw;
    }
}

// Define the destructor
URArm::~URArm() {
    VIAM_SDK_LOG(info) << "Shutting down URArm driver instance for arm model: " << arm_model_.sdk_model().to_string();
    const std::unique_lock wlock{config_mutex_};
    shutdown_(wlock);
}

template <template <typename> typename lock_type>
void URArm::stop_(const lock_type<std::shared_mutex>&) {
    if (auto cancel_future = current_state_->cancel_move_request()) {
        cancel_future->get();
    }
}

void URArm::shutdown_(const std::unique_lock<std::shared_mutex>& lock) noexcept {
    try {
        if (current_state_) {
            const auto destroy_state = make_scope_guard([&] { current_state_.reset(); });

            stop_(lock);
            current_state_->shutdown();
        }
        VIAM_SDK_LOG(info) << "URArm shutdown complete";

    } catch (...) {
        const auto unconditional_abort = make_scope_guard([] { std::abort(); });
        try {
            throw;
        } catch (const std::exception& ex) {
            VIAM_SDK_LOG(error) << "URArm shutdown failed with a std::exception - module service will terminate: " << ex.what();
        } catch (...) {
            VIAM_SDK_LOG(error) << "URArm shutdown failed with an unknown exception - module service will terminate";
        }
    }
}

// We need to requisition different ports for each independent URArm
// instance, otherwise they will all try to use the same ports and
// only one of them will work.
URArm::ports_::ports_() {
    static std::atomic<std::uint32_t> counter{50001};
    reverse_port = counter.fetch_add(4);
    script_sender_port = reverse_port + 1;
    trajectory_port = script_sender_port + 1;
    script_command_port = trajectory_port + 1;
}

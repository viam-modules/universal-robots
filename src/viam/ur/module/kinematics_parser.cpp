#include "kinematics_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <json/json.h>

// rust-utils FFI: lets us delegate the `ov_degrees` -> quaternion conversion
// (RDK's canonical orientation-vector algorithm) to the same Rust
// implementation already used by `ur_arm.cpp`, rather than reimplementing it
// here in Eigen. Matches the unprefixed deprecated names ur_arm.cpp uses.
extern "C" void* new_orientation_vector(double ox, double oy, double oz, double theta);
extern "C" void free_orientation_vector_memory(void* ov);
extern "C" void* quaternion_from_orientation_vector(void* ov);
extern "C" void free_quaternion_memory(void* q);
extern "C" double* quaternion_get_components(void* q);
extern "C" void free_quaternion_components(double* ptr);

namespace {

constexpr std::size_t k_num_dh_joints = 6;

// Guards against a malformed chain creating a runaway walk (cycle, or a
// pathologically long sequence of intermediate static links).
constexpr std::size_t k_max_chain_walk_steps = 64;

[[noreturn]] void throw_parse_error(const std::filesystem::path& path, const std::string& msg) {
    throw std::invalid_argument("parse_kinematics: " + path.string() + ": " + msg);
}

// Returns the first numeric value among `keys` found in `obj`, or `fallback`
// if none are present. Lets us absorb the x/y/z vs X/Y/Z divergence between
// the shipped JSONs without committing to a normalization pass.
double pick_double(const Json::Value& obj, std::initializer_list<const char*> keys, double fallback) {
    for (const char* k : keys) {
        if (obj.isMember(k) && obj[k].isNumeric()) {
            return obj[k].asDouble();
        }
    }
    return fallback;
}

// Read a `translation` sub-object's x/y/z (accepting either case). Missing
// or null block -> zero vector.
Eigen::Vector3d parse_translation(const Json::Value& parent) {
    if (!parent.isMember("translation") || parent["translation"].isNull()) {
        return Eigen::Vector3d::Zero();
    }
    const Json::Value& t = parent["translation"];
    return Eigen::Vector3d{
        pick_double(t, {"x", "X"}, 0.0),
        pick_double(t, {"y", "Y"}, 0.0),
        pick_double(t, {"z", "Z"}, 0.0),
    };
}

// Delegate the `ov_degrees` -> quaternion conversion to rust-utils, which is
// RDK's canonical implementation; see
// https://github.com/viamrobotics/rdk/blob/main/spatialmath/orientationVector.go
Eigen::Quaterniond ov_degrees_to_quaternion(double x, double y, double z, double th_deg) {
    const double theta_rad = th_deg * std::numbers::pi / 180.0;
    const std::unique_ptr<void, decltype(&free_orientation_vector_memory)> ov{
        new_orientation_vector(x, y, z, theta_rad), &free_orientation_vector_memory};
    const std::unique_ptr<void, decltype(&free_quaternion_memory)> q{quaternion_from_orientation_vector(ov.get()),
                                                                     &free_quaternion_memory};
    const std::unique_ptr<double[], decltype(&free_quaternion_components)> comps{quaternion_get_components(q.get()),
                                                                                 &free_quaternion_components};
    return Eigen::Quaterniond{comps[0], comps[1], comps[2], comps[3]};
}

// Read an `orientation` sub-object, handling the three types the shipped
// JSONs use. Missing or null block -> identity. (ur3e ships explicit
// `"orientation": null` on wrist_3_link and base_link.)
Eigen::Quaterniond parse_orientation(const std::filesystem::path& path, const Json::Value& parent) {
    if (!parent.isMember("orientation") || parent["orientation"].isNull()) {
        return Eigen::Quaterniond::Identity();
    }
    const Json::Value& o = parent["orientation"];
    if (!o.isMember("type") || !o["type"].isString()) {
        throw_parse_error(path, "orientation block missing string `type` field");
    }
    const std::string type = o["type"].asString();
    if (!o.isMember("value")) {
        throw_parse_error(path, "orientation block missing `value` field");
    }
    const Json::Value& v = o["value"];

    if (type == "euler_angles") {
        // RDK's ZYX intrinsic: R = Rz(yaw) * Ry(pitch) * Rx(roll), radians.
        const double pitch = pick_double(v, {"pitch"}, 0.0);
        const double roll = pick_double(v, {"roll"}, 0.0);
        const double yaw = pick_double(v, {"yaw"}, 0.0);
        const Eigen::Quaterniond q =                              //
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *    //
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *  //
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());    //
        return q.normalized();
    }
    if (type == "quaternion") {
        const double qw = pick_double(v, {"W", "w"}, 1.0);
        const double qx = pick_double(v, {"X", "x"}, 0.0);
        const double qy = pick_double(v, {"Y", "y"}, 0.0);
        const double qz = pick_double(v, {"Z", "z"}, 0.0);
        return Eigen::Quaterniond{qw, qx, qy, qz}.normalized();
    }
    if (type == "ov_degrees") {
        return ov_degrees_to_quaternion(pick_double(v, {"x", "X"}, 0.0),
                                        pick_double(v, {"y", "Y"}, 0.0),
                                        pick_double(v, {"z", "Z"}, 0.0),
                                        pick_double(v, {"th", "Th", "TH"}, 0.0));
    }
    throw_parse_error(path, "unsupported orientation type `" + type + "`");
}

Eigen::Matrix4d to_homogeneous(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3, 3>(0, 0) = q.toRotationMatrix();
    M.block<3, 1>(0, 3) = t;
    return M;
}

// A link/joint entry's local-frame transform: T(translation) * R(orientation),
// translation in millimeters (the shipped JSONs always store millimeters).
Eigen::Matrix4d local_transform(const std::filesystem::path& path, const Json::Value& entry) {
    return to_homogeneous(parse_orientation(path, entry), parse_translation(entry));
}

// Pull the geometry pose out as a homogeneous matrix (mm + quaternion).
Eigen::Matrix4d geometry_local_transform(const std::filesystem::path& path, const Json::Value& geom) {
    return to_homogeneous(parse_orientation(path, geom), parse_translation(geom));
}

// Compose `geom`'s local pose with the running world transform `W` and return
// a world-frame Geometry. Returns nullopt for "intentionally absent"
// geometries (no `geometry` block, or a box -- ur3e's base_link placeholder).
std::optional<Geometry> parse_geometry_world(const std::filesystem::path& path,
                                             const Json::Value& link_entry,
                                             const Eigen::Matrix4d& W) {
    if (!link_entry.isMember("geometry")) {
        return std::nullopt;
    }
    const Json::Value& g = link_entry["geometry"];

    // Resolve type: explicit `type` wins, otherwise presence-of-`l` is the
    // discriminator (ur5e/ur7e omit `type`).
    std::string type;
    if (g.isMember("type") && g["type"].isString()) {
        type = g["type"].asString();
    } else {
        type = g.isMember("l") ? "capsule" : "sphere";
    }
    if (type == "box") {
        return std::nullopt;
    }
    if (type != "capsule" && type != "sphere") {
        throw_parse_error(path, "unsupported geometry type `" + type + "`");
    }

    if (!g.isMember("r") || !g["r"].isNumeric()) {
        throw_parse_error(path, "geometry missing required `r` (radius)");
    }
    const double r = g["r"].asDouble();

    const Eigen::Matrix4d G_world = W * geometry_local_transform(path, g);
    const Eigen::Vector3d t = G_world.block<3, 1>(0, 3);

    if (type == "sphere") {
        return Geometry{SphereGeometry{
            /* radius_mm */ r,
            /* tx_mm */ t.x(),
            /* ty_mm */ t.y(),
            /* tz_mm */ t.z(),
        }};
    }

    // capsule
    if (!g.isMember("l") || !g["l"].isNumeric()) {
        throw_parse_error(path, "capsule geometry missing required `l` (length)");
    }
    const double l = g["l"].asDouble();
    const Eigen::Quaterniond q{G_world.block<3, 3>(0, 0)};
    const Eigen::Quaterniond qn = q.normalized();
    return Geometry{CapsuleGeometry{
        /* radius_mm */ r,
        /* length_mm */ l,
        /* tx_mm */ t.x(),
        /* ty_mm */ t.y(),
        /* tz_mm */ t.z(),
        /* qw */ qn.w(),
        /* qx */ qn.x(),
        /* qy */ qn.y(),
        /* qz */ qn.z(),
    }};
}

JointLimits parse_joint_limits(const std::filesystem::path& path, const Json::Value& joint) {
    if (!joint.isMember("min") || !joint["min"].isNumeric()) {
        throw_parse_error(path, "joint missing numeric `min`");
    }
    if (!joint.isMember("max") || !joint["max"].isNumeric()) {
        throw_parse_error(path, "joint missing numeric `max`");
    }
    return JointLimits{joint["min"].asDouble(), joint["max"].asDouble()};
}

}  // namespace

ModelTables parse_kinematics(const std::filesystem::path& sva_json_path) {
    std::ifstream in(sva_json_path);
    if (!in) {
        throw_parse_error(sva_json_path, "unable to open file");
    }

    Json::Value root;
    Json::CharReaderBuilder reader_builder;
    std::string errs;
    if (!Json::parseFromStream(reader_builder, in, &root, &errs)) {
        throw_parse_error(sva_json_path, "JSON parse failure: " + errs);
    }

    if (!root.isMember("kinematic_param_type") || root["kinematic_param_type"].asString() != "SVA") {
        throw_parse_error(sva_json_path, "expected `kinematic_param_type: \"SVA\"`");
    }
    if (!root.isMember("links") || !root["links"].isArray()) {
        throw_parse_error(sva_json_path, "missing `links` array");
    }
    if (!root.isMember("joints") || !root["joints"].isArray()) {
        throw_parse_error(sva_json_path, "missing `joints` array");
    }

    // Build `parent_id -> child_entity` so we can walk the chain forward from
    // "world" without ever looking up entries by name. Per the shipped JSONs
    // each parent has at most one child (no branching); we enforce that here.
    std::unordered_map<std::string, const Json::Value*> child_of;
    auto index_entry = [&](const Json::Value& entry, const char* kind) {
        if (!entry.isMember("id") || !entry["id"].isString()) {
            throw_parse_error(sva_json_path, std::string{kind} + " entry missing string `id`");
        }
        // A missing or null `parent` field is treated as "world": ur3e omits
        // `parent` on `base_link` entirely, while ur5e/ur7e/ur20 spell it as
        // an explicit `"parent": "world"`. Both forms denote the chain root.
        const std::string parent =
            (entry.isMember("parent") && entry["parent"].isString()) ? entry["parent"].asString() : std::string{"world"};
        const auto [it, inserted] = child_of.emplace(parent, &entry);
        if (!inserted) {
            throw_parse_error(sva_json_path, "parent `" + parent + "` has multiple children (chain must be linear)");
        }
    };
    for (const auto& link : root["links"]) {
        index_entry(link, "link");
    }
    for (const auto& joint : root["joints"]) {
        index_entry(joint, "joint");
    }

    // `axis` is on joints; links don't have it. Use it as the joint/link discriminator.
    const auto is_joint = [](const Json::Value& entry) { return entry.isMember("axis"); };

    const auto next_in_chain = [&](const Json::Value& entry) -> const Json::Value* {
        const auto it = child_of.find(entry["id"].asString());
        return (it == child_of.end()) ? nullptr : it->second;
    };

    // Chain root: the entity whose parent is "world". Must be a link.
    const auto root_it = child_of.find("world");
    if (root_it == child_of.end()) {
        throw_parse_error(sva_json_path, "no entity has parent `world`");
    }
    const Json::Value& chain_root = *root_it->second;
    if (is_joint(chain_root)) {
        throw_parse_error(sva_json_path, "entity attached directly to `world` must be a link, got a joint");
    }

    ModelTables out{};
    out.link_names[0] = chain_root["id"].asString();

    // base_link's frame *is* the world frame (parent="world"), so its
    // geometry's local pose is already world-frame. ur5e/ur7e ship a real
    // base capsule here; ur20 ships a 1mm placeholder sphere; ur3e ships a
    // box which `parse_geometry_world` intentionally drops.
    out.geometries[0] = parse_geometry_world(sva_json_path, chain_root, Eigen::Matrix4d::Identity());

    // Walk the chain forward, accumulating W as we go. Each link composes its
    // own local transform into W (so W is the cumulative pose at the link's
    // *next* slot in the chain -- which, by the SVA convention, is the frame
    // in which the next link's geometry lives). Joints contribute identity
    // at zero joint state and are not composed.
    Eigen::Matrix4d W = local_transform(sva_json_path, chain_root);
    const Json::Value* cursor = &chain_root;
    std::size_t dh_count = 0;
    std::size_t steps = 0;
    while (dh_count < k_num_dh_joints) {
        if (steps++ > k_max_chain_walk_steps) {
            throw_parse_error(sva_json_path, "chain walk exceeded max steps (cycle or pathologically long chain?)");
        }
        const Json::Value* next = next_in_chain(*cursor);
        if (next == nullptr) {
            throw_parse_error(sva_json_path,
                              "chain terminated after " + std::to_string(dh_count) + " DH joints; expected " +
                                  std::to_string(k_num_dh_joints));
        }
        if (!is_joint(*next)) {
            // Intermediate static link (e.g., ur3e's
            // `base_link-base_link_inertia` and `base_link_inertia` between
            // base_link and shoulder_pan_joint). Compose its transform and
            // continue.
            W = W * local_transform(sva_json_path, *next);
            cursor = next;
            continue;
        }

        // `next` is a DH joint. Record its limits, then advance to its child
        // link (the DH-frame-i link). The joint itself contributes identity
        // to W.
        out.limits[dh_count] = parse_joint_limits(sva_json_path, *next);
        const Json::Value* dh_link = next_in_chain(*next);
        if (dh_link == nullptr) {
            throw_parse_error(sva_json_path, "DH joint `" + next->operator[]("id").asString() + "` has no child link");
        }
        if (is_joint(*dh_link)) {
            throw_parse_error(sva_json_path,
                              "DH joint `" + next->operator[]("id").asString() + "` child must be a link, got a joint");
        }

        // Geometry on `dh_link` sits in its parent (joint) frame, which at
        // zero joints is the current W.
        out.link_names[dh_count + 1] = (*dh_link)["id"].asString();
        out.geometries[dh_count + 1] = parse_geometry_world(sva_json_path, *dh_link, W);

        W = W * local_transform(sva_json_path, *dh_link);
        cursor = dh_link;
        dh_count++;
    }

    return out;
}

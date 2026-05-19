#include "kinematics_parser.hpp"

#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>

#include <json/json.h>

#include "utils.hpp"  // degrees_to_radians / radians_to_degrees

// rust-utils FFI: lets us delegate the orientation -> quaternion conversions
// (RDK's canonical algorithms) to the same Rust implementation already used
// by `ur_arm.cpp`, rather than reimplementing them here. Matches the
// unprefixed deprecated names ur_arm.cpp uses.
extern "C" void* new_orientation_vector(double ox, double oy, double oz, double theta);
extern "C" void free_orientation_vector_memory(void* ov);
extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_components(double* ptr);
extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" void* quaternion_from_orientation_vector(void* ov);
extern "C" void* quaternion_from_euler_angles(double roll, double pitch, double yaw);
extern "C" void* new_quaternion(double real, double i, double j, double k);
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

// Read W/X/Y/Z out of a rust-utils Quaternion handle into Eigen.
Eigen::Quaterniond eigen_from_rust_quaternion(void* q) {
    const std::unique_ptr<double[], decltype(&free_quaternion_components)> comps{quaternion_get_components(q), &free_quaternion_components};
    return Eigen::Quaterniond{comps[0], comps[1], comps[2], comps[3]};
}

// Build a viam-cpp-sdk pose from an Eigen translation+quaternion pair by
// converting the quaternion to an orientation-vector via rust-utils. The pose
// stores its translation in mm (whatever unit the caller supplies) and its
// `theta` field in degrees.
viam::sdk::pose pose_from_translation_quaternion(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    const std::unique_ptr<void, decltype(&free_quaternion_memory)> rust_q{new_quaternion(q.w(), q.x(), q.y(), q.z()),
                                                                          &free_quaternion_memory};
    const std::unique_ptr<void, decltype(&free_orientation_vector_memory)> rust_ov{orientation_vector_from_quaternion(rust_q.get()),
                                                                                   &free_orientation_vector_memory};
    const std::unique_ptr<double[], decltype(&free_orientation_vector_components)> comps{orientation_vector_get_components(rust_ov.get()),
                                                                                         &free_orientation_vector_components};
    return viam::sdk::pose{
        viam::sdk::coordinates{t.x(), t.y(), t.z()},
        viam::sdk::pose_orientation{comps[0], comps[1], comps[2]},
        radians_to_degrees(comps[3]),
    };
}

// Inverse of `pose_from_translation_quaternion`: pull a viam-cpp-sdk pose
// back into an Eigen translation + quaternion via rust-utils.
std::pair<Eigen::Vector3d, Eigen::Quaterniond> translation_quaternion_from_pose(const viam::sdk::pose& p) {
    const std::unique_ptr<void, decltype(&free_orientation_vector_memory)> rust_ov{
        new_orientation_vector(p.orientation.o_x, p.orientation.o_y, p.orientation.o_z, degrees_to_radians(p.theta)),
        &free_orientation_vector_memory};
    const std::unique_ptr<void, decltype(&free_quaternion_memory)> rust_q{quaternion_from_orientation_vector(rust_ov.get()),
                                                                          &free_quaternion_memory};
    return {Eigen::Vector3d{p.coordinates.x, p.coordinates.y, p.coordinates.z}, eigen_from_rust_quaternion(rust_q.get())};
}

// Delegate the `ov_degrees` -> quaternion conversion to rust-utils, which is
// RDK's canonical implementation; see
// https://github.com/viamrobotics/rdk/blob/main/spatialmath/orientationVector.go
Eigen::Quaterniond ov_degrees_to_quaternion(double x, double y, double z, double th_deg) {
    const double theta_rad = degrees_to_radians(th_deg);
    const std::unique_ptr<void, decltype(&free_orientation_vector_memory)> ov{new_orientation_vector(x, y, z, theta_rad),
                                                                              &free_orientation_vector_memory};
    const std::unique_ptr<void, decltype(&free_quaternion_memory)> q{quaternion_from_orientation_vector(ov.get()), &free_quaternion_memory};
    return eigen_from_rust_quaternion(q.get());
}

// Delegate the `euler_angles` -> quaternion conversion to rust-utils
// (Z-Y'-X" intrinsic Tait-Bryan; Z=yaw, Y=pitch, X=roll).
Eigen::Quaterniond euler_angles_to_quaternion(double roll, double pitch, double yaw) {
    const std::unique_ptr<void, decltype(&free_quaternion_memory)> q{quaternion_from_euler_angles(roll, pitch, yaw),
                                                                     &free_quaternion_memory};
    return eigen_from_rust_quaternion(q.get());
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
        return euler_angles_to_quaternion(pick_double(v, {"roll"}, 0.0), pick_double(v, {"pitch"}, 0.0), pick_double(v, {"yaw"}, 0.0));
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

// The local-frame transform of any JSON node with optional `translation` and
// `orientation` sub-objects (chain links, joints, or geometry blocks):
// T(translation) * R(orientation), translation in millimeters.
Eigen::Matrix4d local_transform(const std::filesystem::path& path, const Json::Value& node) {
    return to_homogeneous(parse_orientation(path, node), parse_translation(node));
}

// Compose `geom`'s local pose with the running world transform `W` and return
// a world-frame Geometry. Returns nullopt for "intentionally absent"
// geometries (no `geometry` block, or a box -- ur3e's base_link placeholder).
std::optional<Geometry> parse_geometry_world(const std::filesystem::path& path, const Json::Value& link_entry, const Eigen::Matrix4d& W) {
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

    const Eigen::Matrix4d G_world = W * local_transform(path, g);
    const Eigen::Vector3d t = G_world.block<3, 1>(0, 3);
    const Eigen::Quaterniond q = Eigen::Quaterniond{G_world.block<3, 3>(0, 0)}.normalized();
    const viam::sdk::pose pose = pose_from_translation_quaternion(t, q);

    if (type == "sphere") {
        return Geometry{pose, viam::sdk::sphere{r}};
    }

    // capsule
    if (!g.isMember("l") || !g["l"].isNumeric()) {
        throw_parse_error(path, "capsule geometry missing required `l` (length)");
    }
    return Geometry{pose, viam::sdk::capsule{r, g["l"].asDouble()}};
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
            throw_parse_error(
                sva_json_path,
                "chain terminated after " + std::to_string(dh_count) + " DH joints; expected " + std::to_string(k_num_dh_joints));
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
            throw_parse_error(sva_json_path, "DH joint `" + (*next)["id"].asString() + "` has no child link");
        }
        if (is_joint(*dh_link)) {
            throw_parse_error(sva_json_path, "DH joint `" + (*next)["id"].asString() + "` child must be a link, got a joint");
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

viam::sdk::pose apply_correction_to_pose(const viam::sdk::pose& p, const Eigen::Matrix4d& correction) {
    const auto [t_in, q_in] = translation_quaternion_from_pose(p);

    Eigen::Matrix4d G_in = Eigen::Matrix4d::Identity();
    G_in.block<3, 3>(0, 0) = q_in.toRotationMatrix();
    G_in.block<3, 1>(0, 3) = t_in;

    const Eigen::Matrix4d G_out = correction * G_in;
    const Eigen::Vector3d t_out = G_out.block<3, 1>(0, 3);
    const Eigen::Quaterniond q_out = Eigen::Quaterniond{G_out.block<3, 3>(0, 0)}.normalized();

    return pose_from_translation_quaternion(t_out, q_out);
}

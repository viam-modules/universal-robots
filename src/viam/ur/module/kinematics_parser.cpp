#include "kinematics_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <numbers>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <json/json.h>

namespace {

// The six DH joints in canonical DH index order. Uniform across ur3e, ur5e,
// ur7e, ur20 -- verified in the spec at
// docs/superpowers/specs/2026-05-18-construct-model-table-from-json-design.md.
constexpr std::array<const char*, 6> k_dh_joint_names = {
    "shoulder_pan_joint",   //
    "shoulder_lift_joint",  //
    "elbow_joint",          //
    "wrist_1_joint",        //
    "wrist_2_joint",        //
    "wrist_3_joint",        //
};

// Guards against a malformed parent chain creating a cycle.
constexpr std::size_t k_max_chain_walk_depth = 64;

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

// RDK's `OrientationVectorDegrees` algorithm: (x,y,z) is a point on the unit
// sphere giving the rotated z-axis, and `th` (degrees) is the roll about that
// axis. See
// https://github.com/viamrobotics/rdk/blob/main/spatialmath/orientationVector.go
Eigen::Quaterniond ov_degrees_to_quaternion(double x, double y, double z, double th_deg) {
    Eigen::Vector3d v{x, y, z};
    const double n = v.norm();
    if (n < 1e-12) {
        v = Eigen::Vector3d::UnitZ();
    } else {
        v /= n;
    }
    const double theta_rad = th_deg * std::numbers::pi / 180.0;
    const double lat = std::acos(std::clamp(v.z(), -1.0, 1.0));
    const double lon = ((1.0 - std::abs(v.z())) > 1e-4) ? std::atan2(v.y(), v.x()) : 0.0;
    const Eigen::Quaterniond q =                                       //
        Eigen::AngleAxisd(lon, Eigen::Vector3d::UnitZ()) *             //
        Eigen::AngleAxisd(lat, Eigen::Vector3d::UnitY()) *             //
        Eigen::AngleAxisd(theta_rad, Eigen::Vector3d::UnitZ());        //
    return q.normalized();
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

    // Index links and joints by id. Both are needed by the parent-chain walk:
    // a "parent" name resolves against either map.
    std::unordered_map<std::string, const Json::Value*> links_by_id;
    std::unordered_map<std::string, const Json::Value*> joints_by_id;
    for (const auto& link : root["links"]) {
        if (!link.isMember("id") || !link["id"].isString()) {
            throw_parse_error(sva_json_path, "link entry missing string `id`");
        }
        links_by_id.emplace(link["id"].asString(), &link);
    }
    for (const auto& joint : root["joints"]) {
        if (!joint.isMember("id") || !joint["id"].isString()) {
            throw_parse_error(sva_json_path, "joint entry missing string `id`");
        }
        joints_by_id.emplace(joint["id"].asString(), &joint);
    }

    // For each DH joint, find the unique link whose parent is the joint.
    auto find_child_link = [&](const std::string& joint_id) -> const Json::Value& {
        const Json::Value* found = nullptr;
        for (const auto& link : root["links"]) {
            if (link.isMember("parent") && link["parent"].isString() && link["parent"].asString() == joint_id) {
                if (found != nullptr) {
                    throw_parse_error(sva_json_path, "DH joint `" + joint_id + "` has multiple child links");
                }
                found = &link;
            }
        }
        if (found == nullptr) {
            throw_parse_error(sva_json_path, "DH joint `" + joint_id + "` has no child link");
        }
        return *found;
    };

    ModelTables out{};

    // Resolve and stash the chain-root link name (link_names[0]) by walking
    // the first DH joint's child back. All shipped UR models share the same
    // root ("base_link") in practice, but we read it rather than assume.
    std::string chain_root_name;

    for (std::size_t i = 0; i < k_dh_joint_names.size(); ++i) {
        const std::string joint_id = k_dh_joint_names[i];
        const auto joint_it = joints_by_id.find(joint_id);
        if (joint_it == joints_by_id.end()) {
            throw_parse_error(sva_json_path, "missing DH joint `" + joint_id + "`");
        }
        out.limits[i] = parse_joint_limits(sva_json_path, *joint_it->second);

        const Json::Value& child = find_child_link(joint_id);
        out.link_names[i + 1] = child["id"].asString();

        // Walk the parent chain back from `child` to a root: each step looks
        // up the named parent in either map; entries are pushed in
        // child-to-root order. Termination: parent absent (no map hit) or
        // equal to "world". The link whose parent triggers termination IS
        // included.
        std::vector<const Json::Value*> chain_rev;
        std::string cur_parent = child.isMember("parent") ? child["parent"].asString() : std::string{};
        std::size_t depth = 0;
        while (!cur_parent.empty() && cur_parent != "world") {
            if (depth++ > k_max_chain_walk_depth) {
                throw_parse_error(sva_json_path, "parent chain for `" + std::string{child["id"].asString()} +
                                                     "` exceeded max walk depth (cycle?)");
            }
            const Json::Value* entry = nullptr;
            if (auto it = joints_by_id.find(cur_parent); it != joints_by_id.end()) {
                entry = it->second;
            } else if (auto it = links_by_id.find(cur_parent); it != links_by_id.end()) {
                entry = it->second;
            } else {
                throw_parse_error(sva_json_path, "parent `" + cur_parent + "` not found in links or joints");
            }
            chain_rev.push_back(entry);
            cur_parent = entry->isMember("parent") ? (*entry)["parent"].asString() : std::string{};
        }

        // Reversed-back-to-root: chain_rev is child-up; iterate in reverse to
        // multiply in root-down order.
        Eigen::Matrix4d W = Eigen::Matrix4d::Identity();
        for (auto it = chain_rev.rbegin(); it != chain_rev.rend(); ++it) {
            const Json::Value& entry = **it;
            const bool is_joint = entry.isMember("axis");  // joints carry `axis`; links don't
            if (is_joint) {
                continue;  // joints are identity at zero joint state
            }
            W = W * local_transform(sva_json_path, entry);
        }

        // The first DH joint's chain establishes link_names[0]. Subsequent
        // joints must agree (sanity check, not load-bearing).
        if (!chain_rev.empty()) {
            const Json::Value& root_entry = *chain_rev.back();
            const std::string root_name = root_entry["id"].asString();
            if (i == 0) {
                chain_root_name = root_name;
            } else if (root_name != chain_root_name) {
                throw_parse_error(sva_json_path,
                                  "DH joint `" + joint_id + "` chain root `" + root_name +
                                      "` disagrees with joint 0's chain root `" + chain_root_name + "`");
            }
        }

        out.geometries[i + 1] = parse_geometry_world(sva_json_path, child, W);
    }

    if (chain_root_name.empty()) {
        throw_parse_error(sva_json_path, "could not determine chain root link name");
    }
    out.link_names[0] = chain_root_name;

    // base_link's geometry lives in its parent (world) frame at zero joints,
    // so W is identity. ur5e/ur7e ship a real base capsule here; ur20 ships a
    // 1mm placeholder sphere; ur3e ships a box which `parse_geometry_world`
    // intentionally skips.
    const auto base_it = links_by_id.find(chain_root_name);
    if (base_it == links_by_id.end()) {
        throw_parse_error(sva_json_path, "chain root link `" + chain_root_name + "` not found in links map");
    }
    out.geometries[0] = parse_geometry_world(sva_json_path, *base_it->second, Eigen::Matrix4d::Identity());

    return out;
}

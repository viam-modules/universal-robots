#include "ur_arm_model.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <json/json.h>

#include "rust_utils.hpp"
#include "ur_arm.hpp"
#include "utils.hpp"  // degrees_to_radians / radians_to_degrees

namespace {

constexpr std::size_t k_num_dh_joints = 6;
constexpr std::size_t k_num_emitted_links = k_num_dh_joints + 1;
constexpr double k_m_to_mm = 1000.0;

// Build a 4x4 homogeneous transform for a link's static pose:
//   Rz(theta) * T(a, 0, d) * Rx(alpha)
// All inputs in millimeters/radians. Output is the matrix that takes a child-frame
// point to its parent-frame coordinates.
Eigen::Matrix4d dh_link_pose_matrix(double a_mm, double d_mm, double alpha_rad, double theta_rad) {
    const double ca = std::cos(alpha_rad);
    const double sa = std::sin(alpha_rad);
    const double ct = std::cos(theta_rad);
    const double st = std::sin(theta_rad);

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 0) = ct;
    M(0, 1) = -st * ca;
    M(0, 2) = st * sa;
    M(1, 0) = st;
    M(1, 1) = ct * ca;
    M(1, 2) = -ct * sa;
    M(2, 0) = 0.0;
    M(2, 1) = sa;
    M(2, 2) = ca;
    M(0, 3) = a_mm * ct;
    M(1, 3) = a_mm * st;
    M(2, 3) = d_mm;
    return M;
}

// The emitted chain has 7 link-local slots (1 base + 6 articulated)
// but DH only describes the 6 joints. Row 0's `d` is the
// base->shoulder z-offset, which physically belongs to the base mount,
// so we emit it alone at slot 0; slot 1 takes the rest of row 0 with
// `d` zeroed so d[0] isn't applied twice. Slots 2..6 each consume one
// full DH row.
Eigen::Matrix4d dh_local_for_slot(const DHParams& dh, std::size_t i) {
    if (i == 0) {
        return dh_link_pose_matrix(0.0, dh.d[0] * k_m_to_mm, 0.0, 0.0);
    }
    const std::size_t s = i - 1;
    const double d_mm = (s == 0) ? 0.0 : (dh.d[s] * k_m_to_mm);
    return dh_link_pose_matrix(dh.a[s] * k_m_to_mm, d_mm, dh.alpha[s], dh.theta[s]);
}

// Guards against a malformed chain creating a runaway walk (cycle, or a
// pathologically long sequence of intermediate static links).
constexpr std::size_t k_max_chain_walk_steps = 64;

[[noreturn]] void throw_parse_error(const std::filesystem::path& path, const std::string& msg) {
    throw std::invalid_argument("UrArmModel::load_kinematics: " + path.string() + ": " + msg);
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
    const auto comps = rust_utils::quaternion_components(q);
    return Eigen::Quaterniond{comps[0], comps[1], comps[2], comps[3]};
}

// Build a viam-cpp-sdk pose from an Eigen translation+quaternion pair by
// converting the quaternion to an orientation-vector via rust-utils. The
// pose stores its translation in mm (whatever unit the caller supplies)
// and its `theta` field in degrees.
viam::sdk::pose pose_from_translation_quaternion(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    const auto rust_q = rust_utils::make_quaternion(q.w(), q.x(), q.y(), q.z());
    const auto rust_ov = rust_utils::ov_from_quaternion(rust_q.get());
    const auto comps = rust_utils::ov_components(rust_ov.get());
    return viam::sdk::pose{
        viam::sdk::coordinates{t.x(), t.y(), t.z()},
        viam::sdk::pose_orientation{comps[0], comps[1], comps[2]},
        radians_to_degrees(comps[3]),
    };
}

// Inverse of `pose_from_translation_quaternion`: pull a viam-cpp-sdk pose
// back into an Eigen translation + quaternion via rust-utils.
std::pair<Eigen::Vector3d, Eigen::Quaterniond> translation_quaternion_from_pose(const viam::sdk::pose& p) {
    const auto rust_ov =
        rust_utils::make_orientation_vector(p.orientation.o_x, p.orientation.o_y, p.orientation.o_z, degrees_to_radians(p.theta));
    const auto rust_q = rust_utils::quaternion_from_ov(rust_ov.get());
    return {Eigen::Vector3d{p.coordinates.x, p.coordinates.y, p.coordinates.z}, eigen_from_rust_quaternion(rust_q.get())};
}

// Delegate the `ov_degrees` -> quaternion conversion to rust-utils, which
// is RDK's canonical implementation; see
// https://github.com/viamrobotics/rdk/blob/main/spatialmath/orientationVector.go
Eigen::Quaterniond ov_degrees_to_quaternion(double x, double y, double z, double th_deg) {
    const auto ov = rust_utils::make_orientation_vector(x, y, z, degrees_to_radians(th_deg));
    const auto q = rust_utils::quaternion_from_ov(ov.get());
    return eigen_from_rust_quaternion(q.get());
}

// Delegate the `euler_angles` -> quaternion conversion to rust-utils
// (Z-Y'-X" intrinsic Tait-Bryan; Z=yaw, Y=pitch, X=roll).
Eigen::Quaterniond euler_angles_to_quaternion(double roll, double pitch, double yaw) {
    const auto q = rust_utils::quaternion_from_euler(roll, pitch, yaw);
    return eigen_from_rust_quaternion(q.get());
}

// Read an `orientation` sub-object, handling the three types the shipped
// JSONs use. Missing or null block -> identity.
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
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;
    return transform;
}

// The local-frame transform of any JSON node with optional `translation` and
// `orientation` sub-objects (chain links, joints, or geometry blocks):
// T(translation) * R(orientation), translation in millimeters.
Eigen::Matrix4d local_transform(const std::filesystem::path& path, const Json::Value& node) {
    return to_homogeneous(parse_orientation(path, node), parse_translation(node));
}

// `intermediate_to_emitted_parent` folds in any static-only links
// collapsed between this entry and the previous emitted slot (identity
// otherwise). Returns nullopt for absent geometry blocks or `box`.
std::optional<Geometry> parse_geometry_in_emitted_parent_frame(const std::filesystem::path& path,
                                                               const Json::Value& link_entry,
                                                               const Eigen::Matrix4d& intermediate_to_emitted_parent) {
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

    // g's local pose is in the static-file link's parent frame. To express
    // it in the emitted link's parent frame, pre-multiply by the cumulative
    // transform from the emitted parent to the static parent.
    const Eigen::Matrix4d emitted_local = intermediate_to_emitted_parent * local_transform(path, g);
    const Eigen::Vector3d t = emitted_local.block<3, 1>(0, 3);
    const Eigen::Quaterniond q = Eigen::Quaterniond{emitted_local.block<3, 3>(0, 0)}.normalized();
    const viam::sdk::pose pose = pose_from_translation_quaternion(t, q);

    if (type == "sphere") {
        return Geometry{pose, viam::sdk::sphere{r}};
    }

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

Json::Value translation_json(const Eigen::Vector3d& t) {
    Json::Value out(Json::objectValue);
    out["x"] = t.x();
    out["y"] = t.y();
    out["z"] = t.z();
    return out;
}

Json::Value quaternion_json(const Eigen::Quaterniond& q) {
    Json::Value out(Json::objectValue);
    out["type"] = "quaternion";
    Json::Value val(Json::objectValue);
    val["W"] = q.w();
    val["X"] = q.x();
    val["Y"] = q.y();
    val["Z"] = q.z();
    out["value"] = val;
    return out;
}

Json::Value geometry_to_json(const Geometry& geom) {
    Json::Value json(Json::objectValue);

    json["translation"] = translation_json({geom.pose.coordinates.x, geom.pose.coordinates.y, geom.pose.coordinates.z});

    std::visit(
        [&](const auto& shape) {
            using S = std::decay_t<decltype(shape)>;
            json["r"] = shape.radius;
            if constexpr (std::is_same_v<S, viam::sdk::capsule>) {
                json["type"] = "capsule";
                json["l"] = shape.length;
                Json::Value orient(Json::objectValue);
                orient["type"] = "ov_degrees";
                Json::Value val(Json::objectValue);
                val["x"] = geom.pose.orientation.o_x;
                val["y"] = geom.pose.orientation.o_y;
                val["z"] = geom.pose.orientation.o_z;
                val["th"] = geom.pose.theta;
                orient["value"] = val;
                json["orientation"] = orient;
            } else {
                // Spheres carry no orientation; emit dimensions only.
                json["type"] = "sphere";
            }
        },
        geom.shape);

    return json;
}

// Apply a 4x4 correction matrix to a viam-cpp-sdk pose, returning a new
// pose. Internally round-trips through a quaternion (via rust-utils) to
// compose with the matrix.
viam::sdk::pose apply_correction_to_pose(const viam::sdk::pose& p, const Eigen::Matrix4d& correction) {
    const auto [t_in, q_in] = translation_quaternion_from_pose(p);

    Eigen::Matrix4d input_pose = Eigen::Matrix4d::Identity();
    input_pose.block<3, 3>(0, 0) = q_in.toRotationMatrix();
    input_pose.block<3, 1>(0, 3) = t_in;

    const Eigen::Matrix4d output_pose = correction * input_pose;
    const Eigen::Vector3d t_out = output_pose.block<3, 1>(0, 3);
    const Eigen::Quaterniond q_out = Eigen::Quaterniond{output_pose.block<3, 3>(0, 0)}.normalized();

    return pose_from_translation_quaternion(t_out, q_out);
}

}  // namespace

const std::vector<UrModelDescriptor>& UrModelDescriptor::all() {
    static const std::vector<UrModelDescriptor> table = [] {
        std::vector<UrModelDescriptor> t;
        // ur3e/ur7e have no shipped meshes; `get_3d_models` returns an
        // empty map for them.
        t.push_back({"ur3e", "ur3", {}});
        t.push_back(
            {"ur5e", "ur5", {"base_link", "ee_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}});
        t.push_back({"ur7e", "ur5", {}});
        t.push_back({"ur20",
                     "ur20",
                     {"base_link", "wrist_3_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}});
        return t;
    }();
    return table;
}

const UrModelDescriptor& UrModelDescriptor::for_sdk_name(const std::string& sdk_name) {
    for (const auto& d : all()) {
        if (d.sdk_name == sdk_name) {
            return d;
        }
    }
    throw std::invalid_argument("UrModelDescriptor::for_sdk_name: unknown sdk model name `" + sdk_name + "`");
}

UrArmModel::UrArmModel(viam::sdk::Model sdk_model, const UrModelDescriptor& descriptor)
    : sdk_model_(std::move(sdk_model)), descriptor_(&descriptor) {}

UrArmModel::UrArmModel(viam::sdk::Model sdk_model)
    // sdk_model_ is initialized in declaration order before descriptor_,
    // so reading the moved-into sdk_model_ for the descriptor lookup here
    // is well-defined (a delegating-constructor form risks reading from
    // the moved-from parameter, depending on argument evaluation order).
    : sdk_model_(std::move(sdk_model)), descriptor_(&UrModelDescriptor::for_sdk_name(sdk_model_.model_name())) {}

UrArmModel UrArmModel::from_sdk_name(const std::string& sdk_name) {
    return UrArmModel{URArm::model(sdk_name), UrModelDescriptor::for_sdk_name(sdk_name)};
}

UrArmModel::Kinematics::Kinematics(UrArmModel m) : model(std::move(m)), limits{}, link_locals{}, geometries{}, link_names{} {
    for (auto& mat : link_locals) {
        mat = Eigen::Matrix4d::Identity();
    }
}

UrArmModel::Kinematics UrArmModel::Kinematics::apply_calibration(const DHParams& dh) const {
    Kinematics out = *this;

    std::array<Eigen::Matrix4d, 7> new_link_locals;
    for (std::size_t i = 0; i < 7; ++i) {
        new_link_locals[i] = dh_local_for_slot(dh, i);
    }

    // For each present geometry, re-express its pose in the new emitted
    // parent frame: G_new = inv(W_new[i]) * W_old[i] * G_old. Walk both
    // cumulative chains in lockstep so the correction at slot i uses the
    // parent poses, i.e. the product of link_locals 0..i-1) on both sides.
    Eigen::Matrix4d cum_old = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d cum_new = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < 7; ++i) {
        if (out.geometries[i].has_value()) {
            const Eigen::Matrix4d correction = cum_new.inverse() * cum_old;
            out.geometries[i] = Geometry{apply_correction_to_pose(out.geometries[i]->pose, correction), out.geometries[i]->shape};
        }
        cum_old = cum_old * link_locals[i];
        cum_new = cum_new * new_link_locals[i];
    }

    out.link_locals = new_link_locals;
    return out;
}

Eigen::Matrix4d UrArmModel::Kinematics::parent_pose_at(std::size_t i) const {
    Eigen::Matrix4d acc = Eigen::Matrix4d::Identity();
    for (std::size_t k = 0; k < i; ++k) {
        acc = acc * link_locals[k];
    }
    return acc;
}

UrArmModel::Kinematics UrArmModel::load_kinematics(const std::filesystem::path& sva_json_path) const {
    std::ifstream in(sva_json_path);
    if (!in) {
        throw_parse_error(sva_json_path, "unable to open file");
    }

    // IIFE so `root` can be `const` after parsing -- the rest of this
    // function is read-only, and `child_of` below holds raw pointers into
    // `root` whose validity depends on `root` not being mutated.
    const auto root = [&]() {
        Json::Value root;
        const Json::CharReaderBuilder reader_builder;
        std::string errs;
        if (!Json::parseFromStream(reader_builder, in, &root, &errs)) {
            throw_parse_error(sva_json_path, "JSON parse failure: " + errs);
        }
        return root;
    }();

    if (!root.isMember("kinematic_param_type") || root["kinematic_param_type"].asString() != "SVA") {
        throw_parse_error(sva_json_path, "expected `kinematic_param_type: \"SVA\"`");
    }
    if (!root.isMember("links") || !root["links"].isArray()) {
        throw_parse_error(sva_json_path, "missing `links` array");
    }
    if (!root.isMember("joints") || !root["joints"].isArray()) {
        throw_parse_error(sva_json_path, "missing `joints` array");
    }

    // Build `parent_id -> child_entity` so we can walk the chain forward
    // from "world" without ever looking up entries by name. Per the shipped
    // JSONs each parent has at most one child (no branching); we enforce
    // that here.
    std::unordered_map<std::string, const Json::Value*> child_of;
    auto index_entry = [&](const Json::Value& entry, const char* kind) {
        if (!entry.isMember("id") || !entry["id"].isString()) {
            throw_parse_error(sva_json_path, std::string{kind} + " entry missing string `id`");
        }
        // A missing or null `parent` field is treated as "world".
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

    // `axis` is on joints; links don't have it. Used to tell joints and links apart.
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

    Kinematics out{*this};

    // Slot 0: base_link. Its local is just its own static transform (no
    // intermediates above it); its geometry's parent frame is world, so the intermediate transform is identity.
    out.link_names[0] = chain_root["id"].asString();
    out.link_locals[0] = local_transform(sva_json_path, chain_root);
    out.geometries[0] = parse_geometry_in_emitted_parent_frame(sva_json_path, chain_root, Eigen::Matrix4d::Identity());

    // Walk forward, accumulating any intermediate static-link transforms
    // into `pending_intermediate`. When we reach a DH joint, its child link
    // becomes the next emitted slot: that slot's link_local absorbs the
    // pending intermediates, and its geometry (which sits in the joint
    // frame in static) is re-expressed in the previous emitted link's
    // frame by pre-multiplication with `pending_intermediate` (joints
    // contribute identity at zero state).
    Eigen::Matrix4d pending_intermediate = Eigen::Matrix4d::Identity();
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
            pending_intermediate = pending_intermediate * local_transform(sva_json_path, *next);
            cursor = next;
            continue;
        }

        // `next` is a DH joint. Record its limits, then advance to its
        // child link (the DH-frame-i link). The joint itself contributes
        // identity to the chain pose.
        out.limits[dh_count] = parse_joint_limits(sva_json_path, *next);
        const Json::Value* dh_link = next_in_chain(*next);
        if (dh_link == nullptr) {
            throw_parse_error(sva_json_path, "DH joint `" + (*next)["id"].asString() + "` has no child link");
        }
        if (is_joint(*dh_link)) {
            throw_parse_error(sva_json_path, "DH joint `" + (*next)["id"].asString() + "` child must be a link, got a joint");
        }

        const std::size_t slot = dh_count + 1;
        out.link_names[slot] = (*dh_link)["id"].asString();
        out.link_locals[slot] = pending_intermediate * local_transform(sva_json_path, *dh_link);
        out.geometries[slot] = parse_geometry_in_emitted_parent_frame(sva_json_path, *dh_link, pending_intermediate);

        pending_intermediate = Eigen::Matrix4d::Identity();
        cursor = dh_link;
        dh_count++;
    }

    return out;
}

std::string UrArmModel::Kinematics::to_sva_json() const {
    const std::string& model_name = model.sdk_name();

    Json::Value root(Json::objectValue);
    root["name"] = model_name;
    root["kinematic_param_type"] = "SVA";

    // Every DH joint rotates about its own local z-axis by definition of
    // the Denavit-Hartenberg parameterization; the link transforms have
    // already rotated the parent frame so its z points along the physical
    // joint axis. Build the JSON axis value once and reuse it.
    Json::Value dh_z_axis(Json::objectValue);
    dh_z_axis["x"] = 0.0;
    dh_z_axis["y"] = 0.0;
    dh_z_axis["z"] = 1.0;

    Json::Value joints(Json::arrayValue);
    Json::Value links(Json::arrayValue);

    for (std::size_t i = 0; i < k_num_emitted_links; ++i) {
        if (i > 0) {
            Json::Value joint(Json::objectValue);
            joint["id"] = model_name + "_q_" + std::to_string(i - 1);
            joint["type"] = "revolute";
            joint["parent"] = link_names[i - 1];
            joint["axis"] = dh_z_axis;
            joint["min"] = limits[i - 1].min_deg;
            joint["max"] = limits[i - 1].max_deg;
            joints.append(joint);
        }

        Json::Value link(Json::objectValue);
        link["id"] = link_names[i];
        link["parent"] = (i == 0) ? std::string{"world"} : (model_name + "_q_" + std::to_string(i - 1));
        link["translation"] = translation_json(link_locals[i].block<3, 1>(0, 3));
        link["orientation"] = quaternion_json(Eigen::Quaterniond{link_locals[i].block<3, 3>(0, 0)});
        if (geometries[i].has_value()) {
            link["geometry"] = geometry_to_json(*geometries[i]);
        }
        links.append(link);
    }
    root["joints"] = joints;
    root["links"] = links;

    Json::StreamWriterBuilder writer_builder;
    writer_builder["indentation"] = "  ";
    return Json::writeString(writer_builder, root);
}

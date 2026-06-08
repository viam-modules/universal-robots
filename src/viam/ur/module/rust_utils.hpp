#pragma once

#include <memory>

#include "rust_utils.h"

namespace rust_utils {

using unique_quaternion = std::unique_ptr<void, decltype(&free_quaternion_memory)>;
using unique_orientation_vector = std::unique_ptr<void, decltype(&free_orientation_vector_memory)>;
using unique_quaternion_components = std::unique_ptr<double[], decltype(&free_quaternion_components)>;
using unique_orientation_components = std::unique_ptr<double[], decltype(&free_orientation_vector_components)>;
using unique_axis_angles = std::unique_ptr<double[], decltype(&free_axis_angles_memory)>;

// Constructors. Wrap a freshly-returned handle in its owning unique_ptr.
inline unique_quaternion make_quaternion(double w, double x, double y, double z) {
    return {::new_quaternion(w, x, y, z), &::free_quaternion_memory};
}

inline unique_orientation_vector make_orientation_vector(double ox, double oy, double oz, double theta_rad) {
    return {::new_orientation_vector(ox, oy, oz, theta_rad), &::free_orientation_vector_memory};
}

// Conversions.
inline unique_orientation_vector ov_from_quaternion(void* q) {
    return {::orientation_vector_from_quaternion(q), &::free_orientation_vector_memory};
}

inline unique_quaternion quaternion_from_ov(void* ov) {
    return {::quaternion_from_orientation_vector(ov), &::free_quaternion_memory};
}

inline unique_quaternion quaternion_from_euler(double roll, double pitch, double yaw) {
    return {::quaternion_from_euler_angles(roll, pitch, yaw), &::free_quaternion_memory};
}

inline unique_quaternion quaternion_from_axis_angle(double x, double y, double z, double theta_rad) {
    return {::quaternion_from_axis_angle(x, y, z, theta_rad), &::free_quaternion_memory};
}

// Component readers. Each returns an owning unique_ptr to a 4-double
// array allocated by rust-utils.
inline unique_orientation_components ov_components(void* ov) {
    return {::orientation_vector_get_components(ov), &::free_orientation_vector_components};
}

inline unique_quaternion_components quaternion_components(void* q) {
    return {::quaternion_get_components(q), &::free_quaternion_components};
}

inline unique_axis_angles axis_angle_from_quaternion(void* q) {
    return {::axis_angle_from_quaternion(q), &::free_axis_angles_memory};
}

}  // namespace rust_utils

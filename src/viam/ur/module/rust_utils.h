#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Orientation vectors (RDK "OV" representation; theta in radians).
void* new_orientation_vector(double ox, double oy, double oz, double theta);
void free_orientation_vector_memory(void* ov);
double* orientation_vector_get_components(void* ov);
void free_orientation_vector_components(double* ptr);

// Quaternions (W, X, Y, Z order).
void* new_quaternion(double real, double i, double j, double k);
void free_quaternion_memory(void* q);
double* quaternion_get_components(void* q);
void free_quaternion_components(double* ptr);

// Conversions.
void* orientation_vector_from_quaternion(void* q);
void* quaternion_from_orientation_vector(void* ov);
void* quaternion_from_euler_angles(double roll, double pitch, double yaw);
void* quaternion_from_axis_angle(double x, double y, double z, double theta);

// Axis-angle (returned as a 4-component array: x, y, z, theta).
double* axis_angle_from_quaternion(void* q);
void free_axis_angles_memory(void* aa);

#ifdef __cplusplus
}
#endif

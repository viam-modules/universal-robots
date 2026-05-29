#pragma once

#include <ur_client_library/types.h>

// Per-joint DH parameters for a 6-DOF UR arm, as reported by the controller
// via `urcl::primary_interface::KinematicsInfo`. Each array holds one entry
// per joint; units match what urcl reports: meters for `a` and `d`, radians
// for `alpha` and `theta`. The reported `theta` is a fixed per-joint
// calibration offset; it does not change as the joint moves.
struct DHParams {
    urcl::vector6d_t a;
    urcl::vector6d_t d;
    urcl::vector6d_t alpha;
    urcl::vector6d_t theta;
};

#pragma once

#include <cmath>
#include <compare>
#include <ostream>

#include <viam/trajex/types/arc_acceleration.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_velocity.hpp>
#include <viam/trajex/types/epsilon.hpp>

namespace viam::trajex::totg {

// Minimal strong type for phase plane slope (ds_dot/ds or s_ddot/s_dot).
//
// Represents slopes in the s-s_dot phase plane with dimensions [1/s].
// Physically: change in velocity per unit distance, or trajectory slope.
//
// Note: This is private to trajectory.cpp - not part of public API.
class phase_plane_slope {
   public:
    // Constructs from raw double value.
    explicit constexpr phase_plane_slope(double v) noexcept : value_{v} {}

    // Constructs from epsilon tolerance.
    explicit constexpr phase_plane_slope(class epsilon epsilon) noexcept : phase_plane_slope{static_cast<double>(epsilon)} {}

    // Converts to double explicitly.
    explicit constexpr operator double() const noexcept {
        return value_;
    }

    // Three-way comparison operator.
    constexpr auto operator<=>(const phase_plane_slope&) const = default;

    // Compound subtraction (used for slope comparisons).
    constexpr phase_plane_slope& operator-=(phase_plane_slope other) noexcept {
        value_ -= static_cast<double>(other);
        return *this;
    }

   private:
    double value_;
};

// Subtracts phase plane slopes (used for comparing slopes in exit conditions).
constexpr phase_plane_slope operator-(phase_plane_slope lhs, phase_plane_slope rhs) noexcept {
    return lhs -= rhs;
}

// Streams phase plane slope for debugging and test output.
inline std::ostream& operator<<(std::ostream& os, phase_plane_slope slope) {
    return os << static_cast<double>(slope);
}

// Cross-type operations: constructing phase_plane_slope

// Divides arc velocity by arc length to get phase plane curve slope.
// Dimensional analysis: (ds/dt) / ds = d(ds/dt)/ds = [1/s]
// Physical meaning: change in velocity per unit distance (curve slope)
constexpr phase_plane_slope operator/(arc_velocity vel, arc_length len) noexcept {
    return phase_plane_slope{static_cast<double>(vel) / static_cast<double>(len)};
}

// Divides arc acceleration by arc velocity to get trajectory slope.
// Dimensional analysis: (d^2s/dt^2) / (ds/dt) = [1/s]
// Physical meaning: trajectory's instantaneous slope in phase plane (s_ddot/s_dot)
constexpr phase_plane_slope operator/(arc_acceleration acc, arc_velocity vel) noexcept {
    return phase_plane_slope{static_cast<double>(acc) / static_cast<double>(vel)};
}

// Cross-type operations: using phase_plane_slope

// Multiplies phase plane slope by arc velocity to get curve-following acceleration.
// Dimensional analysis: [1/s] * (ds/dt) = d^2s/dt^2
// Physical meaning: acceleration needed to follow velocity curve tangentially (trajectory.cpp:1211)
constexpr arc_acceleration operator*(phase_plane_slope slope, arc_velocity vel) noexcept {
    return arc_acceleration{static_cast<double>(slope) * static_cast<double>(vel)};
}

// Multiplies arc velocity by phase plane slope (commutative).
constexpr arc_acceleration operator*(arc_velocity vel, phase_plane_slope slope) noexcept {
    return slope * vel;
}

}  // namespace viam::trajex::totg


// namespace viam::trajex {

// template <>
// class epsilon::wrapper<totg::phase_plane_slope>;

// }  // namespace viam::trajex

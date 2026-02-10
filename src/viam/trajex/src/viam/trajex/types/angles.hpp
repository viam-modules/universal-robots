#pragma once

#include <numbers>

namespace viam::trajex {

///
/// Converts degrees to radians.
///
/// Uses perfect forwarding to work with scalars, arrays, and expression templates.
///
/// @param degrees Angle in degrees
/// @return Angle in radians
///
template <typename T>
[[nodiscard]] constexpr decltype(auto) degrees_to_radians(T&& degrees) {
    return std::forward<T>(degrees) * (std::numbers::pi / 180.0);
}

///
/// Converts radians to degrees.
///
/// Uses perfect forwarding to work with scalars, arrays, and expression templates.
///
/// @param radians Angle in radians
/// @return Angle in degrees
///
template <typename T>
[[nodiscard]] constexpr decltype(auto) radians_to_degrees(T&& radians) {
    return std::forward<T>(radians) * (180.0 / std::numbers::pi);
}

}  // namespace viam::trajex

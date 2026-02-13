#pragma once

#include <chrono>

#include <viam/trajex/types/arc_acceleration.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_velocity.hpp>

namespace viam::trajex {

///
/// Multiplies arc velocity by time duration to get arc length.
///
/// Dimensional analysis: (ds/dt) * dt = ds
///
/// Note: Intentionally unidirectional (time on RHS) to encourage idiomatic usage
/// matching physics notation. Use `velocity * duration`, not `duration * velocity`.
///
/// @param vel Arc velocity
/// @param dt Time duration
/// @return Arc length traveled
///
constexpr arc_length operator*(arc_velocity vel, std::chrono::duration<double> dt) noexcept {
    return arc_length{static_cast<double>(vel) * dt.count()};
}

///
/// Divides arc velocity by time duration to get arc acceleration.
///
/// Dimensional analysis: (ds/dt) / dt = d^2s/dt^2
///
/// Note: Intentionally unidirectional (time on RHS) to encourage idiomatic usage
/// matching physics notation. Use `velocity / duration`, not `duration / velocity`.
///
/// @param vel Arc velocity
/// @param dt Time duration
/// @return Arc acceleration
///
constexpr arc_acceleration operator/(arc_velocity vel, std::chrono::duration<double> dt) noexcept {
    return arc_acceleration{static_cast<double>(vel) / dt.count()};
}

///
/// Multiplies arc acceleration by time duration to get arc velocity.
///
/// Dimensional analysis: (d^2s/dt^2) * dt = ds/dt
///
/// Note: Intentionally unidirectional (time on RHS) to encourage idiomatic usage
/// matching physics notation. Use `acceleration * duration`, not `duration * acceleration`.
///
/// @param acc Arc acceleration
/// @param dt Time duration
/// @return Arc velocity
///
constexpr arc_velocity operator*(arc_acceleration acc, std::chrono::duration<double> dt) noexcept {
    return arc_velocity{static_cast<double>(acc) * dt.count()};
}

///
/// Divides arc length by time duration to get arc velocity.
///
/// Dimensional analysis: ds / dt = ds/dt
///
/// Note: Intentionally unidirectional (time on RHS) to encourage idiomatic usage
/// matching physics notation. Use `length / duration`, not `duration / length`.
///
/// @param len Arc length
/// @param dt Time duration
/// @return Arc velocity
///
constexpr arc_velocity operator/(arc_length len, std::chrono::duration<double> dt) noexcept {
    return arc_velocity{static_cast<double>(len) / dt.count()};
}

///
/// Divides arc velocity by arc acceleration to get time duration.
///
/// Dimensional analysis: (ds/dt) / (d^2s/dt^2) = dt
///
/// @param vel Arc velocity
/// @param acc Arc acceleration
/// @return Time duration
///
constexpr std::chrono::duration<double> operator/(arc_velocity vel, arc_acceleration acc) noexcept {
    return std::chrono::duration<double>{static_cast<double>(vel) / static_cast<double>(acc)};
}

///
/// Divides arc length by arc velocity to get time duration.
///
/// Dimensional analysis: ds / (ds/dt) = dt
///
/// @param len Arc length
/// @param vel Arc velocity
/// @return Time duration
///
constexpr std::chrono::duration<double> operator/(arc_length len, arc_velocity vel) noexcept {
    return std::chrono::duration<double>{static_cast<double>(len) / static_cast<double>(vel)};
}

}  // namespace viam::trajex

#pragma once

#include <compare>

namespace viam::trajex {

/// Strong type for arc length in configuration space
///
/// Provides type safety to prevent mixing arc length with other double quantities
/// (time, angles, etc.). Supports explicit conversion and arithmetic operations.
struct arc_length {
    double value;

    /// Construct from raw double value
    explicit constexpr arc_length(double v) noexcept : value{v} {}

    /// Explicit conversion to double
    explicit constexpr operator double() const noexcept {
        return value;
    }

    // clang-format off
    /// Three-way comparison
    constexpr auto operator<=>(const arc_length&) const = default;
    // clang-format on

    /// Addition of arc lengths
    constexpr arc_length operator+(arc_length other) const noexcept {
        return arc_length{value + other.value};
    }

    /// Subtraction of arc lengths
    constexpr arc_length operator-(arc_length other) const noexcept {
        return arc_length{value - other.value};
    }

    /// Scalar multiplication (arc_length * scalar)
    constexpr arc_length operator*(double scalar) const noexcept {
        return arc_length{value * scalar};
    }

    /// Scalar division
    constexpr arc_length operator/(double scalar) const noexcept {
        return arc_length{value / scalar};
    }

    /// Division of arc lengths yields dimensionless ratio
    constexpr double operator/(arc_length other) const noexcept {
        return value / other.value;
    }

    /// Compound addition
    constexpr arc_length& operator+=(arc_length other) noexcept {
        value += other.value;
        return *this;
    }

    /// Compound subtraction
    constexpr arc_length& operator-=(arc_length other) noexcept {
        value -= other.value;
        return *this;
    }

    /// Compound scalar multiplication
    constexpr arc_length& operator*=(double scalar) noexcept {
        value *= scalar;
        return *this;
    }

    /// Compound scalar division
    constexpr arc_length& operator/=(double scalar) noexcept {
        value /= scalar;
        return *this;
    }

    /// Scalar multiplication (scalar * arc_length)
    friend constexpr arc_length operator*(double scalar, arc_length len) noexcept {
        return len * scalar;
    }
};

}  // namespace viam::trajex

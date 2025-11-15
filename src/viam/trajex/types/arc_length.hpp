#pragma once

#include <compare>
#include <ostream>

namespace viam::trajex {

///
/// Strong type for arc length in configuration space.
///
/// Provides type safety to prevent mixing arc length with other double quantities
/// (time, angles, etc.). Supports explicit conversion and arithmetic operations.
///
struct arc_length {
    ///
    /// Arc length value.
    ///
    double value;

    ///
    /// Constructs from raw double value.
    ///
    /// @param v Arc length value
    ///
    explicit constexpr arc_length(double v) noexcept : value{v} {}

    ///
    /// Converts to double explicitly.
    ///
    /// @return Arc length as double
    ///
    explicit constexpr operator double() const noexcept {
        return value;
    }

    // clang-format off
    ///
    /// Three-way comparison operator.
    ///
    constexpr auto operator<=>(const arc_length&) const = default;
    // clang-format on

    ///
    /// Adds arc lengths.
    ///
    /// @param other Arc length to add
    /// @return Sum of arc lengths
    ///
    constexpr arc_length operator+(arc_length other) const noexcept {
        return arc_length{value + other.value};
    }

    ///
    /// Subtracts arc lengths.
    ///
    /// @param other Arc length to subtract
    /// @return Difference of arc lengths
    ///
    constexpr arc_length operator-(arc_length other) const noexcept {
        return arc_length{value - other.value};
    }

    ///
    /// Multiplies arc length by scalar.
    ///
    /// @param scalar Scalar multiplier
    /// @return Scaled arc length
    ///
    constexpr arc_length operator*(double scalar) const noexcept {
        return arc_length{value * scalar};
    }

    ///
    /// Divides arc length by scalar.
    ///
    /// @param scalar Scalar divisor
    /// @return Scaled arc length
    ///
    constexpr arc_length operator/(double scalar) const noexcept {
        return arc_length{value / scalar};
    }

    ///
    /// Divides arc length by arc length (yields dimensionless ratio).
    ///
    /// @param other Arc length divisor
    /// @return Dimensionless ratio
    ///
    constexpr double operator/(arc_length other) const noexcept {
        return value / other.value;
    }

    ///
    /// Compound addition.
    ///
    /// @param other Arc length to add
    /// @return Reference to this
    ///
    constexpr arc_length& operator+=(arc_length other) noexcept {
        value += other.value;
        return *this;
    }

    ///
    /// Compound subtraction.
    ///
    /// @param other Arc length to subtract
    /// @return Reference to this
    ///
    constexpr arc_length& operator-=(arc_length other) noexcept {
        value -= other.value;
        return *this;
    }

    ///
    /// Compound scalar multiplication.
    ///
    /// @param scalar Scalar multiplier
    /// @return Reference to this
    ///
    constexpr arc_length& operator*=(double scalar) noexcept {
        value *= scalar;
        return *this;
    }

    ///
    /// Compound scalar division.
    ///
    /// @param scalar Scalar divisor
    /// @return Reference to this
    ///
    constexpr arc_length& operator/=(double scalar) noexcept {
        value /= scalar;
        return *this;
    }

    ///
    /// Multiplies scalar by arc length (commutative).
    ///
    /// @param scalar Scalar multiplier
    /// @param len Arc length
    /// @return Scaled arc length
    ///
    friend constexpr arc_length operator*(double scalar, arc_length len) noexcept {
        return len * scalar;
    }

    ///
    /// Streams arc length for debugging and test output.
    ///
    /// @param os Output stream
    /// @param len Arc length to output
    /// @return Reference to output stream
    ///
    friend std::ostream& operator<<(std::ostream& os, arc_length len) {
        return os << len.value;
    }
};

}  // namespace viam::trajex

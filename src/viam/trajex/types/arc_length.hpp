#pragma once

#include <cmath>
#include <compare>
#include <ostream>

#include <viam/trajex/types/epsilon.hpp>

namespace viam::trajex {

///
/// Strong type for arc length in configuration space.
///
/// Provides type safety to prevent mixing arc length with other double quantities
/// (time, angles, etc.). Supports explicit conversion and arithmetic operations.
///
class arc_length {
   public:
    ///
    /// Constructs from raw double value.
    ///
    /// @param v Arc length value
    ///
    explicit constexpr arc_length(double v) noexcept : value_{v} {}

    ///
    /// Constructs from epsilon tolerance.
    ///
    /// Allows using epsilon as a dimensional quantity for numerical exploration.
    ///
    /// @param eps Epsilon tolerance value
    ///
    explicit constexpr arc_length(class epsilon epsilon) noexcept : arc_length{static_cast<double>(epsilon)} {}

    ///
    /// Converts to double explicitly.
    ///
    /// @return Arc length as double
    ///
    explicit constexpr operator double() const noexcept {
        return value_;
    }

    // clang-format off
    ///
    /// Three-way comparison operator.
    ///
    constexpr auto operator<=>(const arc_length&) const = default;
    // clang-format on

    ///
    /// Compound addition.
    ///
    /// @param other Arc length to add
    /// @return Reference to this
    ///
    constexpr arc_length& operator+=(arc_length other) noexcept {
        value_ += static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound subtraction.
    ///
    /// @param other Arc length to subtract
    /// @return Reference to this
    ///
    constexpr arc_length& operator-=(arc_length other) noexcept {
        value_ -= static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound scalar multiplication.
    ///
    /// @param scalar Scalar multiplier
    /// @return Reference to this
    ///
    constexpr arc_length& operator*=(double scalar) noexcept {
        value_ *= scalar;
        return *this;
    }

    ///
    /// Compound scalar division.
    ///
    /// @param scalar Scalar divisor
    /// @return Reference to this
    ///
    constexpr arc_length& operator/=(double scalar) noexcept {
        value_ /= scalar;
        return *this;
    }

    ///
    /// Unary negation.
    ///
    /// @return Negated arc length
    ///
    constexpr arc_length operator-() const noexcept {
        return arc_length{-value_};
    }

   private:
    double value_;
};

///
/// Adds arc lengths.
///
/// @param lhs Left arc length
/// @param rhs Right arc length
/// @return Sum of arc lengths
///
constexpr arc_length operator+(arc_length lhs, arc_length rhs) noexcept {
    return lhs += rhs;
}

///
/// Subtracts arc lengths.
///
/// @param lhs Left arc length
/// @param rhs Right arc length
/// @return Difference of arc lengths
///
constexpr arc_length operator-(arc_length lhs, arc_length rhs) noexcept {
    return lhs -= rhs;
}

///
/// Multiplies arc length by scalar.
///
/// @param len Arc length
/// @param scalar Scalar multiplier
/// @return Scaled arc length
///
constexpr arc_length operator*(arc_length len, double scalar) noexcept {
    return len *= scalar;
}

///
/// Multiplies scalar by arc length (commutative).
///
/// @param scalar Scalar multiplier
/// @param len Arc length
/// @return Scaled arc length
///
constexpr arc_length operator*(double scalar, arc_length len) noexcept {
    return len *= scalar;
}

///
/// Divides arc length by scalar.
///
/// @param len Arc length
/// @param scalar Scalar divisor
/// @return Scaled arc length
///
constexpr arc_length operator/(arc_length len, double scalar) noexcept {
    return len /= scalar;
}

///
/// Divides arc length by arc length (yields dimensionless ratio).
///
/// @param lhs Numerator arc length
/// @param rhs Denominator arc length
/// @return Dimensionless ratio
///
constexpr double operator/(arc_length lhs, arc_length rhs) noexcept {
    return static_cast<double>(lhs) / static_cast<double>(rhs);
}

///
/// Streams arc length for debugging and test output.
///
/// @param os Output stream
/// @param len Arc length to output
/// @return Reference to output stream
///
inline std::ostream& operator<<(std::ostream& os, arc_length len) {
    return os << static_cast<double>(len);
}

///
/// Returns absolute value of arc length.
///
/// @param len Arc length
/// @return Absolute arc length
///
inline arc_length abs(arc_length len) noexcept {
    return arc_length{std::abs(static_cast<double>(len))};
}

}  // namespace viam::trajex

#pragma once

#include <cmath>
#include <compare>
#include <numeric>
#include <ostream>

#include <viam/trajex/types/epsilon.hpp>

namespace viam::trajex {

///
/// Strong type for arc acceleration (d^2s/dt^2) in configuration space.
///
/// Provides type safety to prevent mixing arc acceleration with other quantities.
/// Supports dimensional analysis: acceleration * time = velocity.
///
class arc_acceleration {
   public:
    ///
    /// Constructs from raw double value.
    ///
    /// @param a Arc acceleration value
    ///
    explicit constexpr arc_acceleration(double a) noexcept : value_{a} {}

    ///
    /// Constructs from epsilon tolerance.
    ///
    /// Allows using epsilon as a dimensional quantity for numerical exploration.
    ///
    /// @param eps Epsilon tolerance value
    ///
    explicit constexpr arc_acceleration(class epsilon epsilon) noexcept : arc_acceleration{static_cast<double>(epsilon)} {}

    ///
    /// Converts to double explicitly.
    ///
    /// @return Arc acceleration as double
    ///
    explicit constexpr operator double() const noexcept {
        return value_;
    }

    ///
    /// Three-way comparison operator.
    ///
    constexpr auto operator<=>(const arc_acceleration&) const = default;

    ///
    /// Compound addition.
    ///
    /// @param other Arc acceleration to add
    /// @return Reference to this
    ///
    constexpr arc_acceleration& operator+=(arc_acceleration other) noexcept {
        value_ += static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound subtraction.
    ///
    /// @param other Arc acceleration to subtract
    /// @return Reference to this
    ///
    constexpr arc_acceleration& operator-=(arc_acceleration other) noexcept {
        value_ -= static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound scalar multiplication.
    ///
    /// @param scalar Scalar multiplier
    /// @return Reference to this
    ///
    constexpr arc_acceleration& operator*=(double scalar) noexcept {
        value_ *= scalar;
        return *this;
    }

    ///
    /// Compound scalar division.
    ///
    /// @param scalar Scalar divisor
    /// @return Reference to this
    ///
    constexpr arc_acceleration& operator/=(double scalar) noexcept {
        value_ /= scalar;
        return *this;
    }

    ///
    /// Unary negation.
    ///
    /// @return Negated arc acceleration
    ///
    constexpr arc_acceleration operator-() const noexcept {
        return arc_acceleration{-value_};
    }

   private:
    double value_;
};

///
/// Adds arc accelerations.
///
/// @param lhs Left arc acceleration
/// @param rhs Right arc acceleration
/// @return Sum of arc accelerations
///
constexpr arc_acceleration operator+(arc_acceleration lhs, arc_acceleration rhs) noexcept {
    return lhs += rhs;
}

///
/// Subtracts arc accelerations.
///
/// @param lhs Left arc acceleration
/// @param rhs Right arc acceleration
/// @return Difference of arc accelerations
///
constexpr arc_acceleration operator-(arc_acceleration lhs, arc_acceleration rhs) noexcept {
    return lhs -= rhs;
}

///
/// Multiplies arc acceleration by scalar.
///
/// @param acc Arc acceleration
/// @param scalar Scalar multiplier
/// @return Scaled arc acceleration
///
constexpr arc_acceleration operator*(arc_acceleration acc, double scalar) noexcept {
    return acc *= scalar;
}

///
/// Multiplies scalar by arc acceleration (commutative).
///
/// @param scalar Scalar multiplier
/// @param acc Arc acceleration
/// @return Scaled arc acceleration
///
constexpr arc_acceleration operator*(double scalar, arc_acceleration acc) noexcept {
    return acc *= scalar;
}

///
/// Divides arc acceleration by scalar.
///
/// @param acc Arc acceleration
/// @param scalar Scalar divisor
/// @return Scaled arc acceleration
///
constexpr arc_acceleration operator/(arc_acceleration acc, double scalar) noexcept {
    return acc /= scalar;
}

///
/// Divides arc acceleration by arc acceleration (yields dimensionless ratio).
///
/// @param lhs Numerator arc acceleration
/// @param rhs Denominator arc acceleration
/// @return Dimensionless ratio
///
constexpr double operator/(arc_acceleration lhs, arc_acceleration rhs) noexcept {
    return static_cast<double>(lhs) / static_cast<double>(rhs);
}

///
/// Streams arc acceleration for debugging and test output.
///
/// @param os Output stream
/// @param acc Arc acceleration to output
/// @return Reference to output stream
///
inline std::ostream& operator<<(std::ostream& os, arc_acceleration acc) {
    return os << static_cast<double>(acc);
}

///
/// Returns absolute value of arc acceleration.
///
/// @param acc Arc acceleration
/// @return Absolute arc acceleration
///
inline arc_acceleration abs(arc_acceleration acc) noexcept {
    return arc_acceleration{std::abs(static_cast<double>(acc))};
}

///
/// Linear interpolation between two arc accelerations.
///
/// Delegates to std::lerp for the underlying double values.
///
/// @param a Starting arc acceleration
/// @param b Ending arc acceleration
/// @param t Interpolation factor (0.0 yields a, 1.0 yields b)
/// @return Interpolated arc acceleration
///
inline arc_acceleration lerp(arc_acceleration a, arc_acceleration b, double t) {
    return arc_acceleration{std::lerp(static_cast<double>(a), static_cast<double>(b), t)};
}

///
/// Midpoint between two arc accelerations.
///
/// Delegates to std::midpoint for the underlying double values.
///
/// @param a First arc acceleration
/// @param b Second arc acceleration
/// @return Midpoint arc acceleration
///
inline arc_acceleration midpoint(arc_acceleration a, arc_acceleration b) {
    return arc_acceleration{std::midpoint(static_cast<double>(a), static_cast<double>(b))};
}

}  // namespace viam::trajex

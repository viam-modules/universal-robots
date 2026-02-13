#pragma once

#include <cmath>
#include <compare>
#include <numeric>
#include <ostream>

#include <viam/trajex/types/epsilon.hpp>

namespace viam::trajex {

///
/// Strong type for arc velocity (ds/dt) in configuration space.
///
/// Provides type safety to prevent mixing arc velocity with other quantities.
/// Supports dimensional analysis: velocity * time = length, velocity / time = acceleration.
///
class arc_velocity {
   public:
    ///
    /// Constructs from raw double value.
    ///
    /// @param v Arc velocity value
    ///
    explicit constexpr arc_velocity(double v) noexcept : value_{v} {}

    ///
    /// Constructs from epsilon tolerance.
    ///
    /// Allows using epsilon as a dimensional quantity for numerical exploration.
    ///
    /// @param eps Epsilon tolerance value
    ///
    explicit constexpr arc_velocity(class epsilon epsilon) noexcept : arc_velocity{static_cast<double>(epsilon)} {}

    ///
    /// Converts to double explicitly.
    ///
    /// @return Arc velocity as double
    ///
    explicit constexpr operator double() const noexcept {
        return value_;
    }

    ///
    /// Three-way comparison operator.
    ///
    constexpr auto operator<=>(const arc_velocity&) const = default;

    ///
    /// Compound addition.
    ///
    /// @param other Arc velocity to add
    /// @return Reference to this
    ///
    constexpr arc_velocity& operator+=(arc_velocity other) noexcept {
        value_ += static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound subtraction.
    ///
    /// @param other Arc velocity to subtract
    /// @return Reference to this
    ///
    constexpr arc_velocity& operator-=(arc_velocity other) noexcept {
        value_ -= static_cast<double>(other);
        return *this;
    }

    ///
    /// Compound scalar multiplication.
    ///
    /// @param scalar Scalar multiplier
    /// @return Reference to this
    ///
    constexpr arc_velocity& operator*=(double scalar) noexcept {
        value_ *= scalar;
        return *this;
    }

    ///
    /// Compound scalar division.
    ///
    /// @param scalar Scalar divisor
    /// @return Reference to this
    ///
    constexpr arc_velocity& operator/=(double scalar) noexcept {
        value_ /= scalar;
        return *this;
    }

    ///
    /// Unary negation.
    ///
    /// @return Negated arc velocity
    ///
    constexpr arc_velocity operator-() const noexcept {
        return arc_velocity{-value_};
    }

   private:
    double value_;
};

///
/// Adds arc velocities.
///
/// @param lhs Left arc velocity
/// @param rhs Right arc velocity
/// @return Sum of arc velocities
///
constexpr arc_velocity operator+(arc_velocity lhs, arc_velocity rhs) noexcept {
    return lhs += rhs;
}

///
/// Subtracts arc velocities.
///
/// @param lhs Left arc velocity
/// @param rhs Right arc velocity
/// @return Difference of arc velocities
///
constexpr arc_velocity operator-(arc_velocity lhs, arc_velocity rhs) noexcept {
    return lhs -= rhs;
}

///
/// Multiplies arc velocity by scalar.
///
/// @param vel Arc velocity
/// @param scalar Scalar multiplier
/// @return Scaled arc velocity
///
constexpr arc_velocity operator*(arc_velocity vel, double scalar) noexcept {
    return vel *= scalar;
}

///
/// Multiplies scalar by arc velocity (commutative).
///
/// @param scalar Scalar multiplier
/// @param vel Arc velocity
/// @return Scaled arc velocity
///
constexpr arc_velocity operator*(double scalar, arc_velocity vel) noexcept {
    return vel *= scalar;
}

///
/// Divides arc velocity by scalar.
///
/// @param vel Arc velocity
/// @param scalar Scalar divisor
/// @return Scaled arc velocity
///
constexpr arc_velocity operator/(arc_velocity vel, double scalar) noexcept {
    return vel /= scalar;
}

///
/// Divides arc velocity by arc velocity (yields dimensionless ratio).
///
/// @param lhs Numerator arc velocity
/// @param rhs Denominator arc velocity
/// @return Dimensionless ratio
///
constexpr double operator/(arc_velocity lhs, arc_velocity rhs) noexcept {
    return static_cast<double>(lhs) / static_cast<double>(rhs);
}

///
/// Streams arc velocity for debugging and test output.
///
/// @param os Output stream
/// @param vel Arc velocity to output
/// @return Reference to output stream
///
inline std::ostream& operator<<(std::ostream& os, arc_velocity vel) {
    return os << static_cast<double>(vel);
}

///
/// Returns absolute value of arc velocity.
///
/// @param vel Arc velocity
/// @return Absolute arc velocity
///
inline arc_velocity abs(arc_velocity vel) noexcept {
    return arc_velocity{std::abs(static_cast<double>(vel))};
}

///
/// Linear interpolation between two arc velocities.
///
/// Delegates to std::lerp for the underlying double values.
///
/// @param a Starting arc velocity
/// @param b Ending arc velocity
/// @param t Interpolation factor (0.0 yields a, 1.0 yields b)
/// @return Interpolated arc velocity
///
inline arc_velocity lerp(arc_velocity a, arc_velocity b, double t) {
    return arc_velocity{std::lerp(static_cast<double>(a), static_cast<double>(b), t)};
}

///
/// Midpoint between two arc velocities.
///
/// Delegates to std::midpoint for the underlying double values.
///
/// @param a First arc velocity
/// @param b Second arc velocity
/// @return Midpoint arc velocity
///
inline arc_velocity midpoint(arc_velocity a, arc_velocity b) {
    return arc_velocity{std::midpoint(static_cast<double>(a), static_cast<double>(b))};
}

}  // namespace viam::trajex

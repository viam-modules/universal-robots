#pragma once

#include <compare>
#include <concepts>

namespace viam::trajex {

namespace epsilon_details {

///
/// Concept for types that can be explicitly cast to double.
///
template <typename T>
concept explicitly_convertible_to_double = requires(T t) {
    {
        static_cast<double>(t)
    } -> std::same_as<double>;
};

}  // namespace epsilon_details

///
/// Tolerance threshold for numerical comparisons.
///
/// Represents "indistinguishability" threshold for comparing dimensional quantities.
/// Values closer than epsilon are considered equal.
///
/// Only supports comparisons, not arithmetic operations. This prevents accidentally
/// using epsilon in computations while allowing clean comparison syntax.
///
class epsilon {
   public:
    ///
    /// Constructs epsilon from tolerance value.
    ///
    /// @param value Tolerance threshold
    ///
    explicit constexpr epsilon(double value) noexcept : value_{value} {}

    ///
    /// Converts to raw double value.
    ///
    /// @return Tolerance value
    ///
    explicit constexpr operator double() const noexcept {
        return value_;
    }

    ///
    /// Compound scalar multiplication.
    ///
    /// @param scalar Scalar multiplier
    /// @return Reference to this
    ///
    constexpr epsilon& operator*=(double scalar) noexcept {
        value_ *= scalar;
        return *this;
    }

    ///
    /// Compound scalar division.
    ///
    /// @param scalar Scalar divisor
    /// @return Reference to this
    ///
    constexpr epsilon& operator/=(double scalar) noexcept {
        value_ /= scalar;
        return *this;
    }

    ///
    /// Unary negation.
    ///
    /// @return Negated epsilon
    ///
    constexpr epsilon operator-() const noexcept {
        return epsilon{-value_};
    }

   private:
    double value_;
};

///
/// Three-way comparison between dimensional type and epsilon.
///
/// Compares the dimensional quantity against epsilon threshold.
/// Generates relational operators (<, <=, >, >=).
///
/// @param lhs Dimensional value (arc_length, arc_velocity, arc_acceleration)
/// @param rhs Epsilon tolerance
/// @return Comparison result
///
// clang-format off
template <epsilon_details::explicitly_convertible_to_double T>
constexpr auto operator<=>(const T& lhs, epsilon rhs) noexcept {
    return static_cast<double>(lhs) <=> static_cast<double>(rhs);
}
// clang-format on

///
/// Three-way comparison between epsilon and dimensional type (reversed).
///
/// Compares epsilon threshold against the dimensional quantity.
/// Generates relational operators (<, <=, >, >=).
///
/// @param lhs Epsilon tolerance
/// @param rhs Dimensional value (arc_length, arc_velocity, arc_acceleration)
/// @return Comparison result
///
// clang-format off
template <epsilon_details::explicitly_convertible_to_double T>
constexpr auto operator<=>(epsilon lhs, const T& rhs) noexcept {
    return static_cast<double>(lhs) <=> static_cast<double>(rhs);
}
// clang-format on

///
/// Equality comparison between dimensional type and epsilon.
///
/// Required because operator<=> doesn't generate operator== for heterogeneous types.
/// Implemented in terms of operator<=> to avoid duplication.
///
/// @param lhs Dimensional value
/// @param rhs Epsilon tolerance
/// @return True if lhs == rhs
///
template <typename T>
constexpr bool operator==(const T& lhs, epsilon rhs) noexcept {
    // clang-format off
    return (lhs <=> rhs) == 0;
    // clang-format on
}

///
/// Equality comparison between epsilon and dimensional type (reversed).
///
/// Required because operator<=> doesn't generate operator== for heterogeneous types.
/// Implemented in terms of operator<=> to avoid duplication.
///
/// @param lhs Epsilon tolerance
/// @param rhs Dimensional value
/// @return True if lhs == rhs
///
template <typename T>
constexpr bool operator==(epsilon lhs, const T& rhs) noexcept {
    // clang-format off
    return (lhs <=>rhs) == 0;
    // clang-format on
}

///
/// Multiplies epsilon by scalar.
///
/// @param eps Epsilon tolerance
/// @param scalar Scalar multiplier
/// @return Scaled epsilon
///
constexpr epsilon operator*(epsilon eps, double scalar) noexcept {
    return eps *= scalar;
}

///
/// Multiplies scalar by epsilon (commutative).
///
/// @param scalar Scalar multiplier
/// @param eps Epsilon tolerance
/// @return Scaled epsilon
///
constexpr epsilon operator*(double scalar, epsilon eps) noexcept {
    return eps *= scalar;
}

///
/// Divides epsilon by scalar.
///
/// @param eps Epsilon tolerance
/// @param scalar Scalar divisor
/// @return Scaled epsilon
///
constexpr epsilon operator/(epsilon eps, double scalar) noexcept {
    return eps /= scalar;
}

}  // namespace viam::trajex

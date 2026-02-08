#pragma once

#include <algorithm>
#include <compare>
#include <concepts>

namespace viam::trajex {

namespace epsilon_details {

///
/// Concept for types that can be explicitly cast to double.
///
template <typename T>
concept explicitly_convertible_to_double = requires(T t) {
    { static_cast<double>(t) } -> std::same_as<double>;
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
/// Wrapped values from different epsilon objects are compared using the stricter
/// (minimum) tolerance.
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

    template <typename T>
    class wrapper;

    template <typename T>
    constexpr wrapper<T> wrap(const T& t) const noexcept;

   private:
    double value_;
};

namespace epsilon_details {

///
/// Concept for types that can be ordered with each other using epsilon tolerance.
///
/// Two types satisfy this concept if their difference can be compared with epsilon
/// using relational operators, preserving dimensional consistency.
///
template <typename U, typename V>
concept ordered_with = requires(U u, V v, epsilon e) {
    { (u - v) > e } -> std::convertible_to<bool>;
    { (u - v) < e } -> std::convertible_to<bool>;
};

}  // namespace epsilon_details

template <typename T>
class epsilon::wrapper {
    friend class epsilon;

    explicit constexpr wrapper(const epsilon& e, T const& t) noexcept : e_{e}, t_{t} {}

    template <typename U, typename V>
    friend constexpr auto operator<=>(const wrapper<U>& lhs, const wrapper<V>& rhs) noexcept
        requires epsilon_details::ordered_with<U, V>;

    template <typename U, typename V>
    friend constexpr bool operator==(const wrapper<U>& lhs, const wrapper<V>& rhs) noexcept
        requires epsilon_details::ordered_with<U, V>;

    const epsilon& e_;
    const T& t_;
};

template <typename T>
constexpr epsilon::wrapper<T> epsilon::wrap(const T& t) const noexcept {
    return wrapper<T>(*this, t);
}

///
/// Three-way comparison between wrapped values with epsilon tolerance.
///
/// Compares two values accounting for epsilon tolerance. Values are considered
/// equivalent if their difference is within tolerance, otherwise ordered by
/// which value is significantly greater.
///
/// When wrappers have different epsilon values, uses the stricter (minimum) tolerance.
///
/// @param lhs Left wrapped value
/// @param rhs Right wrapped value
/// @return weak_ordering::less if rhs significantly exceeds lhs,
///         weak_ordering::greater if lhs significantly exceeds rhs,
///         weak_ordering::equivalent if values are within tolerance
///
template <typename U, typename V>
constexpr auto operator<=>(const epsilon::wrapper<U>& lhs, const epsilon::wrapper<V>& rhs) noexcept
    requires epsilon_details::ordered_with<U, V>
{
    const auto tol = std::min(lhs.e_, rhs.e_);
    const auto diff = lhs.t_ - rhs.t_;

    if (diff > tol) {
        return std::weak_ordering::greater;
    }
    if (diff < -tol) {
        return std::weak_ordering::less;
    }
    return std::weak_ordering::equivalent;
}

///
/// Equality comparison between wrapped values with epsilon tolerance.
///
/// Required because operator<=> doesn't generate operator== for free functions.
///
/// @param lhs Left wrapped value
/// @param rhs Right wrapped value
/// @return True if values are within tolerance
///
template <typename U, typename V>
constexpr bool operator==(const epsilon::wrapper<U>& lhs, const epsilon::wrapper<V>& rhs) noexcept
    requires epsilon_details::ordered_with<U, V>
{
    return (lhs <=> rhs) == 0;
}

///
/// Three-way comparison between epsilon values.
///
/// @param lhs Left epsilon
/// @param rhs Right epsilon
/// @return Comparison result
///
constexpr auto operator<=>(epsilon lhs, epsilon rhs) noexcept {
    return static_cast<double>(lhs) <=> static_cast<double>(rhs);
}

///
/// Equality comparison between epsilon values.
///
/// @param lhs Left epsilon
/// @param rhs Right epsilon
/// @return True if equal
///
constexpr bool operator==(epsilon lhs, epsilon rhs) noexcept {
    return (lhs <=> rhs) == 0;
}

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
template <epsilon_details::explicitly_convertible_to_double T>
constexpr auto operator<=>(const T& lhs, epsilon rhs) noexcept {
    return static_cast<double>(lhs) <=> static_cast<double>(rhs);
}

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
template <epsilon_details::explicitly_convertible_to_double T>
constexpr auto operator<=>(epsilon lhs, const T& rhs) noexcept {
    return static_cast<double>(lhs) <=> static_cast<double>(rhs);
}

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
    return (lhs <=> rhs) == 0;
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
    return (lhs <=> rhs) == 0;
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

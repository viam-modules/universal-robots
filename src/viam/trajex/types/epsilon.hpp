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

template <typename T>
class epsilon::wrapper {
    explicit constexpr wrapper(const epsilon& e, T const& t) : e_{e}, t_{t} {}

    friend class epsilon;

    template <typename U, typename V>
    friend constexpr auto operator<=>(const wrapper<U>& lhs, const wrapper<V>& rhs) noexcept
        requires std::three_way_comparable_with<U, V> && epsilon_details::explicitly_convertible_to_double<U> &&
                 epsilon_details::explicitly_convertible_to_double<V>;

    template <typename U, typename V>
    friend constexpr bool operator==(const wrapper<U>& lhs, const wrapper<V>& rhs) noexcept
        requires std::three_way_comparable_with<U, V> && epsilon_details::explicitly_convertible_to_double<U> &&
                 epsilon_details::explicitly_convertible_to_double<V>;

   private:
    const epsilon& e_;
    T const& t_;
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
template <typename T, typename U>
constexpr auto operator<=>(const epsilon::wrapper<T>& lhs, const epsilon::wrapper<U>& rhs) noexcept
    requires std::three_way_comparable_with<T, U> && epsilon_details::explicitly_convertible_to_double<T> &&
             epsilon_details::explicitly_convertible_to_double<U>
{
    const double tol = std::min(static_cast<double>(lhs.e_), static_cast<double>(rhs.e_));
    const double diff = static_cast<double>(lhs.t_) - static_cast<double>(rhs.t_);

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
template <typename T, typename U>
constexpr bool operator==(const epsilon::wrapper<T>& lhs, const epsilon::wrapper<U>& rhs) noexcept
    requires std::three_way_comparable_with<T, U> && epsilon_details::explicitly_convertible_to_double<T> &&
             epsilon_details::explicitly_convertible_to_double<U>
{
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

#pragma once

#include <ranges>
#include <utility>
#include <vector>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#include <xtensor/views/xslice.hpp>
#include <xtensor/views/xview.hpp>
#else
#include <xtensor/xarray.hpp>
#include <xtensor/xslice.hpp>
#include <xtensor/xview.hpp>
#endif

namespace viam::trajex::totg {

/// Accumulator for building up a sequence of waypoints
///
/// **Lifetime requirements**: All waypoint arrays passed to this object must outlive it.
/// **Move semantics**: This class is move-only (not copyable).
///
/// Usage:
/// @code
///   waypoint_accumulator waypoints(initial_waypoints);
///   waypoints.add_waypoints(more_waypoints);
/// @endcode
class waypoint_accumulator {
   public:
    /// Construct with initial waypoints
    /// @param waypoints 2D array (num_waypoints, num_joints)
    /// @note The waypoints array must outlive the waypoint_accumulator object
    explicit waypoint_accumulator(const xt::xarray<double>& waypoints);

    // Move-only semantics
    waypoint_accumulator(const waypoint_accumulator&) = delete;
    waypoint_accumulator& operator=(const waypoint_accumulator&) = delete;
    waypoint_accumulator(waypoint_accumulator&&) = default;
    waypoint_accumulator& operator=(waypoint_accumulator&&) = default;

    /// Add additional waypoints
    /// @param waypoints 2D array where each row is a waypoint, shape (num_waypoints, num_joints)
    /// @return Reference to this object for method chaining
    /// @note The waypoints array must outlive the waypoint_accumulator object
    waypoint_accumulator& add_waypoints(const xt::xarray<double>& waypoints);

    /// Get the number of degrees of freedom
    /// @return Number of DOF
    size_t dof() const noexcept;

    /// Get the number of waypoints
    /// @return Total number of waypoints
    size_t size() const noexcept;

    /// Check if empty
    /// @return True if no waypoints
    bool empty() const noexcept;

    /// Access to waypoint views for path construction
    using waypoint_view_t = decltype(xt::view(std::declval<const xt::xarray<double>&>(), std::declval<size_t>(), xt::all()));
    using const_iterator = std::vector<waypoint_view_t>::const_iterator;
    using value_type = waypoint_view_t;
    using size_type = std::size_t;

    /// Begin iterator (const-only container)
    const_iterator begin() const noexcept;

    /// End iterator (const-only container)
    const_iterator end() const noexcept;

    /// Begin const iterator (same as begin() for const-only container)
    const_iterator cbegin() const noexcept;

    /// End const iterator (same as end() for const-only container)
    const_iterator cend() const noexcept;

    /// Access waypoint by index (no bounds checking)
    /// @param i Index of waypoint
    /// @return Reference to waypoint view
    const waypoint_view_t& operator[](size_t i) const;

    /// Access waypoint by index with bounds checking
    /// @param i Index of waypoint
    /// @return Reference to waypoint view
    /// @throws std::out_of_range if i >= size()
    const waypoint_view_t& at(size_t i) const;

   private:
    size_t dof_;
    std::vector<waypoint_view_t> waypoints_;
};

// Verify waypoint_accumulator satisfies C++20 range concepts
static_assert(std::ranges::range<waypoint_accumulator>);
static_assert(std::ranges::sized_range<waypoint_accumulator>);
static_assert(std::ranges::forward_range<waypoint_accumulator>);

}  // namespace viam::trajex::totg

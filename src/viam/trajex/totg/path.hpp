#pragma once

#include <ranges>
#include <variant>
#include <vector>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/types/arc_length.hpp>

namespace viam::trajex::totg {

// Forward declaration
class trajectory;

/// Geometric path through configuration space with linear segments and circular blends
///
/// **Ownership**: Owns all segment data. Safe to use after source waypoints are destroyed.
/// **Thread safety**: All const methods are thread-safe for concurrent access.
///
/// Created via path::create() from waypoints using tube-based coalescing algorithm.
/// Provides queries for segment lookup and arc length parameterization.
///
/// Example usage:
/// @code
///   // Create path from waypoints
///   xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
///   path p = path::create(waypoints);
///
///   // Query path at specific arc length
///   arc_length s{0.5};
///   auto config = p.configuration(s);
///   auto tangent = p.tangent(s);
///   auto curvature = p.curvature(s);
///
///   // Iterate over segments
///   for (const auto& view : p) {
///       view.visit([](const auto& seg_data) {
///           // Process segment data
///       });
///   }
/// @endcode
class path {
   public:
    /// Geometric segment in configuration space
    class segment {
       public:
        /// Linear segment in configuration space
        struct linear {
            /// Construct from start and end configurations
            /// Computes unit_direction and length from endpoints
            /// @throws std::invalid_argument if start == end
            linear(xt::xarray<double> start, xt::xarray<double> end);

            xt::xarray<double> start;           ///< Starting configuration
            xt::xarray<double> unit_direction;  ///< Precomputed unit direction vector (normalized end-start)
            arc_length length;                  ///< Precomputed length (norm of end-start)
        };

        /// Circular blend segment in configuration space
        struct circular {
            /// Construct circular arc with validation
            /// @param center Center of arc in configuration space
            /// @param x First basis vector (must be unit length)
            /// @param y Second basis vector (must be unit length and perpendicular to x)
            /// @param radius Radius of the circular arc
            /// @param angle_rads Total angle swept by arc (radians)
            /// @throws std::invalid_argument if x,y are not orthonormal
            circular(xt::xarray<double> center, xt::xarray<double> x, xt::xarray<double> y, double radius, double angle_rads);

            xt::xarray<double> center;  ///< Center of arc in configuration space
            xt::xarray<double> x;       ///< First basis vector (defines rotation plane)
            xt::xarray<double> y;       ///< Second basis vector (perpendicular to x)
            double radius;              ///< Radius of the circular arc (units: configuration space distance)
            double angle_rads;          ///< Total angle swept by arc (units: radians)
        };

        /// View of a segment's position within the path
        ///
        /// Provides the primary query interface for segments, mapping global path
        /// arc lengths to local segment parameters.
        class view {
           public:
            /// Construct view from segment reference and arc length bounds
            view(const class segment& seg, arc_length start, arc_length end) noexcept;

            /// Access the underlying segment
            const class segment& segment() const noexcept;

            /// Get starting arc length on path
            arc_length start() const noexcept;

            /// Get ending arc length on path
            arc_length end() const noexcept;

            /// Get length of this view
            /// @return Length of the view (end - start)
            arc_length length() const noexcept;

            /// Check if segment holds a specific type
            /// @tparam T Type to check (segment::linear or segment::circular)
            /// @return True if segment holds type T
            template <typename T>
            bool is() const noexcept {
                return std::holds_alternative<T>(seg_.get().data_);
            }

            /// Visit the underlying segment variant
            /// @tparam Visitor Callable object type
            /// @param v Visitor to apply to the segment variant
            /// @return Result of visiting the variant
            template <typename Visitor>
            decltype(auto) visit(Visitor&& v) const {
                return std::visit(std::forward<Visitor>(v), seg_.get().data_);
            }

            /// Get configuration at global arc length
            /// @param s Global arc length on path
            /// @return Configuration vector at arc length s
            xt::xarray<double> configuration(arc_length s) const;

            /// Get tangent vector at global arc length
            /// @param s Global arc length on path
            /// @return Unit tangent vector at arc length s
            xt::xarray<double> tangent(arc_length s) const;

            /// Get curvature vector at global arc length
            /// @param s Global arc length on path
            /// @return Curvature vector at arc length s
            xt::xarray<double> curvature(arc_length s) const;

           private:
            std::reference_wrapper<const class segment> seg_;  ///< Reference to the segment
            arc_length start_;                                 ///< Starting arc length on path
            arc_length end_;                                   ///< Ending arc length on path
        };

        explicit segment(linear data);
        explicit segment(circular data);

       private:
        using detail = std::variant<linear, circular>;
        detail data_;
    };

    /// Default maximum deviation for circular blends.
    /// 0.0 means no blending - path goes exactly through waypoints.
    /// TODO(acm): Consider using a positive-double concept/type for this and similar parameters
    /// (delta, epsilon, and possibly arc_length which should be non-negative)
    static constexpr double k_default_max_deviation = 0.0;

    /// Create a path from waypoints using tube-based coalescing
    ///
    /// **Note**: Uses static factory pattern rather than constructor because path
    /// construction performs non-trivial computation (tube coalescing algorithm).
    /// This makes the computational cost explicit and avoids exception safety
    /// concerns with partially-constructed objects.
    ///
    /// @param waypoints Waypoint sequence to follow
    /// @param max_deviation Maximum deviation from waypoints for circular blends
    /// @return Constructed path with segments
    [[nodiscard]] static path create(const waypoint_accumulator& waypoints, double max_deviation = k_default_max_deviation);

    /// Convenience overload: create path directly from waypoint array
    /// @param waypoints 2D array (num_waypoints, dof) of waypoints
    /// @param max_deviation Maximum deviation from waypoints for circular blends
    /// @return Constructed path with segments
    [[nodiscard]] static path create(const xt::xarray<double>& waypoints, double max_deviation = k_default_max_deviation);

    /// Get total arc length of path
    /// @return Total length in configuration space
    arc_length length() const noexcept;

    /// Get number of segments
    /// @return Number of linear and circular segments
    size_t size() const noexcept;

    /// Check if path is empty
    /// @return True if path has no segments
    bool empty() const noexcept;

    /// Container semantics: forward iterator over segment views
    class const_iterator;

    /// Begin iterator for segment views
    const_iterator begin() const noexcept;

    /// End iterator for segment views
    const_iterator end() const noexcept;

    /// Begin const iterator for segment views (same as begin)
    const_iterator cbegin() const noexcept;

    /// End const iterator for segment views (same as end)
    const_iterator cend() const noexcept;

    /// Get number of degrees of freedom
    /// @return Number of DOF
    size_t dof() const noexcept;

    /// Evaluate path at given arc length
    /// @param s Arc length along path
    /// @return View of the segment at the given arc length, including range [start, end)
    segment::view operator()(arc_length s) const;

    /// Get configuration at arc length
    /// @param s Arc length along path
    /// @return Configuration vector at s
    xt::xarray<double> configuration(arc_length s) const;

    /// Get tangent at arc length
    /// @param s Arc length along path
    /// @return Unit tangent vector at s
    xt::xarray<double> tangent(arc_length s) const;

    /// Get curvature at arc length
    /// @param s Arc length along path
    /// @return Curvature vector at s
    xt::xarray<double> curvature(arc_length s) const;

   private:
    /// Internal storage: segment with its starting position on the path
    struct positioned_segment {
        segment seg;       ///< The segment
        arc_length start;  ///< Starting arc_length of segment on path
    };

    path(std::vector<positioned_segment> segments, size_t dof, arc_length length);

    std::vector<positioned_segment> segments_;
    size_t dof_{0};
    arc_length length_{0.0};

    friend class const_iterator;
};

/// Forward iterator for path segments, yielding segment::view on dereference
///
/// **Note:** Dereferencing returns segment::view by value, not by reference.
/// Each dereference constructs a new view object. This is intentional - views are
/// lightweight (reference + 2 arc_lengths) and computed on-the-fly from storage.
///
/// Usage:
/// @code
///   auto it = path.begin();
///   auto view = *it;        // Copy view by value (recommended)
///   auto& ref = *it;        // Binds to temporary - avoid!
///
///   // Multiple dereferences create distinct objects:
///   &(*it) != &(*it)        // true - different addresses
/// @endcode
class path::const_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = segment::view;
    using difference_type = std::ptrdiff_t;
    using pointer = const segment::view*;
    using reference = segment::view;

    /// Default constructor (creates singular iterator)
    const_iterator() noexcept;

    /// Dereference: returns segment::view for current segment
    segment::view operator*() const;

    /// Pre-increment
    const_iterator& operator++() noexcept;

    /// Post-increment
    const_iterator operator++(int) noexcept;

    /// Equality comparison
    bool operator==(const const_iterator& other) const noexcept;

   private:
    friend class path;

    const_iterator(const path* p, std::vector<positioned_segment>::const_iterator it) noexcept : path_{p}, it_{it} {}

    const path* path_;
    std::vector<positioned_segment>::const_iterator it_;
};

// Verify that path satisfies range concepts
static_assert(std::ranges::range<path>);
static_assert(std::ranges::sized_range<path>);
static_assert(std::ranges::forward_range<path>);

}  // namespace viam::trajex::totg

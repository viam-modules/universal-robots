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

///
/// Geometric path through configuration space with linear segments and circular blends.
///
/// Provides queries for segment lookup and arc length parameterization.
///
/// **Ownership**: Owns all segment data. Safe to use after source waypoints are destroyed.
///
/// **Thread safety**: All const methods are thread-safe for concurrent access.
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
///
class path {
   public:
    ///
    /// Geometric segment in configuration space.
    ///
    class segment {
       public:
        ///
        /// Linear segment in configuration space.
        ///
        struct linear {
            ///
            /// Constructs linear segment from start and end configurations.
            ///
            /// Computes unit_direction and length from endpoints.
            ///
            /// @param start Starting configuration
            /// @param end Ending configuration
            /// @throws std::invalid_argument if start == end
            ///
            linear(xt::xarray<double> start, const xt::xarray<double>& end);

            xt::xarray<double> start;           ///< Starting configuration
            xt::xarray<double> unit_direction;  ///< Precomputed unit direction vector (normalized end-start)
            arc_length length;                  ///< Precomputed length (norm of end-start)
        };

        ///
        /// Circular blend segment in configuration space.
        ///
        struct circular {
            ///
            /// Constructs circular arc with validation.
            ///
            /// @param center Center of arc in configuration space
            /// @param x First basis vector (must be unit length)
            /// @param y Second basis vector (must be unit length and perpendicular to x)
            /// @param radius Radius of the circular arc
            /// @param angle_rads Total angle swept by arc (radians)
            /// @throws std::invalid_argument if x,y are not orthonormal
            ///
            circular(xt::xarray<double> center, xt::xarray<double> x, xt::xarray<double> y, double radius, double angle_rads);

            xt::xarray<double> center;  ///< Center of arc in configuration space
            xt::xarray<double> x;       ///< First basis vector (defines rotation plane)
            xt::xarray<double> y;       ///< Second basis vector (perpendicular to x)
            double radius;              ///< Radius of the circular arc (units: configuration space distance)
            double angle_rads;          ///< Total angle swept by arc (units: radians)
        };

        ///
        /// View of a segment's position within the path.
        ///
        /// Provides the primary query interface for segments, mapping global path
        /// arc lengths to local segment parameters.
        ///
        class view {
           public:
            ///
            /// Constructs view from segment reference and arc length bounds.
            ///
            /// @param seg Segment to view
            /// @param start Starting arc length on path
            /// @param end Ending arc length on path
            ///
            view(const class segment& seg, arc_length start, arc_length end) noexcept;

            ///
            /// Gets the underlying segment.
            ///
            /// @return Reference to the segment
            ///
            const class segment& segment() const noexcept;

            ///
            /// Gets the starting arc length on path.
            ///
            /// @return Starting arc length
            ///
            arc_length start() const noexcept;

            ///
            /// Gets the ending arc length on path.
            ///
            /// @return Ending arc length
            ///
            arc_length end() const noexcept;

            ///
            /// Gets the length of this view.
            ///
            /// @return Length of the view (end - start)
            ///
            arc_length length() const noexcept;

            ///
            /// Checks if segment holds a specific type.
            ///
            /// @tparam T Type to check (segment::linear or segment::circular)
            /// @return True if segment holds type T
            ///
            template <typename T>
            bool is() const noexcept {
                return std::holds_alternative<T>(seg_.get().data_);
            }

            ///
            /// Visits the underlying segment variant.
            ///
            /// @tparam Visitor Callable object type
            /// @param v Visitor to apply to the segment variant
            /// @return Result of visiting the variant
            ///
            template <typename Visitor>
            decltype(auto) visit(Visitor&& v) const {
                return std::visit(std::forward<Visitor>(v), seg_.get().data_);
            }

            ///
            /// Gets configuration at global arc length.
            ///
            /// @param s Global arc length on path
            /// @return Configuration vector at arc length s
            ///
            xt::xarray<double> configuration(arc_length s) const;

            ///
            /// Gets tangent vector at global arc length.
            ///
            /// @param s Global arc length on path
            /// @return Unit tangent vector at arc length s
            ///
            xt::xarray<double> tangent(arc_length s) const;

            ///
            /// Gets curvature vector at global arc length.
            ///
            /// @param s Global arc length on path
            /// @return Curvature vector at arc length s
            ///
            xt::xarray<double> curvature(arc_length s) const;

           private:
            // Reference to the segment
            std::reference_wrapper<const class segment> seg_;

            // Arc length bounds on path
            arc_length start_;
            arc_length end_;
        };

        ///
        /// Constructs segment from linear data.
        ///
        /// @param data Linear segment data
        ///
        explicit segment(linear data);

        ///
        /// Constructs segment from circular data.
        ///
        /// @param data Circular segment data
        ///
        explicit segment(circular data);

       private:
        using detail = std::variant<linear, circular>;

        // Variant holding either linear or circular segment data
        detail data_;
    };

    ///
    /// Options for path creation with fluent interface.
    ///
    class options {
       public:
        ///
        /// Default maximum deviation.
        ///
        static constexpr double k_default_max_deviation = 0.0;

        ///
        /// Constructs options with default values.
        ///
        options();

        ///
        /// Sets maximum blend deviation.
        ///
        /// @param deviation Maximum distance blend arc can deviate from corner
        /// @return Reference to this for method chaining
        ///
        options& set_max_deviation(double deviation);

        ///
        /// Sets maximum blend deviation.
        ///
        /// @param deviation Maximum distance blend arc can deviate from corner
        /// @return Reference to this for method chaining
        /// @note Intended primarily for testing
        ///
        options& set_max_blend_deviation(double deviation);

        ///
        /// Sets maximum linear deviation for coalescing.
        ///
        /// This is `Divergent Behavior 3`: the Kunz & Stilman paper does not include this pass.
        ///
        /// @param deviation Maximum deviation for waypoint coalescing
        /// @return Reference to this for method chaining
        /// @note Intended primarily for testing
        ///
        options& set_max_linear_deviation(double deviation);

        ///
        /// Gets maximum blend deviation.
        ///
        /// @return Maximum distance blend arc can deviate from corner
        ///
        double max_blend_deviation() const noexcept;

        ///
        /// Gets maximum linear deviation.
        ///
        /// @return Maximum deviation for waypoint coalescing
        ///
        double max_linear_deviation() const noexcept;

       private:
        double max_blend_deviation_;
        double max_linear_deviation_;
    };

    ///
    /// Creates a path from a waypoints accumulator.
    ///
    /// @param waypoints Waypoint sequence to follow
    /// @param opts Path creation options (coalescing and blending parameters)
    /// @return Constructed path with segments
    ///
    [[nodiscard]] static path create(const waypoint_accumulator& waypoints, const options& opts = options{});

    ///
    /// Creates path directly from waypoint array.
    ///
    /// Convenience overload that constructs waypoint_accumulator internally.
    ///
    /// @param waypoints 2D array (num_waypoints, dof) of waypoints
    /// @param opts Path creation options (coalescing and blending parameters)
    /// @return Constructed path with segments
    ///
    [[nodiscard]] static path create(const xt::xarray<double>& waypoints, const options& opts = options{});

    ///
    /// Gets total arc length of path.
    ///
    /// @return Total length in configuration space
    ///
    arc_length length() const noexcept;

    ///
    /// Gets number of segments.
    ///
    /// @return Number of linear and circular segments
    ///
    size_t size() const noexcept;

    ///
    /// Checks if path is empty.
    ///
    /// @return True if path has no segments
    ///
    bool empty() const noexcept;

    ///
    /// Forward iterator over segment views.
    ///
    class const_iterator;

    ///
    /// Gets begin iterator for segment views.
    ///
    /// @return Iterator to first segment
    ///
    const_iterator begin() const noexcept;

    ///
    /// Gets end iterator for segment views.
    ///
    /// @return Iterator past last segment
    ///
    const_iterator end() const noexcept;

    ///
    /// Gets const begin iterator for segment views.
    ///
    /// @return Const iterator to first segment (same as begin())
    ///
    const_iterator cbegin() const noexcept;

    ///
    /// Gets const end iterator for segment views.
    ///
    /// @return Const iterator past last segment (same as end())
    ///
    const_iterator cend() const noexcept;

    ///
    /// Gets number of degrees of freedom.
    ///
    /// @return Number of DOF
    ///
    size_t dof() const noexcept;

    ///
    /// Evaluates path at given arc length.
    ///
    /// @param s Arc length along path
    /// @return View of the segment containing s, with segment's arc length bounds
    ///
    segment::view operator()(arc_length s) const;

    ///
    /// Gets configuration at arc length.
    ///
    /// @param s Arc length along path
    /// @return Configuration vector at s
    ///
    xt::xarray<double> configuration(arc_length s) const;

    ///
    /// Gets tangent at arc length.
    ///
    /// @param s Arc length along path
    /// @return Unit tangent vector at s
    ///
    xt::xarray<double> tangent(arc_length s) const;

    ///
    /// Gets curvature at arc length.
    ///
    /// @param s Arc length along path
    /// @return Curvature vector at s
    ///
    xt::xarray<double> curvature(arc_length s) const;

    ///
    /// Cursor for efficient sequential traversal.
    ///
    class cursor;

    ///
    /// Creates a cursor at specified arc length position.
    ///
    /// Cursors provide efficient sequential traversal with O(1) amortized access via hints.
    /// Multiple cursors can exist on the same path (e.g., for forward/backward integration).
    ///
    /// @param s Starting position (default: 0, path start). Clamped to [0, length].
    /// @return Cursor positioned at s
    /// @throws std::invalid_argument if path is empty
    ///
    [[nodiscard]] cursor create_cursor(arc_length s = arc_length{0.0}) const;

   private:
    // Internal storage: segment with its starting position on the path
    struct positioned_segment {
        segment seg;
        arc_length start;
    };

    path(std::vector<positioned_segment> segments, size_t dof, arc_length length);

    std::vector<positioned_segment> segments_;
    size_t dof_{0};
    arc_length length_{0.0};

    friend class const_iterator;
    friend class cursor;
};

///
/// Forward iterator for path segments, yielding segment::view on dereference.
///
/// Dereferencing returns segment::view by value, not by reference.
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
///
class path::const_iterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = segment::view;
    using difference_type = std::ptrdiff_t;
    using pointer = const segment::view*;
    using reference = segment::view;

    ///
    /// Default constructs singular iterator.
    ///
    const_iterator() noexcept;

    ///
    /// Dereferences to get segment view.
    ///
    /// @return Segment view for current segment
    ///
    segment::view operator*() const;

    ///
    /// Pre-increments iterator.
    ///
    /// @return Reference to this
    ///
    const_iterator& operator++() noexcept;

    ///
    /// Post-increments iterator.
    ///
    /// @return Iterator to previous position
    ///
    const_iterator operator++(int) noexcept;

    ///
    /// Pre-decrements iterator.
    ///
    /// @return Reference to this
    ///
    const_iterator& operator--() noexcept;

    ///
    /// Post-decrements iterator.
    ///
    /// @return Iterator to previous position
    ///
    const_iterator operator--(int) noexcept;

    ///
    /// Compares iterators for equality.
    ///
    /// @param other Iterator to compare with
    /// @return True if iterators point to same position
    ///
    bool operator==(const const_iterator& other) const noexcept;

   private:
    friend class path;

    const_iterator(const path* p, std::vector<positioned_segment>::const_iterator it) noexcept : path_{p}, it_{it} {}

    const path* path_;
    std::vector<positioned_segment>::const_iterator it_;
};

///
/// Cursor for efficient sequential traversal of path with hint optimization.
///
/// Maintains current arc length position and segment hint for O(1) amortized
/// access during sequential traversal. Supports bidirectional traversal for
/// forward and backward integration in TOTG algorithm.
///
/// **Hint optimization**: Tracks last-accessed segment to avoid binary search
/// on sequential access. Provides O(1) amortized lookup vs O(log n) per query.
///
/// **Thread safety**: Not thread-safe. Each integration pass should use its own cursor.
///
/// **Semantics**: Cursor is a view-like object with internal state. Cursors are copyable
/// to support snapshots (e.g., for forward/backward integration passes).
///
/// Example usage:
/// @code
///   path p = path::create(waypoints);
///   path::cursor cursor = p.create_cursor();
///
///   // Forward integration
///   while (!cursor.at_end()) {
///       auto config = cursor.configuration();
///       auto tangent = cursor.tangent();
///       // ... process ...
///       cursor.advance_by(arc_length{0.01});
///   }
///
///   // Backward integration
///   cursor = p.create_cursor(p.length());  // Start at end
///   while (!cursor.at_start()) {
///       auto config = cursor.configuration();
///       cursor.advance_by(arc_length{-0.01});  // negative delta
///   }
/// @endcode
///
class path::cursor {
   public:
    ///
    /// Gets the path being traversed.
    ///
    /// @return Reference to the path
    ///
    const class path& path() const noexcept;

    ///
    /// Gets current arc length position.
    ///
    /// @return Current position along path
    ///
    arc_length position() const noexcept;

    ///
    /// Dereferences cursor to get segment view at current position.
    ///
    /// Returns a view of the segment containing the current cursor position,
    /// including the segment's arc length bounds on the path.
    ///
    /// @return View of segment at current position
    /// @note O(1) - uses cached hint
    ///
    segment::view operator*() const;

    ///
    /// Seeks to specific arc length position (absolute positioning).
    ///
    /// Sets cursor position to target arc length. Clamps to [0, infinity).
    /// If target exceeds path length, cursor is set to infinity (sentinel position).
    ///
    /// @param s Target arc length position
    /// @return Reference to this cursor for method chaining
    ///
    cursor& seek(arc_length s) noexcept;

    ///
    /// Seeks along path by arc length delta (relative positioning).
    ///
    /// Advances cursor by arc length offset. Clamps to [0, infinity).
    /// If result exceeds path length, cursor is set to infinity (sentinel position).
    /// Supports bidirectional traversal (positive = forward, negative = backward).
    ///
    /// @param delta Arc length offset
    /// @return Reference to this cursor for method chaining
    ///
    cursor& seek_by(arc_length delta) noexcept;

    ///
    /// Gets configuration at current position.
    ///
    /// @return Configuration vector
    /// @throws std::out_of_range if cursor is at sentinel position or before start
    ///
    xt::xarray<double> configuration() const;

    ///
    /// Gets tangent at current position.
    ///
    /// @return Unit tangent vector
    /// @throws std::out_of_range if cursor is at sentinel position or before start
    ///
    xt::xarray<double> tangent() const;

    ///
    /// Gets curvature at current position.
    ///
    /// @return Curvature vector
    /// @throws std::out_of_range if cursor is at sentinel position or before start
    ///
    xt::xarray<double> curvature() const;

    ///
    /// Gets sentinel for end-of-path comparison.
    ///
    /// Returns a sentinel value that compares equal to cursors positioned
    /// past the end of the path. Useful for detecting when iteration should stop.
    ///
    /// @return Sentinel value for comparison
    ///
    std::default_sentinel_t end() const noexcept;

    ///
    /// Compares cursor with end sentinel.
    ///
    /// @param c Cursor to compare
    /// @return True if cursor is at sentinel position (past end or invalid)
    ///
    friend bool operator==(const cursor& c, std::default_sentinel_t) noexcept;

    ///
    /// Compares end sentinel with cursor (reversed order).
    ///
    /// @param c Cursor to compare
    /// @return True if cursor is at sentinel position (past end or invalid)
    ///
    friend bool operator==(std::default_sentinel_t, const cursor& c) noexcept;

   private:
    friend class path;

    // Construct cursor at given position
    // @param p Path to traverse (must outlive cursor)
    // @param s Initial position
    explicit cursor(const class path* p, arc_length s);

    // Update hint to match current position
    // Called after position changes to maintain O(1) amortized access
    void update_hint_() noexcept;

    // Path being traversed
    const class path* path_;

    // Current position along path
    arc_length position_;

    // Hint: iterator to segment containing current position
    // Maintained by update_hint_() for O(1) amortized lookups
    // Invariant: If position_ is within [hint_->start, hint_->end), then hint_ points to correct segment
    path::const_iterator hint_;
};

///
/// ADL-findable end sentinel for path::cursor.
///
/// Returns a sentinel value that can be compared with cursors to detect
/// end-of-path. This free function enables ADL and provides an alternative
/// to the member function cursor.end().
///
/// @param c Cursor (unused, for ADL only)
/// @return Sentinel value for comparison
///
constexpr std::default_sentinel_t end(const path::cursor&) noexcept {
    return std::default_sentinel;
}

// Verify that path satisfies range concepts
static_assert(std::ranges::range<path>);
static_assert(std::ranges::sized_range<path>);
static_assert(std::ranges::bidirectional_range<path>);

}  // namespace viam::trajex::totg

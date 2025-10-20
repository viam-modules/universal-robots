#pragma once

#include <chrono>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/path.hpp>

namespace viam::trajex::totg {

/// Implementation details for trajectory types
namespace trajectory_details {

/// Generic sampler strategy concept
///
/// A sampler decides the next sample time given cursor state.
/// Parameterized to work with any cursor and sample types.
template <typename S, typename Cursor, typename Sample>
concept sampler = requires(S s, Cursor& c) {
    {
        s.next(c)
    } -> std::convertible_to<std::optional<Sample>>;
};

}  // namespace trajectory_details

/// Time-parameterized trajectory
///
/// Result of time-optimal path parameterization.
/// Provides queries for position, velocity, and acceleration at any time.
///
/// **Note**: Cannot be default-constructed. Must be created via trajectory::create().
class trajectory {
   public:
    /// Time duration in seconds (as floating point)
    using seconds = std::chrono::duration<double>;

    /// Sample from a trajectory
    struct sample {
        seconds time;                      ///< Sample time
        xt::xarray<double> configuration;  ///< Configuration at sample time
        xt::xarray<double> velocity;       ///< Velocity at sample time
        xt::xarray<double> acceleration;   ///< Acceleration at sample time
    };

    /// Options for trajectory generation via TOPP algorithm
    struct options {
        /// Maximum velocity per DOF (units match configuration space)
        xt::xarray<double> max_velocity;

        /// Maximum acceleration per DOF (units match configuration space)
        xt::xarray<double> max_acceleration;

        /// Default integration time step for phase plane integration
        /// Paper (Table I) recommends 0.001s (1ms) as good balance of accuracy vs speed
        static constexpr seconds k_default_delta{0.001};

        /// Default epsilon for numerical comparisons
        static constexpr double k_default_epsilon = 1e-6;

        /// Integration time step for phase plane integration
        /// Smaller values → more accurate but slower computation
        seconds delta{k_default_delta};

        /// Numerical comparison epsilon
        double epsilon{k_default_epsilon};
    };

    /// Integration point from phase plane TOPP algorithm
    ///
    /// Stores (time, arc_length, velocity, acceleration) from phase plane integration.
    /// Path geometry (q, q', q'') is queried on-demand during sampling for exact results.
    /// This is more accurate than storing and interpolating joint-space values, since
    /// the path knows exact circular blend geometry.
    struct integration_point {
        seconds time;   ///< Time at this integration point
        arc_length s;   ///< Arc length position on path
        double s_dot;   ///< Path velocity (ds/dt)
        double s_ddot;  ///< Path acceleration (d²s/dt²)
    };

    /// Collection of integration points from TOPP phase plane integration
    ///
    /// Represents the time-parameterization of a path as a sequence of
    /// (time, arc_length, velocity, acceleration) samples from forward/backward
    /// integration in the phase plane.
    using integration_points = std::vector<integration_point>;

    /// Create time-optimal trajectory from path
    ///
    /// Computes TOPP time-parameterization respecting velocity/acceleration limits.
    /// If test_points are provided, bypasses TOPP for testing with known integration data.
    ///
    /// @param p Path to time-parameterize (moved into trajectory)
    /// @param opt Trajectory generation options (moved into trajectory)
    /// @param test_points Optional integration points for testing (bypasses TOPP algorithm).
    ///                    If provided, must be sorted by time with first point at t=0.
    /// @return Time-parameterized trajectory
    /// @throws std::invalid_argument if options.max_velocity/acceleration DOF doesn't match path DOF,
    ///         if options.delta/epsilon are non-positive, or if test_points are invalid
    [[nodiscard]] static trajectory create(path p, options opt, integration_points test_points = {});

    /// Get total duration of trajectory
    /// @return Duration
    seconds duration() const noexcept;

    /// Get the path this trajectory was generated from
    /// @return Reference to the source path
    const class path& path() const noexcept;

    /// Get number of degrees of freedom
    /// @return Number of DOF
    size_t dof() const noexcept;

    /// Get integration points from TOPP phase plane integration
    /// @return Reference to integration points vector
    const integration_points& get_integration_points() const noexcept;

    /// Get options used to generate this trajectory
    /// @return Reference to trajectory generation options
    const struct options& get_options() const noexcept;

    /// Sample trajectory at given time
    /// @param t Time
    /// @return Sample containing time, position, velocity, and acceleration
    struct sample sample(seconds t) const;

    /// Cursor for efficient sequential trajectory sampling (defined after trajectory)
    class cursor;

    /// Range over trajectory samples using a sampler strategy
    ///
    /// Provides begin()/end() semantics for range-based iteration.
    /// Compatible with C++20 ranges and legacy algorithms.
    /// Implementation defined after trajectory::cursor.
    ///
    /// Example:
    /// @code
    ///   for (auto s : traj.samples(uniform_sampler{seconds{0.01}})) {
    ///       // Process sample
    ///   }
    /// @endcode
    template <typename S>
    class sampled;  // Forward declaration, defined after cursor

    /// Create a sampled range over this trajectory
    ///
    /// @tparam S Sampler type satisfying the trajectory_details::sampler concept
    /// @param s Sampler strategy
    /// @return Range over samples determined by the sampler
    template <trajectory_details::sampler<cursor, struct sample> S>
    [[nodiscard]] sampled<S> samples(S s) const;

    /// Create a cursor for manual sequential sampling
    /// @return Cursor initialized to trajectory start (t=0)
    [[nodiscard]] cursor create_cursor() const;

   private:
    // Private constructor - only create() can construct trajectories
    trajectory(class path p, struct options opt, integration_points points);

    class path path_;                        ///< Geometric path
    struct options options_;                 ///< Options used to generate this trajectory
    integration_points integration_points_;  ///< Time parameterization from TOPP integration
    seconds duration_{seconds{0.0}};         ///< Total trajectory duration
};

/// Cursor for efficient sequential trajectory sampling
///
/// Maintains position and dual hint state for O(1) amortized sequential access:
/// - Time hint: Iterator into samples_ vector for O(1) time lookups
/// - Path hint: Embedded path::cursor for O(1) path geometry queries
///
/// Sequential sampling (the common case) is O(1) amortized because both hints
/// follow along as the cursor advances through time.
///
/// Samplers control cursor advancement to implement different sampling strategies.
class trajectory::cursor {
   public:
    /// Get the trajectory this cursor is traversing
    /// @return Reference to parent trajectory
    const class trajectory& trajectory() const noexcept;

    /// Get current time position
    /// @return Current time along trajectory
    seconds time() const noexcept;

    /// Sample trajectory at current cursor position
    ///
    /// Interpolates (s, ṡ, s̈) from stored samples, then queries path for exact
    /// geometry at interpolated arc length. Uses dual hints for O(1) amortized access.
    ///
    /// @return Sample at current time
    /// @throws std::out_of_range if cursor is at sentinel position or before start
    struct trajectory::sample sample() const;

    /// Seek cursor to specific time (absolute positioning)
    ///
    /// Sets cursor position to target time. Clamps to [0, infinity).
    /// If target exceeds duration, cursor is set to infinity (sentinel position).
    ///
    /// @param t Absolute time to move to
    void seek(seconds t);

    /// Seek cursor by time delta (relative positioning)
    ///
    /// Advances cursor by time offset. Clamps to [0, infinity).
    /// If result exceeds duration, cursor is set to infinity (sentinel position).
    ///
    /// @param dt Time offset to move by
    void seek_by(seconds dt);

    /// Get sentinel for end-of-trajectory comparison
    ///
    /// Returns a sentinel value that compares equal to cursors positioned
    /// past the end of the trajectory. Useful for detecting when iteration
    /// should stop.
    ///
    /// @return Sentinel value for comparison
    std::default_sentinel_t end() const noexcept;

    /// Compare cursor with end sentinel
    /// @return true if cursor is at sentinel position (past end or invalid)
    friend bool operator==(const cursor& c, std::default_sentinel_t) noexcept;

    /// Compare end sentinel with cursor (reversed order)
    /// @return true if cursor is at sentinel position (past end or invalid)
    friend bool operator==(std::default_sentinel_t, const cursor& c) noexcept;

   private:
    friend class trajectory;
    explicit cursor(const class trajectory* traj);

    /// Helper to update path cursor position after time hint is updated
    void update_path_cursor_position_(seconds t);

    const class trajectory* traj_;
    seconds time_{seconds{0.0}};

    /// Hint for O(1) amortized time lookups in integration_points_ vector
    /// Points to the integration point at or before current time_
    /// Maintained by seek/seek_by to avoid repeated binary searches
    std::vector<integration_point>::const_iterator time_hint_;

    /// Path cursor for O(1) amortized path geometry queries
    /// Positioned at interpolated arc length corresponding to current time_
    /// Invariant: After seek(), path_cursor_ is at the s corresponding to time_
    path::cursor path_cursor_;
};

/// Implementation of trajectory::sampled range
template <typename S>
class trajectory::sampled {
   public:
    /// Iterator over trajectory samples
    class iterator;

    /// Begin iterator
    iterator begin();

    /// End sentinel
    std::default_sentinel_t end() const noexcept;

   private:
    friend class trajectory;
    sampled(cursor cursor, S sampler);

    cursor cursor_;
    S sampler_;
};

/// Iterator over trajectory samples
template <typename S>
class trajectory::sampled<S>::iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = struct trajectory::sample;
    using difference_type = std::ptrdiff_t;
    using pointer = const struct trajectory::sample*;
    using reference = const struct trajectory::sample&;

    /// Dereference: returns current sample
    const struct trajectory::sample& operator*() const noexcept;

    /// Arrow operator
    const struct trajectory::sample* operator->() const noexcept;

    /// Pre-increment: advance to next sample
    iterator& operator++();

    /// Post-increment
    iterator operator++(int);

    /// Equality comparison with sentinel
    bool operator==(std::default_sentinel_t) const noexcept;

    /// Equality comparison with other iterator
    bool operator==(const iterator& other) const noexcept;

   private:
    friend class sampled;
    iterator(cursor cursor, S* sampler);

    cursor cursor_;
    S* sampler_;
    std::optional<struct trajectory::sample> current_;
};

// Implementation of trajectory::samples() factory method

template <trajectory_details::sampler<trajectory::cursor, struct trajectory::sample> S>
trajectory::sampled<S> trajectory::samples(S s) const {
    return sampled<S>{create_cursor(), std::move(s)};
}

// Implementation of trajectory::sampled methods

template <typename S>
trajectory::sampled<S>::sampled(cursor cursor, S sampler) : cursor_{std::move(cursor)}, sampler_{std::move(sampler)} {}

template <typename S>
typename trajectory::sampled<S>::iterator trajectory::sampled<S>::begin() {
    return iterator{cursor_, &sampler_};  // Copy cursor for independent iteration
}

template <typename S>
std::default_sentinel_t trajectory::sampled<S>::end() const noexcept {
    return std::default_sentinel;
}

// Implementation of trajectory::sampled::iterator methods

template <typename S>
trajectory::sampled<S>::iterator::iterator(cursor cursor, S* sampler)
    : cursor_{std::move(cursor)}, sampler_{sampler}, current_{sampler_->next(cursor_)} {}

template <typename S>
const struct trajectory::sample& trajectory::sampled<S>::iterator::operator*() const noexcept {
    return *current_;
}

template <typename S>
const struct trajectory::sample* trajectory::sampled<S>::iterator::operator->() const noexcept {
    return &*current_;
}

template <typename S>
typename trajectory::sampled<S>::iterator& trajectory::sampled<S>::iterator::operator++() {
    current_ = sampler_->next(cursor_);
    return *this;
}

template <typename S>
typename trajectory::sampled<S>::iterator trajectory::sampled<S>::iterator::operator++(int) {
    iterator tmp = *this;
    ++(*this);
    return tmp;
}

template <typename S>
bool trajectory::sampled<S>::iterator::operator==(std::default_sentinel_t) const noexcept {
    return !current_.has_value();
}

template <typename S>
bool trajectory::sampled<S>::iterator::operator==(const iterator& other) const noexcept {
    // Two iterators are equal if they're both at end, or both have same time position
    if (!current_.has_value() && !other.current_.has_value()) {
        return true;
    }
    if (current_.has_value() && other.current_.has_value()) {
        return current_->time == other.current_->time;
    }
    return false;
}

/// ADL-findable end sentinel for trajectory::cursor
///
/// Returns a sentinel value that can be compared with cursors to detect
/// end-of-trajectory. This free function enables ADL and provides an
/// alternative to the member function cursor.end().
///
/// @param c Cursor (unused, for ADL only)
/// @return Sentinel value for comparison
constexpr std::default_sentinel_t end(const trajectory::cursor&) noexcept {
    return std::default_sentinel;
}

}  // namespace viam::trajex::totg

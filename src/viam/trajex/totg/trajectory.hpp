#pragma once

#include <chrono>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/arc_acceleration.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_velocity.hpp>
#include <viam/trajex/types/epsilon.hpp>

namespace viam::trajex::totg {

///
/// Implementation details for trajectory types.
///
namespace trajectory_details {

///
/// Generic sampler strategy concept.
///
/// A sampler decides the next sample time given cursor state.
/// Parameterized to work with any cursor and sample types.
///
template <typename S, typename Cursor, typename Sample>
concept sampler = requires(S s, Cursor& c) {
    { s.next(c) } -> std::convertible_to<std::optional<Sample>>;
};

}  // namespace trajectory_details

///
/// Time-parameterized trajectory.
///
/// Result of time-optimal path parameterization.
/// Provides queries for position, velocity, and acceleration at any time.
///
/// **Note**: Cannot be default-constructed. Must be created via trajectory::create().
///
class trajectory {
   public:
    ///
    /// Time duration in seconds (as floating point).
    ///
    using seconds = std::chrono::duration<double>;

    ///
    /// Sample from a trajectory.
    ///
    struct sample {
        seconds time;                      ///< Sample time
        xt::xarray<double> configuration;  ///< Configuration at sample time
        xt::xarray<double> velocity;       ///< Velocity at sample time
        xt::xarray<double> acceleration;   ///< Acceleration at sample time
    };

    // Forward declaration for observer interface.
    class integration_observer;

    // Forward declaration for event observer interface.
    class integration_event_observer;

    ///
    /// Options for trajectory generation via TOTG algorithm.
    ///
    struct options {
        ///
        /// Maximum velocity per DOF (units match configuration space).
        ///
        xt::xarray<double> max_velocity;

        ///
        /// Maximum acceleration per DOF (units match configuration space).
        ///
        xt::xarray<double> max_acceleration;

        ///
        /// Default integration time step for phase plane integration.
        ///
        /// Paper (Table I) recommends 0.001s (1ms) as good balance of accuracy vs speed.
        ///
        static constexpr seconds k_default_delta{0.001};

        ///
        /// Default epsilon for numerical comparisons.
        ///
        static constexpr class epsilon k_default_epsilon{1e-6};

        ///
        /// Integration time step for phase plane integration.
        ///
        /// Smaller values result in more accurate but slower computation.
        ///
        seconds delta{k_default_delta};

        ///
        /// Numerical comparison epsilon.
        ///
        class epsilon epsilon{k_default_epsilon};

        ///
        /// Observer for integration events (optional).
        ///
        /// Receives notifications about algorithm phases: forward/backward integration,
        /// limit curve hits, and trajectory extension. Enables testing and streaming.
        ///
        trajectory::integration_observer* observer = nullptr;
    };

    ///
    /// Classification of switching points in TOTG algorithm.
    ///
    /// Switching points mark locations where forward integration transitions
    /// to backward integration. Different types require different handling.
    ///
    enum class switching_point_kind : std::uint8_t {
        k_discontinuous_curvature,       ///< Acceleration case 1 (Eq 38): f''(s) discontinuous
        k_nondifferentiable_extremum,    ///< Acceleration case 2 (Eq 39): f'_i(s) = 0
        k_velocity_escape,               ///< Velocity case 1 (Eq 40): can escape below velocity curve
        k_discontinuous_velocity_limit,  ///< Equations 41-42: velocity limit discontinuous
        k_path_end,                      ///< Natural termination at path end
    };

    ///
    /// Position in phase plane (s, ṡ) space.
    ///
    /// Represents a point in the 2D phase plane used by TOTG algorithm.
    /// Arc length s on the horizontal axis, path velocity ṡ on the vertical axis.
    ///
    struct phase_point {
        arc_length s;        ///< Arc length position on path
        arc_velocity s_dot;  ///< Path velocity (ds/dt)
    };

    ///
    /// Kinematic state during integration (position, velocity, acceleration).
    ///
    /// Represents the full state during phase plane integration: position and
    /// velocity in the phase plane, plus path acceleration.
    ///
    /// @note: Does not inherit from `phase_point`, though it could, because it
    ///        frustrates use of designated initializers.
    ///
    struct phase_state {
        arc_length s;             ///< Arc length position on path
        arc_velocity s_dot;       ///< Path velocity (ds/dt)
        arc_acceleration s_ddot;  ///< Path acceleration (d^2s/dt^2)
    };

    ///
    /// Integration point from phase plane TOTG algorithm.
    ///
    /// Stores time and kinematic state (s, ṡ, s̈) from integration.
    /// Path geometry (q, q', q'') is queried on-demand during sampling for exact results.
    /// This is more accurate than storing and interpolating joint-space values, since
    /// the path knows exact circular blend geometry.
    ///
    struct integration_point {
        seconds time;             ///< Time at this integration point
        arc_length s;             ///< Arc length position on path
        arc_velocity s_dot;       ///< Path velocity (ds/dt)
        arc_acceleration s_ddot;  ///< Path acceleration (d^2s/dt^2)
    };

    ///
    /// Phase plane velocity limits at a given path position.
    ///
    /// Encapsulates the two types of velocity constraints from TOTG:
    /// acceleration-limited (centripetal) and velocity-limited (direct).
    /// See Kunz & Stilman equations 31 and 36.
    ///
    struct velocity_limits {
        arc_velocity s_dot_max_acc;  ///< Maximum velocity from acceleration constraints
        arc_velocity s_dot_max_vel;  ///< Maximum velocity from velocity constraints
    };

    ///
    /// Phase plane acceleration bounds at a given position and velocity.
    ///
    /// Encapsulates the feasible range of path acceleration given current path
    /// velocity and joint acceleration constraints.
    /// See Kunz & Stilman equations 22-23.
    ///
    struct acceleration_bounds {
        arc_acceleration s_ddot_min;  ///< Minimum feasible path acceleration
        arc_acceleration s_ddot_max;  ///< Maximum feasible path acceleration
    };

    ///
    /// Collection of integration points from TOTG phase plane integration.
    ///
    /// Represents the time-parameterization of a path as a sequence of
    /// (time, arc_length, velocity, acceleration) samples from forward/backward
    /// integration in the phase plane.
    ///
    using integration_points = std::vector<integration_point>;

    ///
    /// Creates time-optimal trajectory from path.
    ///
    /// Computes TOTG time-parameterization respecting velocity/acceleration limits.
    /// If test_points are provided, bypasses TOTG for testing with known integration data.
    ///
    /// @param p Path to time-parameterize (moved into trajectory)
    /// @param opt Trajectory generation options (moved into trajectory)
    /// @param points Optional integration points for testing (bypasses TOTG algorithm).
    ///               If provided, must be sorted by time with first point at t=0.
    /// @return Time-parameterized trajectory
    /// @throws std::invalid_argument if options.max_velocity/acceleration DOF doesn't match path DOF,
    ///         if options.delta/epsilon are non-positive, or if test_points are invalid
    ///
    [[nodiscard]] static trajectory create(path p, options opt, integration_points points = {});

    ///
    /// Gets total duration of trajectory.
    ///
    /// @return Duration
    ///
    seconds duration() const noexcept;

    ///
    /// Gets the path this trajectory was generated from.
    ///
    /// @return Reference to the source path
    ///
    const class path& path() const noexcept;

    ///
    /// Gets number of degrees of freedom.
    ///
    /// @return Number of DOF
    ///
    size_t dof() const noexcept;

    ///
    /// Gets integration points from TOTG phase plane integration.
    ///
    /// @return Reference to integration points vector
    ///
    const integration_points& get_integration_points() const noexcept;

    ///
    /// Gets options used to generate this trajectory.
    ///
    /// @return Reference to trajectory generation options
    ///
    const struct options& get_options() const noexcept;

    ///
    /// Computes phase plane velocity limits at cursor's current position.
    ///
    /// Uses path geometry from cursor and constraint limits from trajectory options
    /// to compute both acceleration-limited and velocity-limited constraints.
    /// See Kunz & Stilman equations 31 and 36.
    ///
    /// @param cursor Path cursor positioned at query location
    /// @return Velocity limits from acceleration and velocity constraints
    ///
    velocity_limits get_velocity_limits(const path::cursor& cursor) const;

    ///
    /// Computes phase plane acceleration bounds at cursor's current position and velocity.
    ///
    /// Uses path geometry from cursor, current velocity, and constraint limits from
    /// trajectory options to compute the feasible range of path acceleration.
    /// See Kunz & Stilman equations 22-23.
    ///
    /// @param cursor Path cursor positioned at query location
    /// @param s_dot Current path velocity
    /// @return Acceleration bounds from joint acceleration constraints
    /// @throws std::runtime_error if bounds are infeasible (shouldn't happen with valid trajectories)
    ///
    acceleration_bounds get_acceleration_bounds(const path::cursor& cursor, arc_velocity s_dot) const;

    ///
    /// Samples trajectory at given time.
    ///
    /// @param t Time
    /// @return Sample containing time, position, velocity, and acceleration
    ///
    struct sample sample(seconds t) const;

    ///
    /// Cursor for efficient sequential trajectory sampling.
    ///
    class cursor;

    ///
    /// Range over trajectory samples using a sampler strategy.
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
    ///
    template <typename S>
    class sampled;  // Forward declaration, defined after cursor

    ///
    /// Creates a sampled range over this trajectory.
    ///
    /// @tparam S Sampler type satisfying the trajectory_details::sampler concept
    /// @param s Sampler strategy
    /// @return Range over samples determined by the sampler
    ///
    template <trajectory_details::sampler<cursor, struct sample> S>
    [[nodiscard]] sampled<S> samples(S s) const;

    ///
    /// Creates a cursor for manual sequential sampling.
    ///
    /// @return Cursor initialized to trajectory start (t=0)
    ///
    [[nodiscard]] cursor create_cursor() const;

   private:
    // Private constructor for completed trajectories with validated integration points
    trajectory(class path p, struct options opt, integration_points points);

    // Private constructor for in-progress trajectory building (integration points added incrementally)
    trajectory(class path p, struct options opt);

    // Geometric path
    class path path_;

    // Options used to generate this trajectory
    struct options options_;

    // Time parameterization from TOTG integration
    integration_points integration_points_;

    // The path(s) not taken
    std::vector<integration_points> frosts_;

    // Total trajectory duration
    seconds duration_{seconds{0.0}};
};

///
/// Observer for high-level trajectory integration events.
///
/// Receives notifications about major algorithm phases and decisions during
/// TOTG trajectory generation. Useful for testing algorithm correctness.
///
class trajectory::integration_observer {
   protected:
    integration_observer();

   public:
    ///
    /// Destructor.
    ///
    virtual ~integration_observer();

    ///
    /// Event fired when forward integration starts or resumes.
    ///
    struct started_forward_event {
        phase_point start;  ///< Phase plane position where integration starts
    };

    ///
    /// Called when forward integration starts or resumes.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Event data containing starting position
    ///
    virtual void on_started_forward_integration(const trajectory& traj, started_forward_event event) = 0;

    ///
    /// Event fired when integration detects exceeding a limit curve.
    ///
    /// Contains the infeasible candidate point that would exceed limits,
    /// not a point on the limit curve itself.
    ///
    struct limit_hit_event {
        phase_point breach;          ///< Infeasible candidate point (exceeds limit)
        arc_velocity s_dot_max_acc;  ///< Maximum velocity from acceleration constraints
        arc_velocity s_dot_max_vel;  ///< Maximum velocity from velocity constraints
    };

    ///
    /// Called when integration detects that the next step would exceed a limit curve.
    ///
    /// The phase_point represents the INFEASIBLE candidate point that would exceed
    /// the limit, not a point on the limit curve itself. The algorithm will transition
    /// to curve following or switching point search to handle this condition.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Event data containing breach point and limit values
    ///
    virtual void on_hit_limit_curve(const trajectory& traj, limit_hit_event event) = 0;

    ///
    /// Event fired when backward integration starts from a switching point.
    ///
    struct started_backward_event {
        phase_point start;          ///< Phase plane position of switching point
        switching_point_kind kind;  ///< Classification of switching point
    };

    ///
    /// Called when backward integration starts from a switching point.
    ///
    /// @param traj Trajectory being integrated
    /// @param event Event data containing switching point position
    ///
    virtual void on_started_backward_integration(const trajectory& traj, started_backward_event event) = 0;

    ///
    /// Event fired when trajectory is extended with finalized integration points.
    ///
    /// Occurs after backward integration finds an intersection with the forward
    /// trajectory.
    ///
    /// Contains the pruned forward integration points that were replaced by
    /// backward-integrated points with lower (more conservative) velocities.
    ///
    struct splice_event {
        ///
        /// Range of integration points that were pruned from the forward trajectory.
        ///
        /// These points represent an over-optimistic forward integration that
        /// didn't account for deceleration to a switching point. They've been
        /// replaced by more conservative backward-integrated points.
        ///
        std::ranges::subrange<trajectory::integration_points::const_iterator> pruned;
    };

    ///
    /// Called when trajectory has been extended with finalized integration points.
    ///
    /// May be called multiple times during generation as the trajectory is incrementally
    /// built. Final call occurs when trajectory reaches path end (last s == path.length()).
    ///
    /// Enables streaming/incremental processing of finalized trajectory segments.
    ///
    /// The finalized phase plane position and duration can be obtained from
    /// the trajectory itself (last integration point and trajectory.duration()).
    ///
    /// @param traj Trajectory with finalized integration points up to current position
    /// @param event Event data containing pruned integration points
    ///
    virtual void on_trajectory_extended(const trajectory& traj, splice_event event) = 0;
};

///
/// A trajectory event observer which forwards all events to a single handler taking a variant. Useful if you
/// want to handle all events uniformly (e.g. storing them).
///
class trajectory::integration_event_observer : public trajectory::integration_observer {
   protected:
    integration_event_observer();

   public:
    ~integration_event_observer() override;

    using event = std::variant<started_forward_event, limit_hit_event, started_backward_event, splice_event>;

    void on_started_forward_integration(const trajectory& traj, started_forward_event event) final;

    void on_hit_limit_curve(const trajectory& traj, limit_hit_event event) final;

    void on_started_backward_integration(const trajectory& traj, started_backward_event event) final;

    void on_trajectory_extended(const trajectory& traj, splice_event event) final;

    ///
    /// Called for all integration events with event type as variant.
    ///
    /// Override this method to handle all event types uniformly. All specific
    /// event callbacks (on_started_forward_integration, on_hit_limit_curve, etc.)
    /// are marked final and forward to this method.
    ///
    /// @param traj Trajectory being integrated
    /// @param ev Event variant containing one of the event types
    ///
    virtual void on_event(const trajectory& traj, event ev) = 0;
};

///
/// Cursor for efficient sequential trajectory sampling.
///
/// Maintains position and dual hint state for O(1) amortized sequential access:
/// - Time hint: Iterator into samples_ vector for O(1) time lookups
/// - Path hint: Embedded path::cursor for O(1) path geometry queries
///
/// Sequential sampling (the common case) is O(1) amortized because both hints
/// follow along as the cursor advances through time.
///
/// Samplers control cursor advancement to implement different sampling strategies.
///
class trajectory::cursor {
   public:
    ///
    /// Gets the trajectory this cursor is traversing.
    ///
    /// @return Reference to parent trajectory
    ///
    const class trajectory& trajectory() const noexcept;

    ///
    /// Gets current time position.
    ///
    /// @return Current time along trajectory
    ///
    seconds time() const noexcept;

    ///
    /// Samples trajectory at current cursor position.
    ///
    /// Interpolates (s, s_dot, s_ddot) from stored samples, then queries path for exact
    /// geometry at interpolated arc length. Uses dual hints for O(1) amortized access.
    ///
    /// @return Sample at current time
    /// @throws std::out_of_range if cursor is at sentinel position or before start
    ///
    struct trajectory::sample sample() const;

    ///
    /// Seeks cursor to specific time (absolute positioning).
    ///
    /// Sets cursor position to target time. Clamps to [0, infinity).
    /// If target exceeds duration, cursor is set to infinity (sentinel position).
    ///
    /// @param t Absolute time to move to
    /// @return Reference to this cursor for method chaining
    ///
    cursor& seek(seconds t);

    ///
    /// Seeks cursor by time delta (relative positioning).
    ///
    /// Advances cursor by time offset. Clamps to [0, infinity).
    /// If result exceeds duration, cursor is set to infinity (sentinel position).
    ///
    /// @param dt Time offset to move by
    /// @return Reference to this cursor for method chaining
    ///
    cursor& seek_by(seconds dt);

    ///
    /// Gets sentinel for end-of-trajectory comparison.
    ///
    /// Returns a sentinel value that compares equal to cursors positioned
    /// past the end of the trajectory. Useful for detecting when iteration
    /// should stop.
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
    friend class trajectory;
    explicit cursor(const class trajectory* traj);

    // Helper to update path cursor position after time hint is updated
    void update_path_cursor_position_(seconds t);

    const class trajectory* traj_;
    seconds time_{seconds{0.0}};

    // Hint for O(1) amortized time lookups in integration_points_ vector
    // Points to the integration point at or before current time_
    // Maintained by seek/seek_by to avoid repeated binary searches
    std::vector<integration_point>::const_iterator time_hint_;

    // Path cursor for O(1) amortized path geometry queries
    // Positioned at interpolated arc length corresponding to current time_
    // Invariant: After seek(), path_cursor_ is at the s corresponding to time_
    path::cursor path_cursor_;
};

///
/// Range over trajectory samples using a sampler strategy.
///
template <typename S>
class trajectory::sampled : public std::ranges::view_interface<trajectory::sampled<S>> {
   public:
    ///
    /// Iterator over trajectory samples.
    ///
    class iterator;

    ///
    /// Gets begin iterator.
    ///
    /// @return Iterator to first sample
    ///
    iterator begin();

    ///
    /// Gets end sentinel.
    ///
    /// @return Sentinel past last sample
    ///
    std::default_sentinel_t end() const noexcept;

   private:
    friend class trajectory;
    sampled(cursor cursor, S sampler);

    cursor cursor_;
    S sampler_;
};

///
/// Iterator over trajectory samples.
///
template <typename S>
class trajectory::sampled<S>::iterator {
   public:
    // NOTE: Potentially, the sampler could describe its iterator tag and we would just
    // forward that here. For now, the only sampler we have has input semantics.
    using iterator_category = std::input_iterator_tag;
    using value_type = struct trajectory::sample;
    using difference_type = std::ptrdiff_t;
    using pointer = const struct trajectory::sample*;
    using reference = const struct trajectory::sample&;

    ///
    /// Dereferences to get current sample.
    ///
    /// @return Current sample
    ///
    const struct trajectory::sample& operator*() const noexcept;

    ///
    /// Arrow operator for member access.
    ///
    /// @return Pointer to current sample
    ///
    const struct trajectory::sample* operator->() const noexcept;

    ///
    /// Pre-increments to next sample.
    ///
    /// @return Reference to this
    ///
    iterator& operator++();

    ///
    /// Post-increments to next sample.
    ///
    /// @return Iterator to previous position
    ///
    iterator operator++(int);

    ///
    /// Compares with end sentinel.
    ///
    /// @return True if at end
    ///
    bool operator==(std::default_sentinel_t) const noexcept;

    ///
    /// Compares with other iterator.
    ///
    /// @param other Iterator to compare with
    /// @return True if iterators are equal
    ///
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

///
/// ADL-findable end sentinel for trajectory::cursor.
///
/// Returns a sentinel value that can be compared with cursors to detect
/// end-of-trajectory. This free function enables ADL and provides an
/// alternative to the member function cursor.end().
///
/// @param c Cursor (unused, for ADL only)
/// @return Sentinel value for comparison
///
constexpr std::default_sentinel_t end(const trajectory::cursor&) noexcept {
    return std::default_sentinel;
}

}  // namespace viam::trajex::totg

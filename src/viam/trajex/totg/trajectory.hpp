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

        /// Default integration step size in arc length (configuration space distance units)
        static constexpr double k_default_delta = 0.001;

        /// Default epsilon for numerical comparisons
        static constexpr double k_default_epsilon = 1e-6;

        /// Integration step size in arc length (configuration space distance units)
        double delta{k_default_delta};

        /// Numerical comparison epsilon
        double epsilon{k_default_epsilon};
    };

    /// Create time-optimal trajectory from path
    ///
    /// Computes TOPP time-parameterization respecting velocity/acceleration limits.
    ///
    /// @param p Path to time-parameterize (moved into trajectory)
    /// @param opt Trajectory generation options
    /// @return Time-parameterized trajectory
    /// @throws std::invalid_argument if options.max_velocity/acceleration DOF doesn't match path DOF,
    ///         or if options.delta/epsilon are non-positive
    [[nodiscard]] static trajectory create(path p, const options& opt);

    /// Get total duration of trajectory
    /// @return Duration
    seconds duration() const noexcept;

    /// Get the path this trajectory was generated from
    /// @return Reference to the source path
    const class path& path() const noexcept;

    /// Get number of degrees of freedom
    /// @return Number of DOF
    size_t dof() const noexcept;

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
    /// @return Cursor initialized to start of trajectory
    [[nodiscard]] cursor create_cursor() const;

   private:
    // Private constructor - only create() can construct trajectories
    explicit trajectory(class path p);

    // TODO(acm): Store time parameterization
    // - Could be sampled points with interpolation
    // - Or piecewise polynomial representation
    // - Need to support queries for position/velocity/acceleration at time t

    class path path_;
    seconds duration_{seconds{0.0}};
};

/// Cursor for efficient sequential trajectory sampling
///
/// Maintains position and hint state for O(1) amortized sequential access.
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
    /// @return Sample at current time
    struct trajectory::sample sample() const;

    /// Advance cursor to specific time
    /// @param t Absolute time to move to
    void advance_to(seconds t);

    /// Advance cursor by time delta
    /// @param dt Time offset to advance by
    void advance_by(seconds dt);

   private:
    friend class trajectory;
    explicit cursor(const class trajectory* traj);

    const class trajectory* traj_;
    seconds time_{seconds{0.0}};
    // TODO(acm): Add hints for time parameterization lookup
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

}  // namespace viam::trajex::totg

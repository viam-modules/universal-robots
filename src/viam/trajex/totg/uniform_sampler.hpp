#pragma once

#include <optional>

#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/types/hertz.hpp>

namespace viam::trajex::totg {

///
/// Uniform time-step sampler.
///
/// Samples trajectory at regular time intervals.
/// Returns samples at t=0, dt, 2*dt, ... until trajectory duration is exceeded.
///
class uniform_sampler {
   public:
    ///
    /// Constructs uniform sampler with given time step.
    ///
    /// @param dt Time interval between samples
    ///
    explicit uniform_sampler(trajectory::seconds dt);

    ///
    /// Creates uniform sampler with adjusted timestep to hit duration endpoint exactly.
    ///
    /// Calculates number of samples needed for the given frequency, rounds up to ensure
    /// endpoint coverage, then adjusts dt to land exactly on duration.
    /// Slightly oversamples to guarantee endpoint hit.
    ///
    /// @param duration Duration to quantize over
    /// @param frequency Desired sampling frequency
    /// @return uniform_sampler with dt adjusted to align with duration
    ///
    static uniform_sampler quantized_for_duration(trajectory::seconds duration, types::hertz frequency);

    ///
    /// Creates uniform sampler with adjusted timestep to hit trajectory endpoint exactly.
    ///
    /// Convenience wrapper that extracts duration from trajectory.
    ///
    /// @param traj Trajectory to sample (used for duration)
    /// @param frequency Desired sampling frequency
    /// @return uniform_sampler with dt adjusted to align with trajectory duration
    ///
    static uniform_sampler quantized_for_trajectory(const trajectory& traj, types::hertz frequency);

    ///
    /// Calculates adjusted timestep for quantized sampling.
    ///
    /// Exposes the core quantization calculation for testing and custom use.
    ///
    /// @param duration_sec Duration in seconds
    /// @param frequency_hz Frequency in Hz
    /// @return Adjusted timestep in seconds
    /// @throws std::invalid_argument if values are non-positive or exceed limits
    ///
    static double calculate_quantized_dt(double duration_sec, double frequency_hz);

    ///
    /// Gets next sample, advancing cursor by dt.
    ///
    /// @param cursor Cursor to sample and advance
    /// @return Sample at current cursor time, or nullopt if past trajectory end
    ///
    std::optional<struct trajectory::sample> next(trajectory::cursor& cursor);

   private:
    trajectory::seconds dt_;
};

}  // namespace viam::trajex::totg

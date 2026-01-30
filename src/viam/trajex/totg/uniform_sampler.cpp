#include <viam/trajex/totg/uniform_sampler.hpp>

#include <cmath>
#include <stdexcept>

#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

uniform_sampler::uniform_sampler(std::size_t num_samples) : num_samples_{num_samples} {}

std::size_t uniform_sampler::calculate_quantized_samples(double duration_sec, double frequency_hz) {
    if (duration_sec <= 0.0 || frequency_hz <= 0.0) {
        throw std::invalid_argument{"duration and frequency must both be positive"};
    }

    const double putative_samples = duration_sec * frequency_hz;

    static constexpr std::size_t k_max_samples = 1000000;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument{"duration and frequency exceed maximum allowable samples"};
    }

    // Round up to ensure we slightly oversample, then add 1 to get at
    // least 2 samples (start and end).
    return static_cast<std::size_t>(std::ceil(putative_samples) + 1);
}

double uniform_sampler::calculate_quantized_dt(double duration_sec, double frequency_hz) {
    const auto num_samples = calculate_quantized_samples(duration_sec, frequency_hz);

    // Recalculate dt to divide the duration evenly
    return duration_sec / static_cast<double>(num_samples - 1);
}

uniform_sampler uniform_sampler::quantized_for_duration(trajectory::seconds duration, types::hertz frequency) {
    return uniform_sampler{calculate_quantized_samples(duration.count(), frequency.value)};
}

uniform_sampler uniform_sampler::quantized_for_trajectory(const trajectory& traj, types::hertz frequency) {
    return quantized_for_duration(traj.duration(), frequency);
}

std::optional<struct trajectory::sample> uniform_sampler::next(trajectory::cursor& cursor) {
    if (next_sample_ == num_samples_) {
        return std::nullopt;
    }

    // Compute target time via `linspace` with a special case for the exact endpoint.
    const auto when = (next_sample_ == num_samples_ - 1) ? 1.0 : static_cast<double>(next_sample_) / static_cast<double>(num_samples_ - 1);

    // Seek cursor to computed time and sample
    cursor.seek(trajectory::seconds{std::lerp(0.0, cursor.trajectory().duration().count(), when)});
    auto sample = cursor.sample();

    ++next_sample_;
    return sample;
}

}  // namespace viam::trajex::totg

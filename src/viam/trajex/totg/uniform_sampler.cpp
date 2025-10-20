#include <viam/trajex/totg/uniform_sampler.hpp>

#include <cmath>
#include <stdexcept>

namespace viam::trajex::totg {

uniform_sampler::uniform_sampler(trajectory::seconds dt) : dt_{dt} {}

double uniform_sampler::calculate_quantized_dt(double duration_sec, double frequency_hz) {
    if (duration_sec <= 0.0 || frequency_hz <= 0.0) {
        throw std::invalid_argument{"duration and frequency must both be positive"};
    }

    // Calculate putative number of samples
    const double putative_samples = duration_sec * frequency_hz;

    // Check for reasonable bounds (similar to sampling_func)
    static constexpr std::size_t k_max_samples = 1000000;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument{"duration and frequency exceed maximum allowable samples"};
    }

    // Round up and add 1 to ensure at least 2 samples and endpoint coverage
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Recalculate dt to land exactly on duration
    return duration_sec / static_cast<double>(num_samples - 1);
}

uniform_sampler uniform_sampler::quantized_for_duration(trajectory::seconds duration, types::hertz frequency) {
    const double adjusted_dt_sec = calculate_quantized_dt(duration.count(), frequency.value);
    return uniform_sampler{trajectory::seconds{adjusted_dt_sec}};
}

uniform_sampler uniform_sampler::quantized_for_trajectory(const trajectory& traj, types::hertz frequency) {
    return quantized_for_duration(traj.duration(), frequency);
}

std::optional<struct trajectory::sample> uniform_sampler::next(trajectory::cursor& cursor) {
    // Check if cursor is at end sentinel
    if (cursor == cursor.end()) {
        return std::nullopt;
    }

    // Sample at current position (may be exactly at endpoint)
    auto sample = cursor.sample();

    // Advance cursor for next iteration (may go past duration to sentinel)
    cursor.seek_by(dt_);

    return sample;
}

}  // namespace viam::trajex::totg

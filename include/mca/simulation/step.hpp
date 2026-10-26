#pragma once

#include <cmath>
#include <cstddef>
#include <limits>

namespace mca::simulation {

// Deterministic, simulator-independent description of one fixed-step update.
// time_s is the time at the start of the step; dt_s is the step duration.
struct SimulationStep {
  double time_s{0.0};
  double dt_s{0.0};
  std::size_t step_index{0};

  bool valid() const noexcept {
    return std::isfinite(time_s) && std::isfinite(dt_s) && time_s >= 0.0 &&
           dt_s > 0.0;
  }
};

// Small deterministic clock used by simulation adapters and experiments.
// Advancing the clock is transactional: invalid or overflowing candidates do
// not mutate the current time/index.
class SimulationClock {
 public:
  bool configure(double dt_s) noexcept {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) return false;
    dt_s_ = dt_s;
    configured_ = true;
    return true;
  }

  bool configured() const noexcept { return configured_; }
  double dt() const noexcept { return dt_s_; }
  double time() const noexcept { return time_s_; }
  std::size_t step_index() const noexcept { return step_index_; }

  SimulationStep current_step() const noexcept {
    return {time_s_, dt_s_, step_index_};
  }

  bool reset() noexcept {
    if (!configured_) return false;
    time_s_ = 0.0;
    step_index_ = 0;
    return true;
  }

  bool advance() noexcept {
    if (!configured_) return false;
    const double next_time = time_s_ + dt_s_;
    if (!std::isfinite(next_time) || step_index_ == std::numeric_limits<std::size_t>::max()) {
      return false;
    }
    time_s_ = next_time;
    ++step_index_;
    return true;
  }

 private:
  double dt_s_{0.0};
  double time_s_{0.0};
  std::size_t step_index_{0};
  bool configured_{false};
};

}  // namespace mca::simulation

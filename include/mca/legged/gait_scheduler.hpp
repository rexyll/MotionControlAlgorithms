#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace mca::legged {

struct GaitSchedulerParameters {
  double period_s{0.5};
  double duty_factor{0.6};
  // Per-leg phase offsets in cycles. Offsets need not be pre-normalized.
  std::vector<double> phase_offsets{0.0, 0.5, 0.5, 0.0};
};

struct LegGaitPhase {
  double cycle_phase{0.0};       // [0, 1)
  double normalized_phase{0.0};  // [0, 1) within stance or swing
  bool in_stance{true};
};

struct GaitSchedulerOutput {
  std::vector<LegGaitPhase> legs{};
};

// Deterministic periodic gait clock. The caller defines leg ordering and phase
// offsets (e.g. diagonal pairs for trot or alternating pairs for walk).
inline bool evaluate_gait(const GaitSchedulerParameters& parameters,
                           double time_s,
                           GaitSchedulerOutput& output,
                           std::string* error = nullptr) {
  const auto fail = [error](const char* message) {
    if (error) *error = message;
    return false;
  };
  if (!std::isfinite(parameters.period_s) || parameters.period_s <= 0.0 ||
      !std::isfinite(parameters.duty_factor) ||
      parameters.duty_factor <= 0.0 || parameters.duty_factor >= 1.0 ||
      !std::isfinite(time_s)) {
    return fail("gait time/period must be finite and period positive; duty factor must be in (0, 1)");
  }
  if (parameters.phase_offsets.empty())
    return fail("at least one leg phase offset is required");
  for (double offset : parameters.phase_offsets) {
    if (!std::isfinite(offset))
      return fail("leg phase offsets must be finite");
  }

  const double cycle = time_s / parameters.period_s;
  if (!std::isfinite(cycle))
    return fail("normalized gait time is non-finite");
  GaitSchedulerOutput candidate;
  candidate.legs.reserve(parameters.phase_offsets.size());
  for (double offset : parameters.phase_offsets) {
    double phase = std::fmod(cycle + offset, 1.0);
    if (!std::isfinite(phase))
      return fail("computed gait phase is non-finite");
    if (phase < 0.0) phase += 1.0;
    // Guard round-off at the upper edge of the half-open cycle interval.
    if (phase >= 1.0) phase = 0.0;
    LegGaitPhase leg;
    leg.cycle_phase = phase;
    leg.in_stance = phase < parameters.duty_factor;
    leg.normalized_phase = leg.in_stance
        ? phase / parameters.duty_factor
        : (phase - parameters.duty_factor) / (1.0 - parameters.duty_factor);
    candidate.legs.push_back(leg);
  }
  output = std::move(candidate);
  return true;
}

}  // namespace mca::legged

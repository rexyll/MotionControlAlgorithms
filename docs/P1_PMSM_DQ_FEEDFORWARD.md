# P1-09 PMSM dq Decoupling / Feedforward

## Scope

P1-09 adds a small, ROS2/Gazebo-independent voltage feedforward component for the PMSM dq current loop. It uses the same rotor-frame convention as `P1-07 PmsmDq` and `P1-06 Clarke/Park`.

The component computes the model-based terms that compensate the dq current equations before the PI output is applied:

```text
vd_ff = -omega_e * Lq * iq
vq_ff =  omega_e * (Ld * id + psi_f)
```

A typical current controller composition is:

```text
vd = vd_pi + vd_ff
vq = vq_pi + vq_ff
```

where the PI controller remains responsible for tracking current error and its own anti-windup behavior.

## API

Header: `include/mca/control/pmsm_dq_feedforward.hpp`

- `PmsmDqFeedforwardConfig`
  - `d_axis_inductance_h`
  - `q_axis_inductance_h`
  - `permanent_magnet_flux_wb`
  - independent d/q feedforward voltage limits
- `PmsmDqFeedforwardOutput`
  - `d_voltage`
  - `q_voltage`
- `PmsmDqFeedforward::compute(omega_e, id, iq, output)`

`omega_e` is electrical angular velocity in rad/s. The component does not convert mechanical speed or pole-pair count; callers should use the same electrical-speed definition as the PMSM dq model.

## Sign convention

The signs follow the P1-07 equations:

```text
vd = Rs*id + Ld*did/dt - omega_e*Lq*iq
vq = Rs*iq + Lq*diq/dt + omega_e*(Ld*id + psi_f)
```

Therefore the feedforward terms are the opposite of the cross-coupling and PM back-EMF terms. Reversing electrical speed reverses both feedforward outputs.

## Limits and responsibility

The component independently clamps its feedforward contribution to configured d/q voltage limits. In a complete FOC implementation, the PI output and feedforward contribution should be composed with an explicit total-voltage limit consistent with the inverter voltage capability. This component deliberately does not modify `FocCurrentController`, so the two pieces remain independently testable and reusable.

## Validation

Unit tests cover:

- invalid inductance configuration and non-finite runtime input;
- analytical d/q feedforward reference values;
- zero-speed behavior;
- negative-speed sign reversal;
- independent d/q voltage limiting.

A Google Benchmark target `mca_pmsm_dq_feedforward_benchmark` measures the hot-path `compute()` operation.

## Acceptance

P1-09 is accepted only after Linux and Windows CI both pass, the analytical/sign/limit tests pass, the benchmark target is registered, and this document is present.

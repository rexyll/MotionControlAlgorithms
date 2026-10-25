# Motor Power-Diagnostic API Contract Matrix

This document is the cross-model contract for the status-returning power diagnostics in `mca::plant::DcMotor`, `mca::plant::BldcPmsm`, and `mca::plant::PmsmDq`. The public declarations are in `include/mca/plant/*.hpp`; regression coverage is in the corresponding `tests/plant/*_test.cpp` files.

## Shared API contract

All listed methods are `const noexcept` diagnostic queries with a caller-owned `double& output`. They do not advance or otherwise mutate the plant's runtime state. They commit `output` only after all applicable checks succeed; on any returned error, the caller's previous output value is preserved.

Status precedence and meaning:

| Condition | Status |
|---|---|
| The model has not been configured | `NotConfigured` |
| A supplied voltage argument is NaN or infinite | `InvalidArgument` |
| An intermediate product, sum, square, scaling operation, derived torque, or final result is non-finite | `NumericalError` |
| All applicable checks succeed | success (`Status::ok()`) |

For the methods in this matrix, configuration is checked before supplied voltage arguments. `copper_loss(output)` and `mechanical_power(output)` have no explicit numeric input arguments, so their failures are configuration or derived-arithmetic failures. All arithmetic checks are intended to contain finite-input overflow rather than silently publish infinity or NaN.

## Cross-model formula and unit matrix

| Diagnostic | `DcMotor` | `BldcPmsm` | `PmsmDq` |
|---|---|---|---|
| Electrical input power | `electrical_power(V, out)`: (P_e=Vi) | `electrical_power(va,vb,vc,out)`: (P_e=v_ai_a+v_bi_b+v_ci_c) | `electrical_power(vd,vq,out)`: (P_e=\frac{3}{2}(v_di_d+v_qi_q)) |
| Winding copper loss | `copper_loss(out)`: (P_{cu}=Ri^2) | `copper_loss(out)`: (P_{cu}=R_s(i_a^2+i_b^2+i_c^2)) | `copper_loss(out)`: (P_{cu}=\frac{3}{2}R_s(i_d^2+i_q^2)) |
| Mechanical shaft power | `mechanical_power(out)`: (P_m=T_e\omega_m), (T_e=K_t i) | `mechanical_power(out)`: (P_m=T_e\omega_m), using checked phase-domain electromagnetic torque | `mechanical_power(out)`: (P_m=T_e\omega_m), using checked dq electromagnetic torque |

All power outputs are in watts (W). Voltages are volts (V), currents amperes (A), resistance ohms (Ω), torque newton-metres (N·m), and mechanical angular speed radians per second (rad/s). The dq formulas use the model's amplitude-invariant transform convention and therefore include the factor (3/2). The three-phase model uses instantaneous phase-domain quantities and does not apply an additional (3/2) scale factor.

## Sign and interpretation

- Electrical input power is positive when net electrical power enters the modeled motor at the supplied terminals; negative values indicate instantaneous power returned toward the source under this sign convention.
- Mechanical shaft power uses (T_e\omega_m). For a fixed torque, reversing mechanical speed reverses the sign; zero speed gives zero shaft power when the product is finite.
- Copper loss is based on squared currents and positive configured resistance, so it is nonnegative and does not change sign when current polarity is reversed.
- These three quantities are independent instantaneous diagnostics. They are **not** a complete transient energy-balance API: equality between electrical input power and shaft power is not guaranteed during transients. The models do not include inverter/switching losses, and the diagnostic formulas do not account for all stored-energy changes or other loss mechanisms.

## Method-level status matrix

| Method family | `NotConfigured` | `InvalidArgument` | `NumericalError` |
|---|---|---|---|
| `electrical_power(..., output)` | Before configuration | Any supplied voltage is non-finite | Any product, accumulation/sum, scaling, or final result is non-finite |
| `copper_loss(output)` | Before configuration | Not applicable; no caller numeric input | Any current square, sum, resistance product, scaling, or final result is non-finite |
| `mechanical_power(output)` | Before configuration | Not applicable; no caller numeric input | Checked electromagnetic-torque derivation or torque-speed product is non-finite |

The three-phase and dq mechanical-power methods propagate a failure from their checked electromagnetic-torque query; the DC method explicitly checks its derived torque before multiplying by mechanical speed. The caller output is written only after the complete calculation succeeds in all three models.

## Declaration / implementation / regression cross-check

The following source-level cross-check identifies the public method declarations/definitions and their regression suites:

| Model | Public implementation | Regression suite and relevant coverage |
|---|---|---|
| DC | `include/mca/plant/dc_motor.hpp` | `tests/plant/dc_motor_test.cpp`: checked electrical/mechanical power and copper-loss status, nominal outputs, invalid/unconfigured paths, overflow/output preservation, and cross-model power consistency/state-preservation tests |
| Three-phase BLDC/PMSM | `include/mca/plant/bldc_pmsm.hpp` | `tests/plant/bldc_pmsm_test.cpp`: checked electrical/mechanical power and copper-loss status, nominal outputs, invalid/unconfigured paths, overflow/output preservation, and three-phase cross-model power consistency |
| PMSM dq | `include/mca/plant/pmsm_dq.hpp` | `tests/plant/pmsm_dq_test.cpp`: checked electrical/mechanical power and copper-loss status, nominal outputs, invalid/unconfigured paths, overflow/output preservation, and dq power consistency |

Cross-model tests also exercise zero/reversed-speed shaft power, terminal-power/copper-loss consistency, finite boundary inputs, and output/runtime-state preservation on failure. The matrix describes instantaneous values at the model's current stored state; the supplied voltage arguments affect only the electrical input-power query and are not applied to the model or integrated.

## Maintenance rule

When changing one of these public diagnostics, update this matrix and the relevant analytic/error-path regression tests in the same change. Do not weaken finite-arithmetic checks merely to accommodate an overflowing test input; keep test inputs representable through all documented conversions (including pole-pair scaling).

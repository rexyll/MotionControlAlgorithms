# CBF Multi-Step Closed-Loop Regression

P8-20 adds a deterministic 12-cycle closed-loop regression for the four discrete-time CBF filter variants:

- nominal discrete-time CBF;
- robust discrete-time CBF with bounded additive prediction error;
- discrete-time CBF with actuator slew-rate limits;
- robust discrete-time CBF with both uncertainty and slew-rate limits.

## Regression invariants

Each cycle feeds the applied control back as the previous actuator command and updates a scalar barrier state. The regression verifies:

1. the filter returns a successful status on the feasible trajectory;
2. controls remain inside configured actuator bounds;
3. slew-limited variants respect the configured per-cycle control delta;
4. robust variants preserve the worst-case barrier margin under the configured prediction error;
5. the barrier remains positive and satisfies the configured discrete-time decay condition;
6. the filter reports nominal-command modification when the safety/rate projection is active;
7. the robust+slew trace is deterministic when the same sequence is executed twice.

The test uses the public umbrella header and therefore also protects the public API wiring of all four variants.

## Numerical scope

The regression is intentionally deterministic and uses a scalar one-input closed-loop model so that failures identify filter semantics rather than plant-model behavior. It complements the single-step numerical-boundary tests from P8-19; it is not a replacement for application-specific closed-loop validation.

A production application should select the prediction-error bound, barrier decay factor, actuator limits and slew limits from measured/model uncertainty and hardware constraints rather than copying the regression constants.

## Acceptance

P8-20 is accepted only when the regression passes on both Linux and Windows in GitHub Actions with Configure, Build and Test successful.

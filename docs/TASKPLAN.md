# MotionControlAlgorithms Task Plan

## P8 — Advanced optimization/control

- [x] P8-37 nonlinear MPC line-search step underflow handling (stop backtracking when repeated halving underflows the step to zero, since subsequent candidates cannot differ from the current controls; add a denormal-step regression that checks termination reason, cost/control preservation and evaluation count; Linux/Windows Configure/Build/Test passed: Run #711)
- [x] P8-38 nonlinear MPC no-op line-search candidate detection (detect when finite step-gradient multiplication or bound projection leaves every control unchanged; avoid redundant rollout and count only actual candidate evaluations; add subnormal-step regression and document evaluation-count semantics; Linux/Windows Configure/Build/Test passed: Run #715)
- [x] P8-39 nonlinear MPC callback exception safety (convert exceptions from user-supplied dynamics/cost callbacks into transactional solver failure; preserve the caller's existing result on exception; add standard and non-standard exception regression coverage; document callback exception contract; Linux/Windows Configure/Build/Test passed: Run #723)
- [ ] P8-40 nonlinear MPC projected-gradient mapping underflow safety (avoid reporting a zero projected-gradient residual solely because `gradient_step * gradient` underflows to zero; preserve correct first-order stationarity diagnostics for interior and active-bound controls; add tiny-gradient and active-bound regressions; verify Linux/Windows Configure/Build/Test)

Acceptance for P8-39: user callback exceptions from dynamics, stage cost, and terminal cost are converted to solver failure; both standard and non-standard exception paths are covered; the caller's existing result is preserved; Linux and Windows Configure/Build/Test plus dedicated exception regressions pass.

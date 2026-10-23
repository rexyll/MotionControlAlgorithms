# MotionControlAlgorithms Task Plan

## P8 — Advanced optimization/control

- [x] P8-37 nonlinear MPC line-search step underflow handling (stop backtracking when repeated halving underflows the step to zero, since subsequent candidates cannot differ from the current controls; add a denormal-step regression that checks termination reason, cost/control preservation and evaluation count; Linux/Windows Configure/Build/Test passed: Run #711)
- [x] P8-38 nonlinear MPC no-op line-search candidate detection (detect when finite step-gradient multiplication or bound projection leaves every control unchanged; avoid redundant rollout and count only actual candidate evaluations; add subnormal-step regression and document evaluation-count semantics; Linux/Windows Configure/Build/Test passed: Run #715)
- [ ] P8-39 nonlinear MPC callback exception safety (convert exceptions from user-supplied dynamics/cost callbacks into transactional solver failure; preserve the caller's existing result on exception; add standard and non-standard exception regression coverage; document callback exception contract; verify Linux/Windows Configure/Build/Test)

Acceptance for P8-38: Linux and Windows Configure/Build/Test pass; no-op line-search candidates do not trigger redundant rollout and work counters count only actual candidate evaluations.

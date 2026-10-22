# P8-02 Discrete-time LQR baseline

This header-only baseline solves the discrete algebraic Riccati equation for
x[k+1] = A*x[k] + B*u[k] and stage cost x'Qx + u'Ru. It returns the feedback
law u[k] = -K*x[k], the Riccati matrix P, iteration count, and update residual.

The solver uses bounded Riccati fixed-point iteration, symmetric projection
of each iterate, and Eigen LDLT factorization for R + B'PB. It validates
dimensions, finite values, symmetry of Q/R, positive definiteness of R, option
ranges, positive definiteness of each control Hessian, and convergence. The
output object is only replaced on success.

Assumptions and limits: Q is expected to be positive semidefinite and R positive
definite; this implementation does not prove stabilizability/detectability or
certify closed-loop stability for every input system. Callers should check
closed-loop poles for safety-critical use. This is the LQR foundation for later
LQG estimator integration; it is not itself a Kalman filter.

Configure with CMake options MCA_BUILD_TESTS=ON and MCA_BUILD_BENCHMARKS=ON.
Tests include a scalar analytical reference, double-integrator closed-loop pole
check, invalid-input transactional behavior, symmetry/dimension rejection, and
bounded-iteration failure.

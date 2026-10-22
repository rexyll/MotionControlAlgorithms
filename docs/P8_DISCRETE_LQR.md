# P8-02 Discrete LQR and LQG baseline

The discrete LQR Riccati solver is provided by `mca/control/discrete_lqr.hpp`.
This companion header adds linear discrete Kalman prediction and measurement
updates for x[k+1]=A*x[k]+B*u[k]+w and y[k]=C*x[k]+v. Q and R represent
process and measurement noise covariance. The update uses the Joseph covariance
form. The LQG certainty-equivalent helper applies u=-K*x_hat, using the
estimated state supplied by the Kalman filter.

Both Kalman operations validate dimensions, finite values, covariance
symmetry/semidefiniteness, and positive definiteness of measurement noise and
innovation covariance. Output is transactional on failure. The implementation
does not automatically tune noise covariances, establish observability, or
guarantee closed-loop stability; callers must validate model assumptions and
closed-loop behavior for their plant.

Tests include scalar analytical Kalman gain/update, controlled prediction with
process noise, LQG feedback sign, invalid dimensions/non-finite values and
transactional output. The benchmark measures measurement-update cost at several
state dimensions.

# P4-02 UAV attitude representations

## Conventions

The utilities in `mca::uav::AttitudeRepresentation` convert among Euler ZYX angles, unit quaternions and rotation matrices. All angles are radians. Euler order is yaw about Z, pitch about the intermediate Y axis, then roll about body X; quaternions are stored as `(w,x,y,z)` and represent body-to-world rotation.

## API

- `from_euler_zyx(EulerZYX, Quaternion&, error)`: converts roll/pitch/yaw to a normalized quaternion.
- `to_euler_zyx(Quaternion, EulerZYX&, error)`: normalizes a finite nonzero quaternion and returns ZYX Euler angles.
- `to_rotation_matrix(Quaternion)`: delegates to the shared geometry rotation-matrix conversion.
- `from_rotation_matrix(Mat3, Quaternion&, error)`: validates finite values, orthonormality and determinant +1 before conversion.

Conversions that return `bool` preserve the output on invalid input. Near gimbal lock, Euler roll and yaw are not individually observable; the conversion returns the deterministic representative from the standard ZYX formulas. Controllers and state estimators should prefer quaternions or rotation matrices when avoiding Euler singularities matters.

## Validation

Tests cover Euler/quaternion round trips, quaternion/matrix consistency, invalid and non-finite inputs, transactional output preservation, and pitch near gimbal lock.

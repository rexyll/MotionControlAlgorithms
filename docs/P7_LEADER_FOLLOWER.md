# P7-03 Leader-follower control

## Model

The leader's externally planned velocity is passed through unchanged. For every follower \(i\), the desired position is the leader position plus that follower's configured offset, and the velocity command is

\[
v_i = v_L + k\big[(p_L + r_i)-p_i\big].
\]

Each follower command is independently limited to the configured maximum speed. The leader index may be any valid row; its desired offset is ignored.

## Validation and tests

The tests cover tracking direction and feed-forward, a nonzero leader index, follower speed saturation, malformed dimensions, non-finite values, and transactional output preservation. The benchmark covers 8, 32 and 128 agents.

## Scope boundary

This is a kinematic velocity-level primitive. It does not generate or limit the leader trajectory, guarantee collision avoidance, synchronize clocks, or handle delay/loss/asynchronous communication. Validate offsets, safe separation and the leader command upstream.

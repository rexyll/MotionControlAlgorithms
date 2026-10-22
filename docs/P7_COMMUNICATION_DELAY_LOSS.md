# P7-06 Communication delay and packet-loss model

## Inputs and deterministic semantics

The communication primitive operates on one row per agent and supports directed weighted links. Each receiver/sender link specifies a deterministic latency of either 0 samples (use the sender's current message) or 1 sample (use the supplied previous message), plus an explicit packet-delivery mask. This is intentionally a bounded one-step delay model, not an unbounded queue or wall-clock network simulator.

For a receiver with at least one delivered incoming packet, the output is the weighted average of delivered neighbors only; weights are renormalized over links that actually delivered. A dropped packet is excluded from that sample. If all incoming packets drop, the receiver retains its last delivered aggregate (zero-order hold). The output also reports the number of fresh packets and whether the last aggregate was held.

## Validation and failure behavior

The function validates agent and message dimensions, matrix dimensions, finite values, nonnegative link weights, zero diagonal, and exact 0/1 latency and delivery-mask entries. The output is transactional: invalid input returns false and leaves every output field unchanged. The caller owns message history and the deterministic packet mask, which makes simulation, testing and replay reproducible without global random state.

## Tests and benchmark

Unit tests cover per-link current/one-step-delayed messages, loss with weight renormalization, all-packets-lost hold behavior, invalid latency/delivery masks, self-links, non-finite data and transactional output preservation. The benchmark scales agent count and message dimension and exercises a mix of delayed and dropped packets.

## Scope boundary

This is a deterministic communication aggregation primitive, not a transport implementation, stochastic channel model, clock synchronizer or multi-step packet queue. Delays beyond one sample must be modeled by caller-maintained history. The held aggregate is an input from the caller and must be updated from the returned value on the next simulation step. Stability under communication faults remains a property of the coupled controller and must be validated separately.

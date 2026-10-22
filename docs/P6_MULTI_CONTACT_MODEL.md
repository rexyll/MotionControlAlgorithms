# P6-01 Multi-contact model

## Scope

P6-01 extends the single-point penalty contact model to a deterministic multi-contact aggregate suitable for humanoid and whole-body control experiments. Each contact is evaluated independently with the existing unilateral normal/friction model, then the resulting forces and moments are accumulated about a configurable reference point.

## Model

For contact `i`, the existing contact model produces `F_i` at world position `r_i`. The aggregate wrench is:

- `F = sum(F_i)`
- `tau = sum((r_i - r_ref) x F_i)`

The output also preserves contact order and reports the number of active contacts.

## Validation and transactional behavior

The model rejects an empty contact set, non-finite reference/contact positions, and any invalid underlying contact model input. A failed evaluation leaves the caller's existing output unchanged.

## Acceptance coverage

`tests/legged/multi_contact_model_test.cpp` covers:

1. two-contact force and moment aggregation;
2. contact ordering and inactive-contact handling;
3. empty/non-finite input rejection;
4. transactional rejection when one contact is invalid.

The benchmark evaluates 2, 4, 8 and 16 contacts.

## Deliberate scope boundary

This task does not yet solve contact-wrench optimization, friction-cone feasibility across contacts, contact switching logic, balance/ZMP control, or whole-body QP. Those are subsequent P6 tasks.

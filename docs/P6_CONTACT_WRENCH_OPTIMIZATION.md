# P6-03 Contact wrench optimization

The optimizer distributes a desired world-frame six-dimensional wrench across point contacts. Each contact contributes a three-dimensional force; the aggregate wrench is the sum of forces and moment arms crossed with forces.

A deterministic projected-gradient method minimizes a regularized least-squares wrench residual. Per-contact constraints enforce a bounded non-negative normal force and a conservative square friction pyramid: each tangential component is bounded by friction coefficient times normal force divided by sqrt(2). Inputs and intermediate results are checked for finite values, and output is committed only after successful completion.

This is a baseline allocator, not a full rigid-body contact QP. It omits contact moments, torsional friction, center-of-pressure constraints, coupled joint torques and exact second-order friction cones. The optional step size must be no greater than twice the reciprocal of the Hessian Frobenius upper bound; the default uses a conservative reciprocal bound.

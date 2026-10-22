# P6-04 Balance, ZMP and centroidal control baseline

This module adds a COM PD balance-acceleration command and a flat-ground zero-moment-point (ZMP) estimate. For COM height h above the support plane, the baseline relation is x_zmp = x_com - h*a_x/g and y_zmp = y_com - h*a_y/g. A configurable axis-aligned rectangular support region classifies whether the estimated ZMP lies inside the support polygon.

The balance controller computes per-axis proportional position and derivative velocity feedback and clamps the requested acceleration to explicit limits. Inputs are finite-checked, gains and limits validated, and outputs are committed transactionally.

Scope: this is a reduced-order baseline, not a complete humanoid balance controller. It assumes a flat horizontal support plane, omits angular-momentum-rate corrections and contact-force feasibility, and uses an axis-aligned rectangle rather than a general support polygon. Use the ZMP flag alongside contact/wrench feasibility checks before commanding hardware.

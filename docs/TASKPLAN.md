# P12 continuation note (2026-11-03)

- P12-05c.2 remains **open** for full EtherCAT SIL/master acceptance.
- The IgH 1.6.13 adapter implementation is now present behind `MCA_ENABLE_IGH` and has passed the pinned 1.6.13 API compile plus deterministic fake-API coverage for cyclic exchange, topology/health, PDO mapping and asynchronous SDO request/result lifecycle, including busy-slot isolation, timeout quarantine, terminal-buffer relocation and response-capacity enforcement.
- This evidence is still **API/fake-master evidence**, not wire-level EtherCAT SIL, slave-simulator validation, NIC timing validation or HIL. Therefore P12-05c.2 must not be marked accepted yet.
- SOEM remains blocked by the documented GPLv3/commercial licensing and packaging constraints; no SOEM dependency is being added to the root build.
- The next technical gate is to close the remaining IgH adapter evidence without weakening the dependency-free root build: verify all adapter-side mailbox/cyclic ownership rules, expand deterministic failure/recovery coverage where a concrete gap is found, and only then consider an external slave simulator or real target environment.


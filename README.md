# codedata
This repository accompanies the manuscript **“Dynamic Task Allocation and Local Route Optimization for Multi-UAV Offshore Wind Farm Inspection.”** It contains two MATLAB experiment modules for a hierarchical framework:

1. **Upper-layer dynamic task reassignment** (`untitled11.m`): feasibility-aware CADQN logic with battery, availability, wind, and task-feasibility checks; the common MGPGA route layer is used to execute each reassigned task set.
2. **Lower-layer route-optimizer comparison** (`untitled13.m`): conventional GA, IGA, and the proposed memory-guided protective GA (MGPGA) are compared under the same prescribed post-event task allocation.
3. Repository layout

```text
.
├── untitled11.m              # 8-UAV/48-WT CADQN task-reassignment experiment
├── untitled13.m              # GA vs IGA vs MGPGA route-optimization experiment
├── README_CADQN.md           # README to place with untitled11.m
├── README_MGPGA.md           # README to place with untitled13.m
└── outputs/                  # Created automatically by MATLAB scripts
```

## Quick start

1. Use MATLAB R2021b or newer.
2. Put `untitled11.m` and `untitled13.m` in the same directory.
3. Run `untitled11` to reproduce the upper-layer reassignment experiment and create:
   - `CADQN_MGPGA_8UAV_48WT_results.csv`
   - `CADQN_MGPGA_8UAV_48WT_assignment_sequences.csv`
   - `CADQN_MGPGA_8UAV_48WT_results.mat`
4. Run `untitled13` to reproduce the lower-layer GA/IGA/MGPGA comparison.

## Important interface statement

The present `untitled13.m` is **standalone**: its variable `common` is a fixed, documented post-event allocation benchmark. It does **not** programmatically import the route/task output written by `untitled11.m`. Thus, do not state that `untitled13.m` automatically consumes the preceding script’s output.

The intended scientific relationship is:

```text
dynamic state and events
        │
        ▼
untitled11.m: CADQN feasibility-aware task reassignment
        │  (exported allocation/route-sequence artifact)
        ▼
untitled13.m: GA / IGA / MGPGA route optimization under one fixed allocation
```

For a strict end-to-end release, add a small import function that loads the exported `CADQN-MGPGA` allocation from `CADQN_MGPGA_8UAV_48WT_results.mat` or the assignment CSV and passes it to `untitled13.m` instead of the hard-coded `common` cell array. Until that connector is provided, the two modules should be cited as **component-level validation scripts**.

- MATLAB random state is fixed by `rng(20260731,'twister')`.
- All comparison methods share the same farm layout, base, task pool, dynamic event schedule, and safety/energy model within each script.
- In the MGPGA study, GA, IGA, and MGPGA receive the same task allocation; only the lower-layer route optimizer changes.
- In the CADQN study, RS, PDQN, and CADQN use the same MGPGA route executor; only the upper-layer reassignment rule changes.
- Non-target wind turbines are treated as circular obstacles in route construction.


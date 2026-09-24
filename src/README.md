# Source code

This directory contains the C++ implementation of SimHGS, the hybrid genetic simheuristic introduced in the paper.

## Main components

| Files | Role |
| --- | --- |
| `main.cpp` | Program entry point and selection of the experiment variant. |
| `commandline.h` | Command-line parsing and the original experiment input/output paths. |
| `AlgorithmParameters.*` | Default algorithm parameters and parameter reporting. |
| `InstanceLib.*` | Parser for the SCDSP instance format. |
| `Params.*` | Problem data and shared algorithm state. |
| `Individual.*` | Scheduling-policy representation and evaluation state. |
| `Population.*` | Population management, selection, diversity, and output routines. |
| `Genetic.*` | SimHGS control flow and comparison experiment entry points. |
| `LocalSearch.*` | Deterministic VND and simulation-based local-search neighborhoods. |
| `SAA.*` | Scenario generation, simulation, policy evaluation, and fast evaluation support. |

## Build

From the repository root:

```bash
g++ -O3 -std=c++14 src/*.cpp -o simhgs
```

The source uses the C++ standard library. The reported experiments used GCC 10.2.0 and one CPU core.

## Runtime paths

`commandline.h` preserves the original experiment paths:

- inputs: `Instances/<instance>.txt`
- SimHGS outputs: `result/SimHGS/`
- deterministic-policy inputs: `result/DetHGS/`

Create these directories and copy or link the files from [`../instances/`](../instances/) before running. The root [`README.md`](../README.md) gives a complete example.

The active `solver.run()` call in `main.cpp` runs SimHGS. Commented calls correspond to the multi-start, traditional-simheuristic, baseline-policy, multi-VND, and sample-size experiments. Enable the relevant call and recompile to reproduce a variant.

Several files retain the MIT license notice for upstream hybrid genetic search code by Thibaut Vidal (https://github.com/vidalt/HGS-CVRP). Preserve those notices in derived versions.

# Computational results

This directory contains raw solution and simulation outputs for the 72 benchmark instances, together with aggregation programs and the summary workbook `result.xlsx`.

## Method directories

| Directory | Meaning in the paper |
| --- | --- |
| [`SimHGS/`](SimHGS/) | Proposed hybrid genetic simheuristic. |
| [`TSim/`](TSim/) | Traditional simheuristic comparator. |
| [`SBDF100/`](SBDF100/) | Simulation-based descent using first improvement and 100 in-search scenarios, without fast evaluation (SBDF). |
| [`SBDB100/`](SBDB100/) | Simulation-based descent using best improvement and 100 in-search scenarios, without fast evaluation (SBDB). |
| [`DetHGS/`](DetHGS/) | Deterministic scheduling policies used as the Det baseline and related comparisons. |

`instance.txt` lists the instance order used by the aggregation programs. `Statistic.cpp` and `StatisticSim.cpp` parse archived outputs; `result.xlsx` contains consolidated statistics.

## Output naming and contents

- `*_sol.txt` stores door assignments/sequences, scheduled start times, objective values, deterministic objective values, and runtimes. Multiple independent runs may be appended to one file.
- `*_solSim.txt` stores scenario-evaluation values and best, worst, and average summaries.
- `*_Bestlog.txt` in `SimHGS/` records the out-of-sample simulated objective value of the policy corresponding to the best solution.

The paper reports 10 independent runs per algorithm and instance. Final policies are assessed with 20,000 out-of-sample scenarios. These are archived results, not destinations for a new run; the default executable writes to the separate original-layout directory `result/TDSP/`.

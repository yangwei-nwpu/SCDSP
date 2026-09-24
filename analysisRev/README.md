# Revised-manuscript analysis

This directory contains post-processing inputs, Jupyter notebooks, archived experiment variants, summary tables, and figures for the revised manuscript and online supplement.

## Principal notebooks

- `Statistic.ipynb`: parses and summarizes solution outputs.
- `StatSolandAvg.ipynb`: prepares best/average comparisons and significance results.
- `Plot.ipynb`: creates runtime, affected-scenario, improvement, policy, and sensitivity figures.
- `SampleSizeRobust.ipynb`: analyzes scenario-size accuracy and out-of-sample robustness.

Run notebooks from this directory so relative paths resolve. A typical environment needs Python 3, Jupyter, NumPy, pandas, SciPy, Matplotlib, and openpyxl. Some inputs are ZIP archives; extract only the archive needed by a notebook in its current directory.

## Subdirectories

| Directory | Analysis |
| --- | --- |
| `AffScenarios/` | Percentage of scenarios affected by timing moves. |
| `SimHGSTimeLog/` | SimHGS runtime decomposition. |
| `NOFAST/` | Ablation without fast scenario evaluation. |
| `NoCutOff/` | Ablation without the cutoff rule. |
| `MVND/` | Multistart-NSBLS. |
| `SampleSize/` | Scenario-size study for 100, 500, 1,000, and 1,500 scenarios. |
| `UncertaintyDistribution/` | Low, medium, and high arrival-uncertainty comparisons. |
| `WeightAnalysis/` | Sensitivity to the unit tardiness-cost weight. |

Top-level CSV, TXT, XLSX, and PDF files are derived tables and publication figures. Each subdirectory has its own README.

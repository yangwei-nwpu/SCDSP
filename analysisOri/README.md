# Original analysis files

This directory preserves analysis artifacts from an earlier stage of the study for provenance.

- `PyPlot.ipynb` and `figures.R` generate comparison and performance-guarantee plots.
- `GapBstRes3.csv` and `GapAvgRes3.csv` contain best- and average-objective gaps.
- `MultitimeSetAX.csv`, `MultitimeSetBX.csv`, and `MultitimeSetCX.csv` contain multi-start runtime data by set.
- `PerfGuaA.csv`, `PerfGuaB.csv`, and `PerfGuaC.csv` contain performance-guarantee data.
- The PDF files are generated figures.

For final revised analyses, use [`../analysisRev/`](../analysisRev/). To rerun `figures.R`, install `ggplot2`, `reshape2`, `latex2exp`, `ggsci`, and `gridExtra`; for the notebook, use Python/Jupyter with pandas, NumPy, and Matplotlib.

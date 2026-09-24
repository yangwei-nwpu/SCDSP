# Scenario-size analysis

This directory supports the sensitivity study for in-search scenario counts `Ns = 100, 500, 1000, 1500` and the independent out-of-sample validation.

- `S100/`, `S500/`, `S1000/`, and `S1500/` contain archived move-selection statistics for the corresponding scenario count.
- `SimHGS1000/` contains policies produced using 1,000 in-search scenarios.
- `instance.txt` records the instance order used by the analysis.

The results are processed by `../SampleSizeRobust.ipynb` and reported in Online Supplement Section OS.5.3.

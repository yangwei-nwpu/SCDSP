[![INFORMS Journal on Computing Logo](https://INFORMSJoC.github.io/logos/INFORMS_Journal_on_Computing_Header.jpg)][def]

# A Simheuristic for the Cross-Docking Scheduling Problem under Uncertain Truck Arrival Times

This repository is distributed in association with the [INFORMS Journal on Computing][def] under the [MIT License](LICENSE).

It contains the code, benchmark instances, raw computational results, and analysis files used in:

- **Title:** A simheuristic for the cross-docking scheduling problem under uncertain truck arrival times
- **Authors:** Wei Yang, Yang Wang, Jin-Kao Hao, Ada Che, and Abraham P. Punnen
- **INFORMS JoC repository ID:** `2025.1515`

The paper introduces SimHGS, a hybrid genetic simheuristic for a stochastic cross-docking scheduling problem with uncertain truck arrival times. SimHGS combines variable neighborhood descent, simulation-based local search, and a fast scenario-evaluation technique to optimize fixed truck sequences and release times.

Ongoing development is hosted at [https://github.com/yangwei-nwpu/SCDSP](https://github.com/yangwei-nwpu/SCDSP). This repository is the archival snapshot associated with the paper.

## Cite

To cite the contents of this repository, please cite both the paper and the repository snapshot using their respective DOIs:

- Paper: [https://doi.org/10.1287/ijoc.2025.1515](https://doi.org/10.1287/ijoc.2025.1515)
- Repository snapshot: [https://doi.org/10.1287/ijoc.2025.1515.cd](https://doi.org/10.1287/ijoc.2025.1515.cd)

```bibtex
@misc{yang2025.1515,
  author    = {Wei Yang and Yang Wang and Jin-Kao Hao and Ada Che and Abraham P. Punnen},
  publisher = {INFORMS Journal on Computing},
  title     = {{A Simheuristic for the Cross-Docking Scheduling Problem under Uncertain Truck Arrival Times}},
  year      = {2026},
  doi       = {10.1287/ijoc.2025.1515.cd},
  url       = {https://github.com/INFORMSJoC/2025.1515},
  note      = {Available for download at https://github.com/INFORMSJoC/2025.1515},
}
```

## Repository contents

| Path | Contents |
| --- | --- |
| [`src/`](src/) | C++ implementation of SimHGS and its deterministic and simulation-based local-search components. |
| [`instances/`](instances/) | The 72 benchmark instances: 24 sizes for each of Sets A, B, and C. |
| [`results/`](results/) | Raw outputs for SimHGS and the comparison methods, plus aggregation programs and the summary workbook. |
| [`analysisRev/`](analysisRev/) | Analysis notebooks, intermediate data, ablation/sensitivity outputs, and figures for the revised manuscript and online supplement. |
| [`analysisOri/`](analysisOri/) | Earlier analysis scripts, input tables, and figures retained for provenance. |
| [`docs/`](docs/) | Online supplement and detailed computational results accompanying the paper. |

Each data, source, result, and analysis directory contains a local `README.md` describing its files.

## Benchmark naming convention

An instance name has the form `TDSP<n>_<d>_<n>_<d>_<set>.txt`, where `n` is the number of inbound trucks and outbound trucks, and `d` is the number of inbound doors and outbound doors.

| Suffix | Paper set | Main characteristic |
| --- | --- | --- |
| `M` | Set A | Sparse goods-flow matrix (density 0.3). |
| `MD` | Set B | Dense goods-flow matrix (density 1.0), baseline arrival pattern. |
| `X` | Set C | Dense goods-flow matrix with more concentrated truck arrivals. |

See [`instances/README.md`](instances/README.md) for the complete text format and generation rules.

## Software requirements

- A C++ compiler supporting C++14. The reported experiments used GNU GCC 10.2.0 with `-O3`.
- Linux or another Unix-like environment is recommended. The reported runs used one core of an Intel Xeon Gold 6226R at 2.90 GHz under Linux 3.10 x86-64.
- For the analysis notebooks: Python 3, Jupyter, NumPy, pandas, SciPy, Matplotlib, and openpyxl.
- For [`analysisOri/figures.R`](analysisOri/figures.R): R with `ggplot2`, `reshape2`, `latex2exp`, `ggsci`, and `gridExtra`.

The C++ implementation uses only the standard library; CPLEX is not required to compile or run SimHGS. CPLEX 22.1.0 was used separately for the deterministic MILP benchmark reported in the online supplement.

## Build

From the repository root:

```bash
g++ -O3 -std=c++14 src/*.cpp -o simhgs
```

This command has been checked against the archived source. See [`src/README.md`](src/README.md) for implementation details.

## Run SimHGS

The archived program uses the original experiment directory layout compiled into `src/commandline.h`. Prepare it from the repository root:

```bash
mkdir -p Instances/SCDSP result/SimHGS result/DetHGS
cp instances/*.txt Instances/SCDSP/
```

Run an instance by its basename, without the `.txt` extension:

```bash
./simhgs -Ins TDSP10_3_10_3_M -t 1800 -seed 1
```

| Option | Meaning | Archived default |
| --- | --- | --- |
| `-Ins` | Instance basename. | Required for an explicit run. |
| `-t` | Time limit in seconds. | `1800` |
| `-seed` | Random seed. | `1` |
| `-it` or `-nbIter` | Iteration limit. | `10000` |
| `-tollerance` | Deterministic cutoff tolerance (spelling follows the code). | `0.9` |
| `-nbGranular`, `-mu`, `-lambda`, `-nbElite`, `-nbClose` | Population and neighborhood parameters. | See `src/AlgorithmParameters.cpp`. |

The executable writes solution, simulation, log, and affected-scenario files under `result/TDSP/`. Archived paper outputs are stored separately under [`results/`](results/).

## Replicating the computational study

The study used 10 independent runs per method and instance. Candidate solutions were re-evaluated with 1,000 scenarios, and final out-of-sample performance was assessed with 20,000 scenarios.

1. Compile the code and prepare the original runtime directory layout shown above.
2. Run SimHGS for the 72 names in [`results/instance.txt`](results/instance.txt), changing the random seed for each independent run.
3. Use the outputs in [`results/`](results/) and `Statistic.cpp` and `StatisticSim.cpp` to inspect raw solutions and summary statistics.
4. Run the notebooks in [`analysisRev/`](analysisRev/) from that directory to reproduce revised tables and figures. Extract the relevant archived ZIP in place when a notebook needs it.
5. Use [`analysisOri/`](analysisOri/) for earlier figures and provenance files.

Several variants are selected by the active method call in `src/main.cpp`. Reproducing baseline-policy, multi-VND, sample-size, or traditional-simheuristic experiments requires enabling the corresponding call and recompiling. See [`results/README.md`](results/README.md) and [`analysisRev/README.md`](analysisRev/README.md).

## License and third-party notice

The repository is released under the [MIT License](LICENSE). Parts of the hybrid genetic search implementation retain the MIT notice of Thibaut Vidal in the relevant source headers; preserve those notices in redistributed source files.

## Support

For this archival snapshot, open an issue in the [INFORMS JoC repository](https://github.com/INFORMSJoC/2025.1515). For ongoing development, use the [development repository](https://github.com/yangwei-nwpu/SCDSP).


[def]: https://pubsonline.informs.org/journal/ijoc

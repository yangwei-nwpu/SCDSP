/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */
// This header file must be readable in C.

#ifndef ALGORITHMPARAMETERS_H
#define ALGORITHMPARAMETERS_H

// Collects all user-configurable controls for SimHGS and the retained experiments.
struct AlgorithmParameters {
	int nbGranular;			// Granular search parameter, limits the number of moves in the RI local search
	int mu;					// Minimum population size
	int lambda;				// Number of solutions created before reaching the maximum population size (i.e., generation size)
	int nbElite;			// Number of elite individuals
	int nbClose;			// Number of closest solutions/individuals considered when calculating diversity contribution
	// Relative deterministic-cost tolerance used to trigger stochastic refinement.
	double tollerance;
	int seed;				// Random seed. Default value: 0
	int nbIter;				// Nb iterations without improvement until termination (or restart if a time limit is specified). Default value: 20,000 iterations
	int nbIterTraces;       // Number of iterations between traces display during HGS execution
	double timeLimit;		// CPU time limit until termination in seconds. Default value: 0 (i.e., inactive)
	int useSwapStar;		// Use SWAP* local search or not. Default value: 1. Only available when coordinates are provided.
	// Enables sample-average approximation during solution improvement.
	bool useSAA;
	// Selects the best available incumbent as the starting point of SBD.
	bool useBest;
	// Maximum number of restart phases allowed in the selected experiment.
	int maxrestart;
	// Experiment-specific ratio controlling the search/evaluation allocation.
	double rato;
};

#ifdef __cplusplus
extern "C"
#endif
// Returns the repository's default algorithm configuration.
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
// Prints the effective configuration used by a run.
void print_algorithm_parameters(const AlgorithmParameters & ap);
#endif

#endif //ALGORITHMPARAMETERS_H

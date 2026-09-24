#pragma once
/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef GENETIC_H
#define GENETIC_H

#include "Population.h"
#include "Individual.h"

// Coordinates population evolution, crossover, local search, and simulation evaluation.
class Genetic
{
public:

	Params & params;				// Problem parameters
	LocalSearch localSearch;		// Local Search structure
	Population population;			// Population (public for now to give access to the solutions, but should be be improved later on)
	Individual offspring;			// First individual to be used as input for the crossover
	// Scenario generator and sample-average approximation evaluator.
	SAA saa;
	// Generates a randomized inbound/outbound door assignment and service order.
	void generateAssignemnt(Individual & result);
	// Applies order crossover to the two scheduling chromosomes.
	void crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2);
	// Runs the main SimHGS procedure.
	void run();
	// Runs the traditional simheuristic variant retained for comparison.
	void TradiSim();
	// Runs the repeated deterministic VND experiment.
	void MultiVND();
	// Evaluates the effect of the simulation sample size and writes its results.
	void SampleSizeTest(std::string resultfile);
	// Runs the multi-start local-search experiment.
	void Multi(std::string file);
	// Compares SimHGS with deterministic and dispatching-policy baselines.
	void BaselineComparison(std::string resultpath, std::string deterministicpath, std::string instancefile);
	// Constructor
	Genetic(Params & params);
};

#endif

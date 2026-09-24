#pragma once
/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */
#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"
#include "LocalSearch.h"
//#include "Split.h"

typedef std::vector <Individual*> SubPopulation;
// Records one time-stamped incumbent improvement for convergence output.
struct SP {
	double time;
	double SAALB;
	int DECLB;
	double INISimLB;
	int INILB;
};
// Owns the evolutionary population and tracks restart and global incumbents.
class Population
{
private:

	Params & params;							// Problem parameters
	// Shared stochastic evaluator used to assess and refine candidates.
	SAA &saa;
	LocalSearch & localSearch;					// Local search structure
	SubPopulation feasibleSubpop;			    // Feasible subpopulation, kept ordered by increasing penalized cost
	std::vector<SP> searchProgress; // Keeps tracks of the time stamps of successive best solutions
	Individual bestSolutionRestart;              // Best solution found during the current restart of the algorithm
	Individual bestSolutionOverall;              // Best solution found during the complete execution of the algorithm

	// Evaluates the biased fitness of all individuals in the population
	void updateBiasedFitnesses(SubPopulation & pop);

	// Removes the worst individual in terms of biased fitness
	void removeWorstBiasedFitness(SubPopulation & subpop);
	// Alternative survivor-removal rule retained for the path-relinking experiment.
	void removeWorstBiasedFitness2(SubPopulation & subpop);

public:

	// Creates an initial population of individuals
	void generatePopulation();

	void generatePopulationTradiSim();

	// Generates the initial population used by the path-relinking variant.
	void generatePopulationPR();

	// Add an individual in the population (survivor selection is automatically triggered whenever the population reaches its maximum size)
	// Returns TRUE if a new best solution of the run has been found
	bool addIndividual(const Individual & indiv, bool updateFeasible);
	// Inserts an individual without the standard survivor-selection workflow.
	bool addIndividualDirect(const Individual & indiv);
	// Updates stored incumbent information from an improved individual.
	void updateIndividual(const Individual & indiv);
	// Cleans all solutions and generates a new initial population (only used when running HGS until a time limit, in which case the algorithm restarts until the time limit is reached)
	void restart();
	void restartTradiSim();
	// Performs the retained population-management step and reports its outcome.
	bool manegement();

	// Select an individal by binary tournament in the union of the feasible and infeasible subpopulations.
	const Individual & getBinaryTournament();
	// Returns the individual at the requested population position.
	 Individual * getIndiv(int ith);
	// Accesses the best feasible individual
	const Individual * getBestFeasible();


	// Accesses the best found solution at all time
	const Individual * getBestFound();

	// Prints population state
	void printState(int nbIter, int nbIterNoImprovement);

	// Distance measure between two individuals, used for diversity calculations
	double brokenPairsDistance(const Individual & indiv1, const Individual & indiv2);
	// Alternative distance measure used by the retained population variant.
	double NewDistance(const Individual & indiv1, const Individual & indiv2);
	// Returns the average broken pairs distance of this individual with the nbClosest individuals in the population
	double averageBrokenPairsDistanceClosest(const Individual & indiv, int nbClosest);

	// Returns the average diversity value among the 50% best individuals in the subpopulation
	double getDiversity(const SubPopulation & pop);

	// Returns the average solution value among the 50% best individuals in the subpopulation
	double getAverageCost(const SubPopulation & pop);

	// Exports in a file the history of solution improvements
	void exportSearchProgress(std::string fileName, std::string instanceName);
	void exportSearchTime(std::string fileName);
	void exportAffectedScenarios(std::string fileName);
	// Re-evaluates the incumbent with a large independent scenario sample.
	void LargeSampleEvaluation();
	// Writes the schedule, simulation summary, and detailed log files.
	void exportCVRPLibFormat(const Individual & indiv, std::string fileName, std::string fileName2, std::string filename3);


	// Constructor
	Population(Params & params,  LocalSearch & localSearch,SAA &saa);

	// Destructor
	~Population();
};

#endif

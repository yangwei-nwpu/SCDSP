/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H
#include <vector>
#include <list>
#include <set>
#include "Params.h"
constexpr auto MAXTR = 201;
constexpr auto MAXDR = 50;
// Objective components and feasibility information attached to one candidate schedule.
struct EvalIndiv
{
	// Deterministic objective value of the encoded schedule.
	int determineCost = 0;
	// Penalized value used for population ranking and survivor selection.
	double penalizedCost = 0;		// Penalized cost of the solution
	// Initial simulation estimate before stochastic improvement.
	double Ini_Sim_LB = 0;
	// Initial deterministic estimate before stochastic improvement.
	int Ini_LB=0;
	// Waiting/tardiness contribution of the schedule.
	int WTime = 0.;			// Total distance
	// Transfer-related contribution used in the penalized objective.
	int penalizedTransCost = 0;
	// Aggregate time-window infeasibility of the schedule.
	int TimeWarp = 0;		// Sum of excess duration in all routes
	bool isFeasible = false;		// Feasibility status of the individual
};

// Encodes door assignments, service sequences, timing, and fitness for one solution.
class Individual
{
public:

	EvalIndiv eval;															// Solution cost parameters
	int chromTO[MAXTR];											         	// Giant tour representing the individual
	int chromRO[MAXDR][MAXTR];							                 	// For each vehicle, the associated sequence of deliveries (complete solution)
	int chromTD[MAXTR];											         	// Giant tour representing the individual
	// Outbound service sequence at each outbound door.
	int chromRD[MAXDR][MAXTR];
	int successorsO[MAXTR];											        // For each node, the successor in the solution (can be the depot 0)
	int predecessorsO[MAXTR];	
	int successorsD[MAXTR];											        // For each node, the successor in the solution (can be the depot 0)
	int predecessorsD[MAXTR];	// For each node, the predecessor in the solution (can be the depot 0)
	std::multiset < std::pair < double, Individual* > > indivsPerProximity;	// The other individuals in the population, ordered by increasing proximity (the set container follows a natural ordering based on the first value of the pair)
	double biasedFitness;														// Biased fitness of the solution
	int AssignO[MAXTR];
	int AssignD[MAXTR];                                                     // indicates to which door the truck is assigned 
	// Planned inbound truck start times.
	int StimeO[MAXTR];
	// Planned outbound truck start times.
	int StimeD[MAXTR];
	// Per-truck transfer-cost contributions cached during evaluation.
	int penaltyTrans[MAXTR];
	// Per-truck inbound waiting/tardiness contributions.
	int penaltyWTimeO[MAXTR];
	// Per-truck outbound waiting/tardiness contributions.
	int penaltyWTimeD[MAXTR];
	int posi;                                                              //indicates the position in the population according to its cost
	// Measuring cost and feasibility of an Individual from the information of chromR (needs chromR filled and access to Params)
	void evaluateCompleteCost(const Params & params);
	// Recomputes the alternative complete-cost formulation used by retained experiments.
	void evaluateCompleteCost2(const Params & params);
	// Constructor of a random individual containing only a giant tour with a shuffled visit order
	Individual(Params & params);

	// Constructor of an individual from a file in CVRPLib solution format as produced by the algorithm (useful if a user wishes to input an initial solution)
	//Individual(Params & params, std::string fileName);
};
#endif

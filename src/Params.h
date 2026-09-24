
/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef PARAMS_H
#define PARAMS_H


#include "AlgorithmParameters.h"
#include <string>
#include <vector>
#include <list>
#include <set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <time.h>
#include <climits>
#include <algorithm>
#include <unordered_set>
#include <random>
#define MY_EPSILON 0.0001 // Precision parameter, used to avoid numerical instabilities
#define PI 3.14159265359
//inbound truck
// Stores the time window, processing time, and positive-flow successors of one inbound truck.
struct TruckO
{
	int AT;			
	int ES;			
	int EF;
	int LS;
	int LF;
	int GT;
	int *Suc;//indices of successor outbound truck
	
};
// Stores the time window, processing time, and positive-flow predecessors of one outbound truck.
struct TruckD
{
	int AT;			
	int ES;			
	int EF;
	int LS;
	int LF;
	int GT;
	int *Pre;//indices of successor outbound truck
	
};
// Holds instance data, precomputed cost tables, random state, and algorithm parameters.
class Params
{
public:

	/* PARAMETERS OF THE GENETIC ALGORITHM */
	bool verbose;                       // Controls verbose level through the iterations
	AlgorithmParameters ap;	            // Main parameters of the HGS algorithm

	/* ADAPTIVE PENALTY COEFFICIENTS */
	// Weight applied to time-window violations during search.
	double penaltyTW;				// Penalty for one unit of capacity excess (adapted through the search)
	//double penaltyDuration;				// Penalty for one unit of duration excess (adapted through the search)

	/* START TIME OF THE ALGORITHM */
	clock_t startTime;                  // Start time of the optimization (set when Params is constructed)

	/* RANDOM NUMBER GENERATOR */
	std::minstd_rand ran;               // Using the fastest and simplest LCG. The quality of random numbers is not critical for the LS, but speed is

	/* DATA OF THE PROBLEM INSTANCE */
	// Counts of inbound/outbound trucks and doors in the current instance.
	int nbInTrucks;											// Number of clients (excluding the depot)
	int nbOutTrucks;											// Number of clients (excluding the depot)
	int nbInDoors;										// Number of vehicles
	int nbOutDoors;										// Number of vehicles
	std::vector< TruckO > ITK;								// Vector containing information on each client
	std::vector< TruckD > OTK;
	// Door distances, transfer times, priorities, and cargo flows.
	std::vector< std::vector< int > > Dis;	// Distance matrix
	std::vector< std::vector< int > > Time;
	std::vector< std::vector< int > > PP;
	std::vector< std::vector< int > > Flow;
	std::vector< std::vector< int > > correlatedVerticesO;	// Neighborhood restrictions: For each client, list of nearby customers
	std::vector< std::vector< int > > correlatedVerticesD;
	// Time-indexed lookup tables used by deterministic and stochastic delta evaluation.
	int ****funTrans;
	int **funOTO;
	int **funOTD;
	int **FP;
	// Time-horizon and objective-scaling constants derived from the instance.
	int MAXDLINE;
	int TotalPenality = 0;
	int MAXOTime = 0;
	double gammaTW = 0;
	double gammaWT = 0;
	double miniTep = 10;
	int alpha = 1;
	// Initialization from a given data set
	// Copies parsed arrays and builds the lookup tables required by the search.
	Params(int * AT_O, int * AT_D,
		int * ES_O, int * ES_D,
		int * EF_O, int * EF_D,
		int * LS_O, int * LS_D,
		int * LF_O, int * LF_D,
		int * GT_O, int * GT_D,
		int ** Dis, int ** _Time,
		int ** PP, int ** Flow,
		int ** Suc, int**Pre,
		int NumTruck_O,									
		int NumTruck_D,
		int NumDoor_O,
		int NumDoor_D,
		int MAXDLINE,
		bool verbose,
		const AlgorithmParameters& ap);
};
#endif


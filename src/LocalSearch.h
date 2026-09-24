#pragma once
/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include "Individual.h"
#include "SAA.h"
#include <iomanip>

struct Node;

// Structure containing a route
// In this SCDSP adaptation, a route represents the service sequence of one door.
struct Route
{
	int cour;							// Route index
	int nbTrucks;					// Number of trucks visited in the route
	int whenLastModified;				// "When" this route has been last modified
	int whenLastTestedSWAPStar;			// "When" the SWAP* moves for this route have been last tested
	Node * head;						// Pointer to the associated depot
	int WTime;					// Total time on the route
	int TimeWarp;						// Total load on the route
	int penalty;						
};

// Represents one truck, or a sentinel head, in a linked door schedule.
struct Node
{
	bool ishead;						// Tells whether this node represents a depot or not
	bool isfeasible;
	int cour;							// Node index or id
	int position;						// Position in the route 
	int whenLastTestedRI;				// "When" the RI moves for this node have been last tested
	Node * next;						// Next node in the route order
	Node * prev;						// Previous node in the route order
	Route * route;						// Pointer towards the associated route
	int cumulatedStart;				    // Cumulated load on this route until the truck (including itself)
	int cumulatedEnd;				    // Cumulated time on this route until the truck (including itself)
	int TransCost;
	int WTimeCost;
};

// Main local learch structure
// Implements deterministic VND and simulation-based local improvement procedures.
class LocalSearch
{

private:

	Params & params;							// Problem parameters
	SAA &saa;
	bool searchCompleted;						// Tells whether all moves have been evaluated without success
	int nbMoves;								// Total number of moves (RI and SWAP*) applied during the local search. Attention: this is not only a simple counter, it is also used to avoid repeating move evaluations
	std::vector < int > orderTrucksO;				// Randomized order for checking the nodes in the RI local search
	std::vector < int > orderTrucksD;
	std::vector < int > orderDoorsO;			// Randomized order for checking the routes in the SWAP* local search
	std::vector < int > orderDoorsD;
												//std::set < int > emptyDoors;				// indices of all empty routes
	int loopID;									// Current loop index
	int Tpr = 1000;
	/* THE SOLUTION IS REPRESENTED AS A LINKED LIST OF ELEMENTS */
	// Truck nodes and sentinel heads form the inbound and outbound door sequences.
	std::vector < Node > trucksO;				// Elements representing clients (clients[0] is a sentinel and should not be accessed)
	std::vector < Node > trucksD;
	std::vector < Node > headsO;				// Elements representing clients (clients[0] is a sentinel and should not be accessed)
	std::vector < Node > headsD;										//std::vector < Node > depots;				// Elements representing depots
	std::vector < Route > doorsO;				// Elements representing routes
	std::vector < Route > doorsD;
	// Scratch buffers used to evaluate and commit incremental neighborhood moves.
	int changedStime[MAXTR];
	int changedIndex[MAXTR];
	int changedWTcost[MAXTR];
	int changedTrans[MAXTR];
	int changedNbTrucks[MAXDR];
	int bestchangedStime[MAXTR];
	int bestchangedIndex[MAXTR];
	int bestchangedWTcost[MAXTR];
	int bestchangedTrans[MAXTR];
	int bestchangedNbTrucks[MAXDR];
	
	/*======  SAA result   =======*/
	// Incumbent objective values shared between deterministic and stochastic search.
	double local_best;//SAA local best
	int localDetCost;//LS local best under SAA local best
	double ini_sim_LB;//simluation cost under refinebest
	
	int local_StimeO[MAXTR];
	int local_StimeD[MAXTR];
	// Door sequences retained for the best locally improved solution.
	int Local_SequenceX[MAXDR][MAXTR];//updated aftr SAA_LocalSearch, note that the local_sequenceX is not the same with saa.SequenceX
	int Local_SequenceY[MAXDR][MAXTR];
	/*========   schedulingY result and data structure  =========*/
	// Cached cost changes for advancing or delaying outbound start times.
	int DltVec[2][MAXTR];
	int determinedCost;// if and only if determinedCost < local_best,  the SAA.LS procedure is triggered 
	int transcost[MAXTR][MAXTR];//flow ij is transfered or not
	int historicalBest;
	int refineBest;
	//int DeterminedCostSAA;
	/*========   Auxiliary variables used to allocate   =========*/
	// Nodes and door schedules currently examined by a neighborhood move.
	Node * nodeU;
	Node * nodeX;
	Node * nodeV;
	Node * nodeY;
	Route * routeU;
	Route * routeV;
	int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex;
	int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex;
	
	int penaltyTransLS;
	int WTimeLS;
	bool intraRouteMove;
	int TotalTimeWarp;

	// Caches indices and adjacency information for the current pair of routes.
	void setLocalVariablesRouteU(); // Initializes some local variables and distances associated to routeU to avoid always querying the same values in the distance matrix
	void setLocalVariablesRouteV(); // Initializes some local variables and distances associated to routeV to avoid always querying the same values in the distance matrix
	
	// Incremental objective changes for inbound shift, swap, and 2-opt neighborhoods.
	int delta_shiftO();//u is inserted after v, move1
	int delta_shiftO2();//u x is  inserted after v,move2
	int delta_2shiftO();// remove u x, then  insert x u after v,move3
	int delta_swapO2T1(); //swap (u,x) and v ,move5
	int delta_swapO2T2();//swap (u,x) and (v,y), move 6
	int delta_swapO(Node*U, Node*V); //swap u,v, move 4,
	int delta_2_optO();//move 7,8
	int delta_2_optStarO();//move 9

	// Incremental objective changes for outbound shift, swap, and 2-opt neighborhoods.
	int delta_shiftD();//u is inserted after v, move1
	int delta_shiftD2();//u x is  inserted after v,move2
	int delta_2shiftD();// remove u x, then  insert x u after v,move3
	int delta_swapD2T1(); //swap (u,x) and v ,move5
	int delta_swapD2T2();//swap (u,x) and (v,y), move 6
	int delta_swapD(Node*U, Node*V); //swap u,v, move 4,
	int delta_2_optD();//move 7,8
	int delta_2_optStarD();//move 9
	/* ROUTINES TO UPDATE THE SOLUTIONS */
	// Primitive linked-list edits used to apply accepted moves.
	static void insertNode(Node * U, Node * V, bool intraRouteMove);		// Solution update: Insert U after V
	static void swapNode(Node * U, Node * V, bool intraRouteMove);		// Solution update: Swap U and V							   
	//void updateRouteDataD(Route * myRoute);
	void updateRouteDataO(Route * myRoute, int changeNB);
	//void updateRouteDataO(Route * myRoute);			// Updates the preprocessed data of a route
	// Evaluates the timing and transfer contribution of one truck at a door.
	int computeIncost(int index, int stime,int indexdoor);         //compute the cost of "index" intruck with start time "stime"
	int computeOutcost(int index, int stime, int indexdoor);         //compute the cost of "index" outtruck with start time "stime"
	int compute_trial_insert_O(int nodeindex, int _cnt, int &_Curtime, int indexdoor);
	int compute_trial_insert_D(int nodeindex, int _cnt, int &_Curtime, int indexdoor);
	void updateTrucksO();
	void updateTrucksD();
	void updateRouteDataD(Route * myRoute, int changeNB);
	void initial_DoorsO(Route * myDoor);
	void initial_DoorsD(Route * myDoor);
	// Builds and incrementally improves the outbound start-time schedule.
	void schedulingY();
	void iniDltVector(int refineBest);
	int FastComputeDlt(int kth, int buf);
	void MoveFastUpdate(int kth, int buf);
	int ComputeCost(int *st);
	void initialTrans();
public:
	// Runtime statistics reported by the experimental drivers.
	double VNDTIME;                 // time of VND to sequence
	double LSTIME;                  // time of determine local search to release time
	double SAATIME;                 // time of SBLS to release time
	double SimulationTime;          // time of SBLS to release time
	// Run the local search with the specified penalty values
	// Executes one inbound or outbound neighborhood pass.
	bool runO3();
	bool runD3();
	bool runO3TradiSim();
	bool runD3TradiSim();
	// Runs the complete deterministic or simulation-based local-search cycle.
	void run3(Individual & indiv);
	void Iteraterun3(Individual & indiv);
	void Iteraterun3TradiSim(Individual & indiv);
	// Loading an initial solution into the local search
	void loadIndividual(const Individual & indiv);
	// Computes inbound/outbound sequence distances between two individuals.
	int ComputeDis_O(const Individual & Parent2);
	int ComputeDis_D(const Individual & Parent2);
	
	// Exporting the LS solution into an individual and calculating the penalized cost according to the original penalty weights from Params
	// Converts the linked-list schedule back into the individual's chromosome representation.
	void exportIndividual(Individual & indiv);
	void exportIndividualStoch(Individual & indiv);
	void exportSAAData();//start->stx sty
	// Loads the current stochastic schedule into the local-search representation.
	void loadSAA();
	void loadScheduling();

	//sequence ->Local_Sequence
	// Validates or updates outbound and inbound door sequences after a move.
	void checkOBJO();
	void checkOBJD(int delta);
	// Constructor
	void computeCostO(int delta,int x);
	void computeCostD(int delta,int x);
	// Applies diversification moves to both sides or to one side of the schedule.
	void perturb();
	void perturbO();
	void perturbD();
	void narrowSearch();
	void narrowSearch2();
	// Runs repeated local searches and writes the multi-start experiment data.
	void MultiStart(Individual & indiv,int iter, std::string file);
	//void MultiStart();
	void updatechange();
	void updateTrucksNewO();
	void updateTrucksNewD();
	// Allocates the linked-list search structures for the supplied instance and SAA engine.
	LocalSearch(Params & params,SAA& saa);
};

#endif

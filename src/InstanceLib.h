#pragma once
/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef INSTANCELIB_H
#define INSTANCELIB_H
#include<string>
#include<vector>

// Owns the raw arrays parsed from one SCDSP instance file.
class InstanceLIB
{
public:
	// Arrival, earliest/latest start, and earliest/latest finish times.
	int* AT_O;
	int* AT_D;
	int* EF_O;
	int* EF_D;
	int* ES_O;
	int* ES_D;
	int* LS_O;
	int* LS_D;
	int* LF_O;
	int* LF_D;
	// Door-distance, transfer-time, cargo-flow, and priority/penalty matrices.
	int** Dis;//I*J
	int** T;//M*N
	int** Flow;//M*N
	int** PP;//M*N
	int**Suc;//Successor node of inbound truck with a positive cargo flow M*N
	int**Pre;//Predecessor node of outbound truck  with a positive cargo flow N*M
	// Inbound and outbound processing times.
	int* GT_O;
	int* GT_D;
	int NumTruck_O;											
	int NumTruck_D;
	int NumDoor_O;
	int NumDoor_D;
	int velocity;
	// Largest latest-finish time, used to size time-indexed lookup tables.
	int MAXDLINE;
	// Loads an instance and derives its time-window and flow-adjacency arrays.
	InstanceLIB(std::string pathToInstance, bool isRoundingInteger);
};


#endif //INSTANCECVRPLIB_H

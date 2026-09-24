/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <iostream>
#include <string>
#include <climits>
#include "AlgorithmParameters.h"

// Parses command-line options and stores repository-relative input/output paths.
class CommandLine
{
public:
	AlgorithmParameters ap = default_algorithm_parameters();

	int nbVeh = INT_MAX;		// Number of vehicles. Default value: infinity
	std::string pathInstance;		// Instance path
	std::string pathSolution;		// Solution path
	bool verbose = true;
	bool isRoundingInteger = true;
	// Directory containing SCDSP instance files.
	std::string filepath = "Instances/SCDSP/";
	// Directory receiving SimHGS results.
	std::string resultpath = "result/SimHGS/";
	// Directory containing or receiving deterministic baseline results.
	std::string deterministicpath = "result/TDSPDet/";
	// Reads the line of command and extracts possible options
	CommandLine(int argc, char* argv[])
	{
		if (argc % 2 != 1 || argc > 27 || argc < 3)
		{
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			//display_help();
			std::cout << "Incorrect line of command and the defult parameter setting is applied!" << std::endl;
			pathInstance = "TDSP25_3_25_3_X";// 12_3_12_3  40_6_40_6
			pathSolution = pathInstance+"_sol";//10_3_10_3_X  25_3_25_3_M 50_8_50_8_M
		}
		else
		{
			//pathInstance = std::string(argv[1]);
			//pathSolution = std::string(argv[2]);
			for (int i = 1; i < argc; i += 2)
			{
				if (std::string(argv[i]) == "-Ins")
				{
					pathInstance = std::string(argv[i + 1]);
					pathSolution = std::string(argv[i + 1]) + "_sol";
				}
				else if (std::string(argv[i]) == "-t")
					ap.timeLimit = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-it")
					ap.nbIter = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-seed")
				{
					ap.seed = atoi(argv[i + 1]);
					//ap.seed = (unsigned int)time(NULL);
				}
				else if (std::string(argv[i]) == "-Rato")
					ap.rato = strtod(argv[i + 1], NULL);
				else if (std::string(argv[i]) == "-MaxIt")
					ap.maxrestart = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-tollerance")
					ap.tollerance = strtod(argv[i+1], NULL);
				else if (std::string(argv[i]) == "-nbGranular")
					ap.nbGranular = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-mu")
					ap.mu = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-lambda")
					ap.lambda = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-nbElite")
					ap.nbElite = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-nbClose")
					ap.nbClose = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-nbIter")
					ap.nbIter = atoi(argv[i + 1]);
				else
				{
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					display_help(); throw std::string("Incorrect line of command");
				}
			}
		}
	}

	// Printing information about how to use the code
	void display_help()
	{
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- HGS-CVRP algorithm (2020) ---------------------------------------------------" << std::endl;
		std::cout << "Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-seed mySeed] [-veh nbVehicles] [-log verbose]               " << std::endl;
		std::cout << "[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                         " << std::endl;
		std::cout << "[-t <double>] sets a time limit in seconds. If this parameter is set the code will be run iteratively until the time limit      " << std::endl;
		std::cout << "[-seed <int>] sets a fixed seed. Defaults to 0                                                                                  " << std::endl;
		std::cout << "[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. " << std::endl;
		std::cout << "[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                     " << std::endl;
		std::cout << std::endl;
		std::cout << "Additional Arguments:                                                                                                           " << std::endl;
		std::cout << "[-nbIterTraces <int>] Number of iterations between traces display during HGS execution. Defaults to 500                         " << std::endl;
		std::cout << "[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20                " << std::endl;
		std::cout << "[-mu <int>] Minimum population size. Defaults to 25                                                                             " << std::endl;
		std::cout << "[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40 " << std::endl;
		std::cout << "[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                     " << std::endl;
		std::cout << "[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4      " << std::endl;
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
};
#endif

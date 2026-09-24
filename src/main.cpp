/*
 * The software architecture of this implementation was inspired by
 * HGS-CVRP: https://github.com/vidalt/HGS-CVRP
 *
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "InstanceLib.h"
using namespace std;

int main(int argc, char *argv[])
{
	try
	{
		// Reading the arguments of the program
		CommandLine commandline(argc, argv);

		// Print all algorithm parameter values
		if (commandline.verbose) print_algorithm_parameters(commandline.ap);

		// Reading the data file and initializing some data structures
		if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		InstanceLIB TDSP(commandline.filepath+commandline.pathInstance+".txt", commandline.isRoundingInteger);
		Params params(TDSP.AT_O, TDSP.AT_D, TDSP.ES_O, TDSP.ES_D, TDSP.EF_O, TDSP.EF_D, TDSP.LS_O, TDSP.LS_D, TDSP.LF_O, TDSP.LF_D, TDSP.GT_O, TDSP.GT_D, TDSP.Dis,
			TDSP.T, TDSP.PP, TDSP.Flow, TDSP.Suc, TDSP.Pre, TDSP.NumTruck_O, TDSP.NumTruck_D, TDSP.NumDoor_O, TDSP.NumDoor_D, TDSP.MAXDLINE, commandline.verbose, commandline.ap);
		// 0. Initialize solver
		Genetic solver(params);

		// 1. Running SimHGS
		solver.run();

		//3. Running the MultiStart comparision
		//solver.Multi(commandline.resultpath + commandline.pathSolution);
		//system("pause");
		//return 0;

		// 4. Running HGS with traditional simheuristic
		//solver.TradiSim();

		// 5. Running baseline policy
		//solver.BaselineComparison(commandline.resultpath, commandline.deterministicpath, commandline.pathSolution);

		// 6. Running Multi-VND
		//solver.MultiVND();

		// 7. Sample Size Analysis  using saa.SAA_LS_SmpNumTest()
		//solver.SampleSizeTest(commandline.resultpath + commandline.pathSolution);
		
		// Exporting the best solution
		if (solver.population.getBestFound() != NULL)
		{
			if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
			solver.population.exportCVRPLibFormat(*solver.population.getBestFound(), commandline.resultpath + commandline.pathSolution + ".txt", commandline.resultpath + commandline.pathSolution + "Sim.txt", commandline.resultpath + commandline.pathSolution + "log.txt");
			solver.population.exportSearchProgress(commandline.resultpath + commandline.pathSolution + ".PG.csv", commandline.pathInstance);
			//solver.population.exportSearchTime(commandline.resultpath + commandline.pathSolution+"_timeLog.txt");
			//solver.population.exportAffectedScenarios(commandline.resultpath + commandline.pathSolution + "_Aff.txt");
			//std::cout << "Best " << solver.population.getBestFound()->eval.penalizedCost << std::endl;
		}
	}
	catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
	system("pause");
	return 0;
}

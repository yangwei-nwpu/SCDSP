#include "Genetic.h"

void Genetic::run()
{

	population.generatePopulation();
	
	int nbIter;
	int nbIterNonProd = 1;
	if (params.verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	for (nbIter = 0; nbIterNonProd <= params.ap.nbIter && ((int)(params.ap.timeLimit)==0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit); nbIter++)
	{
		/* SELECTION AND CROSSOVER */
		crossoverOX(offspring, population.getBinaryTournament(), population.getBinaryTournament());
		localSearch.Iteraterun3(offspring);
		bool isNewBest = population.addIndividual(offspring, true);
	
		/* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
		if (isNewBest) nbIterNonProd = 1;
		else nbIterNonProd++;

		/* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
		if (nbIter % params.ap.nbIterTraces == 0) population.printState(nbIter, nbIterNonProd);

		/* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED*/
		if ((int)(params.ap.timeLimit) != 0 && nbIterNonProd == params.ap.nbIter)
		{
			population.restart();
			nbIterNonProd = 1;
		}
	}
	if(!params.ap.useSAA){ population.LargeSampleEvaluation(); }
	if (params.verbose) std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC << std::endl;
}

void Genetic::TradiSim()
{
	
	population.generatePopulationTradiSim();

	int nbIter;
	int nbIterNonProd = 1;
	if (params.verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	for (nbIter = 0; nbIterNonProd <= params.ap.nbIter && (params.ap.timeLimit == 0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit); nbIter++)
	{
		/* SELECTION AND CROSSOVER */
		crossoverOX(offspring, population.getBinaryTournament(), population.getBinaryTournament());

		localSearch.Iteraterun3TradiSim(offspring);
		bool isNewBest = population.addIndividual(offspring, true);

		/* TRACKING THE NUMBER OF ITERATIONS SINCE LAST SOLUTION IMPROVEMENT */
		if (isNewBest) nbIterNonProd = 1;
		else nbIterNonProd++;

		if (nbIter % params.ap.nbIterTraces == 0) population.printState(nbIter, nbIterNonProd);

		/* FOR TESTS INVOLVING SUCCESSIVE RUNS UNTIL A TIME LIMIT: WE RESET THE ALGORITHM/POPULATION EACH TIME maxIterNonProd IS ATTAINED*/
		if (params.ap.timeLimit != 0 && nbIterNonProd == params.ap.nbIter)
		{
			population.restartTradiSim();
			nbIterNonProd = 1;
		}
	}
	population.LargeSampleEvaluation();
	if (params.verbose) std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC << std::endl;
}


void Genetic::MultiVND() { //compare with SimHGS
	
	while ((double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit) {
		Individual randomIndiv(params);
		localSearch.Iteraterun3(randomIndiv);
		population.updateIndividual(randomIndiv);
	}
}

void Genetic::SampleSizeTest(std::string resultfile)
{
	
	Individual randomIndiv(params);
	localSearch.Iteraterun3(randomIndiv);
	population.updateIndividual(randomIndiv);
	std::ofstream myfile(resultfile + "_SamSta.txt", std::ios::app);
	myfile << saa.TotalFalseIter << " " << saa.TotalIter << std::endl;
	myfile << "AveRegret: " << (double)(saa.TotalRegret / saa.TotalFalseIter) << std::endl;
	myfile << "AveRegretRate: " << (double)(saa.AveRateRegret / saa.TotalFalseIter) << std::endl;
	myfile << "********" << std::endl;
	myfile.close();
	std::ofstream mylog(resultfile + "_SamLog.txt", std::ios::app);
	for (int i = 0; i < saa.DetailRegret.size(); ++i)
	{
		mylog << saa.DetailRegret[i] << " " << saa.DetailAveRate[i] << "\n";
	}
	mylog.close();
}

void Genetic::Multi(std::string file) {
	for (int i = 1; i <= 10; i++) {
		std::cout << i << std::endl;
		int localBEst = 999999999;
		Individual randomIndiv(params);
		localSearch.MultiStart(randomIndiv, i, file);
	}
}

void Genetic::BaselineComparison(std::string resultpath, std::string deterministicpath, std::string instancefile) {
	
	/*************       Simulation using Deterministic policy            ***********/
	saa.LoadDeterministicPolicy(deterministicpath + instancefile + ".txt");
	double average_csot = 0.0;
	double best_simcost = 999999999;
	double worstsimcost = 0.0;
	double testcost = 0.0;
	std::ofstream myfileDet(resultpath + instancefile + "_SimDet.txt", std::ios::app);
	for (int x = 1; x <= 20; x++) {
		long long int _test_cost = 0;
		saa.SmpNum = 1000;
		saa.generate_sample2(x, 0);
		saa.generate_EST();
		saa.sortSenario();
		saa.SGS2_profiles(_test_cost, saa.sty, resultpath + instancefile + "_logDet.txt");
		testcost = (double)(_test_cost*1.0 / saa.SmpNum) + params.TotalPenality;
		if (worstsimcost < testcost - 1e-6) { worstsimcost = testcost; }
		if (best_simcost > testcost + 1e-6) { best_simcost = testcost; }
		average_csot += testcost;
		myfileDet << "Sim " << std::fixed << std::setprecision(2) << testcost << std::endl;
	}
	myfileDet << "Bst " << std::fixed << std::setprecision(2) << best_simcost << std::endl;
	myfileDet << "Wst " << std::fixed << std::setprecision(2) << worstsimcost << std::endl;
	myfileDet << "Avg " << std::fixed << std::setprecision(2) << average_csot / 20.0 << std::endl;
	myfileDet.close();
	/*************       Simulation using FDD policy            ***********/
	average_csot = 0.0;
	best_simcost = 999999999;
	worstsimcost = 0.0;
	testcost = 0.0;
	std::ofstream myfileEDD(resultpath + instancefile + "_SimEDD.txt", std::ios::app);
	for (int x = 1; x <= 20; x++) {
		saa.SmpNum = 1000;
		saa.generate_sample2(x, 0);
		saa.BaselinePolicy_profiles(testcost, saa.SmpNum, POLICY_EDD, resultpath + instancefile + "_logEDD.txt");
		if (worstsimcost < testcost - 1e-6) { worstsimcost = testcost; }
		if (best_simcost > testcost + 1e-6) { best_simcost = testcost; }
		average_csot += testcost;
		myfileEDD << "Sim " << std::fixed << std::setprecision(2) << testcost << std::endl;
	}
	myfileEDD << "Bst " << std::fixed << std::setprecision(2) << best_simcost << std::endl;
	myfileEDD << "Wst " << std::fixed << std::setprecision(2) << worstsimcost << std::endl;
	myfileEDD << "Avg " << std::fixed << std::setprecision(2) << average_csot / 20.0 << std::endl;
	myfileEDD.close();
	/*************       Simulation using MAG policy            ***********/
	average_csot = 0.0;
	best_simcost = 999999999;
	worstsimcost = 0.0;
	testcost = 0.0;
	std::ofstream myfileMAG(resultpath + instancefile + "_SimMAG.txt", std::ios::app);
	for (int x = 1; x <= 20; x++) {
		saa.SmpNum = 1000;
		saa.generate_sample2(x, 0);
		saa.BaselinePolicy_profiles(testcost, saa.SmpNum, POLICY_MAG, resultpath + instancefile + "_logMAG.txt");
		if (worstsimcost < testcost - 1e-6) { worstsimcost = testcost; }
		if (best_simcost > testcost + 1e-6) { best_simcost = testcost; }
		average_csot += testcost;
		myfileMAG << "Sim " << std::fixed << std::setprecision(2) << testcost << std::endl;
	}
	myfileMAG << "Bst " << std::fixed << std::setprecision(2) << best_simcost << std::endl;
	myfileMAG << "Wst " << std::fixed << std::setprecision(2) << worstsimcost << std::endl;
	myfileMAG << "Avg " << std::fixed << std::setprecision(2) << average_csot / 20.0 << std::endl;
	myfileMAG.close();
}

void Genetic::crossoverOX(Individual & result, const Individual & parent1, const Individual & parent2)
{
	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool>(params.nbInTrucks, false);
	
	// Picking the beginning and end of the crossover zone
	std::uniform_int_distribution<> distr(0, params.nbInTrucks - 1);
	int start = distr(params.ran);
	int end = distr(params.ran);

	// Avoid that start and end coincide by accident
	while (end == start) end = distr(params.ran);

	// Copy from start to end
	int j = start;
	while (j % params.nbInTrucks != (end + 1) % params.nbInTrucks)
	{
		result.chromTO[j % params.nbInTrucks] = parent1.chromTO[j % params.nbInTrucks];
		freqClient[result.chromTO[j % params.nbInTrucks]] = true;
		j++;
	}
	//j is equal to end+1 now
	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <=params.nbInTrucks; i++)
	{
		int temp = parent2.chromTO[(end + i) % params.nbInTrucks];
		if (freqClient[temp] == false)
		{
			result.chromTO[j % params.nbInTrucks] = temp;
			j++;
		}
	}

	std::vector <bool> freqClientD = std::vector <bool>(params.nbOutTrucks, false);


	start = distr(params.ran);
	end = distr(params.ran);

	// Avoid that start and end coincide by accident
	while (end == start) end = distr(params.ran);

	// Copy from start to end
	j = start;
	while (j % params.nbOutTrucks != (end + 1) % params.nbOutTrucks)
	{
		result.chromTD[j % params.nbOutTrucks] = parent1.chromTD[j % params.nbOutTrucks];
		freqClientD[result.chromTD[j % params.nbOutTrucks]] = true;
		j++;
	}
	//j is equal to end now
	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <= params.nbOutTrucks; i++)
	{
		int temp = parent2.chromTD[(end + i) % params.nbOutTrucks];
		if (freqClientD[temp] == false)
		{
			result.chromTD[j % params.nbOutTrucks] = temp;
			j++;
		}
	}
	
	generateAssignemnt(result);
}


void Genetic::generateAssignemnt(Individual &result) {
	int j = 0;
	int cnt = 0;
	int restT = params.nbInTrucks;
	int num = 0;//record the number of trucks assigned to current door
	
	for (int i = 0; i < params.nbInDoors; i++) {
		num = (int)(restT / (params.nbInDoors - i));
		j = 0;
		while (j < num) {
			j++;
			result.chromRO[i][j] = result.chromTO[cnt];
			cnt++;
		}
		if (cnt<params.nbInTrucks && params.ITK[result.chromTO[cnt]].LF > params.ITK[result.chromTO[cnt - 1]].LF) {
			j++;
			result.chromRO[i][j] = result.chromTO[cnt];
			cnt++;
		}
		result.chromRO[i][0] = j;
		restT += -j;
	}
	if (cnt != params.nbInTrucks) { std::cout << "The Inassignment was not correct, check it. " << cnt << " " << params.nbInTrucks << std::endl; system("pause"); }
	
	cnt = 0;
	restT = params.nbOutTrucks;
	for (int i = 0; i < params.nbOutDoors; i++) {
		num = (int)(restT / (params.nbOutDoors - i));
		j = 0;
		while (j < num) {
			j++;
			result.chromRD[i][j] = result.chromTD[cnt];
			cnt++;
		}
		if (cnt<params.nbOutTrucks && params.OTK[result.chromTD[cnt]].LF > params.OTK[result.chromTD[cnt - 1]].LF) {
			j++;
			result.chromRD[i][j] = result.chromTD[cnt];
			cnt++;
		}
		result.chromRD[i][0] = j;
		restT += -j;
	}

	if (cnt != params.nbOutTrucks) { std::cout << "The Outassignment was not correct, check it. " << cnt << " " << params.nbOutTrucks << std::endl; system("pause"); }
}
Genetic::Genetic(Params & params) :
	params(params),
	saa(params),
	localSearch(params,this->saa),
	population(params, this->localSearch,this->saa),
	offspring(params) {}


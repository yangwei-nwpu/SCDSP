#include "Population.h"
//#include<iomanip>
void Population::generatePopulation()
{
	if (params.verbose) std::cout << "----- BUILDING INITIAL POPULATION -----" << std::endl;
	for (int i = 0; i < 2 * params.ap.mu && (i == 0 || params.ap.timeLimit == 0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit); i++)
	{
		Individual randomIndiv(params);
		localSearch.Iteraterun3(randomIndiv);
		addIndividual(randomIndiv, true);
	}
}
void Population::generatePopulationTradiSim()
{
	
	if (params.verbose) std::cout << "----- BUILDING INITIAL POPULATION -----" << std::endl;
	for (int i = 0; i < 2 * params.ap.mu && (i == 0 || params.ap.timeLimit == 0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit); i++)
	{
		Individual randomIndiv(params);
		localSearch.Iteraterun3TradiSim(randomIndiv);
		std::cout << "cost="<<randomIndiv.eval.penalizedCost << std::endl;
		addIndividual(randomIndiv, true);
		
	}
}
void Population::generatePopulationPR()
{
	
	if (params.verbose) std::cout << "----- BUILDING INITIAL POPULATION -----" << std::endl;
	for (int i = 0; i < 2 * params.ap.mu && (i == 0 || params.ap.timeLimit == 0 || (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit); i++)
	{
		Individual randomIndiv(params);
		localSearch.Iteraterun3(randomIndiv);
		addIndividualDirect(randomIndiv);
		SubPopulation & subpop = feasibleSubpop;
		while(subpop.size()> params.ap.mu)
			removeWorstBiasedFitness(subpop);
		int count = 0;
		for (Individual * myIndividual : subpop)
		{
			count++;
			myIndividual->posi = params.ap.mu + count;
		}
	}
}
bool Population::addIndividual(const Individual & indiv, bool updateFeasible)
{
	
	// Find the adequate subpopulation in relation to the individual feasibility
	SubPopulation & subpop = feasibleSubpop;
	
	// Create a copy of the individual and updade the proximity structures calculating inter-individual distances
	Individual * myIndividual = new Individual(indiv);
	for (Individual * myIndividual2 : subpop)
	{
		double myDistance = brokenPairsDistance(*myIndividual, *myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });//ordered
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	// Identify the correct location in the subpopulation and insert the individual
	int place = (int)subpop.size();
	while (place > 0 && subpop[place - 1]->eval.penalizedCost > indiv.eval.penalizedCost - MY_EPSILON) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);

	// Trigger a survivor selection if the maximimum subpopulation size is exceeded
	if ((int)subpop.size() > params.ap.mu + params.ap.lambda)
		while ((int)subpop.size() > params.ap.mu)
			removeWorstBiasedFitness(subpop);
	// Track best solution
	if (indiv.eval.penalizedCost < bestSolutionRestart.eval.penalizedCost - MY_EPSILON)
	{
		bestSolutionRestart = indiv; // Copy
		
		if (indiv.eval.penalizedCost < bestSolutionOverall.eval.penalizedCost - MY_EPSILON)
		{
			bestSolutionOverall = indiv;
			//bestSolutionOverall.eval.penalizedCost = indiv.eval.penalizedCost;
			SP tmp;
			tmp.SAALB = indiv.eval.penalizedCost;
			tmp.DECLB = indiv.eval.determineCost;
			tmp.INISimLB = indiv.eval.Ini_Sim_LB;
			tmp.INILB = indiv.eval.Ini_LB;
			tmp.time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;
			searchProgress.push_back(tmp);

			//searchProgress.push_back({ (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC, indiv.eval.penalizedCost });
		}
		return true;
	}
	else
		return false;
}

bool Population::addIndividualDirect(const Individual & indiv)
{
	SubPopulation & subpop = feasibleSubpop;

	// Create a copy of the individual and updade the proximity structures calculating inter-individual distances
	Individual * myIndividual = new Individual(indiv);
	
	
	for (Individual * myIndividual2 : subpop)
	{
		//double myDistance = NewDistance(*myIndividual, *myIndividual2);
		double myDistance = brokenPairsDistance(*myIndividual, *myIndividual2);
		myIndividual2->indivsPerProximity.insert({ myDistance, myIndividual });//ordered
		myIndividual->indivsPerProximity.insert({ myDistance, myIndividual2 });
	}

	// Identify the correct location in the subpopulation and insert the individual
	int place = (int)subpop.size();
	myIndividual->posi = place;//ith indiv inserted, if >mu indicates that this is a new indiv, vice versa
	while (place > 0 && subpop[place - 1]->eval.penalizedCost > indiv.eval.penalizedCost - MY_EPSILON) place--;
	subpop.emplace(subpop.begin() + place, myIndividual);

	
	// Track best solution
	if (indiv.eval.penalizedCost < bestSolutionRestart.eval.penalizedCost - MY_EPSILON)
	{
		std::cout << indiv.eval.penalizedCost << std::endl;
		bestSolutionRestart = indiv; // Copy
		if (indiv.eval.penalizedCost < bestSolutionOverall.eval.penalizedCost - MY_EPSILON)
		{
			bestSolutionOverall = indiv;
			SP tmp;
			tmp.SAALB = indiv.eval.penalizedCost;
			tmp.DECLB = indiv.eval.determineCost;
			tmp.INISimLB = indiv.eval.Ini_Sim_LB;
			tmp.INILB = indiv.eval.Ini_LB;
			tmp.time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;
			searchProgress.push_back(tmp);
		}
		return true;
	}
	else
		return false;
}

void Population::updateIndividual(const Individual & indiv)
{
	if (indiv.eval.penalizedCost < bestSolutionRestart.eval.penalizedCost - MY_EPSILON)
	{
		bestSolutionRestart = indiv; // Copy

		if (indiv.eval.penalizedCost < bestSolutionOverall.eval.penalizedCost - MY_EPSILON)
		{
			bestSolutionOverall = indiv;
			SP tmp;
			tmp.SAALB = indiv.eval.penalizedCost;
			tmp.DECLB = indiv.eval.determineCost;
			tmp.INISimLB = indiv.eval.Ini_Sim_LB;
			tmp.INILB = indiv.eval.Ini_LB;
			tmp.time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;
			searchProgress.push_back(tmp);
		}
	}
}

void Population::updateBiasedFitnesses(SubPopulation & pop)
{
	// Ranking the individuals based on their diversity contribution (decreasing order of distance)
	std::vector <std::pair <double, int> > ranking;
	for (int i = 0; i < (int)pop.size(); i++)
		ranking.push_back({ -averageBrokenPairsDistanceClosest(*pop[i],params.ap.nbClose),i });
	std::sort(ranking.begin(), ranking.end());

	// Updating the biased fitness values
	if (pop.size() == 1)
		pop[0]->biasedFitness = 0;
	else
	{
		for (int i = 0; i < (int)pop.size(); i++)
		{
			double divRank = (double)i / (double)(pop.size() - 1); // Ranking from 0 to 1
			double fitRank = (double)ranking[i].second / (double)(pop.size() - 1);
			if ((int)pop.size() <= params.ap.nbElite) // Elite individuals cannot be smaller than population size
				pop[ranking[i].second]->biasedFitness = fitRank;
			else
				pop[ranking[i].second]->biasedFitness = fitRank + (1.0 - (double)params.ap.nbElite / (double)pop.size()) * divRank;
		}
	}
}

void Population::removeWorstBiasedFitness(SubPopulation & pop)
{
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in HGS");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 1; i < (int)pop.size(); i++)
	{
		bool isClone = (averageBrokenPairsDistanceClosest(*pop[i], 1) < MY_EPSILON); // A distance equal to 0 indicates that a clone exists
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFitness > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFitness;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}

	// Removing the individual from the population and freeing memory
	pop.erase(pop.begin() + worstIndividualPosition);

	// Cleaning its distances from the other individuals in the population
	for (Individual * indiv2 : pop)
	{
		auto it = indiv2->indivsPerProximity.begin();
		while (it->second != worstIndividual) ++it;
		indiv2->indivsPerProximity.erase(it);
	}

	// Freeing memory
	delete worstIndividual;
}

void Population::removeWorstBiasedFitness2(SubPopulation & pop)
{
	updateBiasedFitnesses(pop);
	if (pop.size() <= 1) throw std::string("Eliminating the best individual: this should not occur in HGS");

	Individual * worstIndividual = NULL;
	int worstIndividualPosition = -1;
	bool isWorstIndividualClone = false;
	double worstIndividualBiasedFitness = -1.e30;
	for (int i = 1; i < (int)pop.size(); i++)
	{
		bool isClone = (averageBrokenPairsDistanceClosest(*pop[i], 1) < MY_EPSILON); // A distance equal to 0 indicates that a clone exists
		if ((isClone && !isWorstIndividualClone) || (isClone == isWorstIndividualClone && pop[i]->biasedFitness > worstIndividualBiasedFitness))
		{
			worstIndividualBiasedFitness = pop[i]->biasedFitness;
			isWorstIndividualClone = isClone;
			worstIndividualPosition = i;
			worstIndividual = pop[i];
		}
	}

	// Removing the individual from the population and freeing memory
	pop.erase(pop.begin() + worstIndividualPosition);

	// Cleaning its distances from the other individuals in the population
	for (Individual * indiv2 : pop)
	{
		auto it = indiv2->indivsPerProximity.begin();
		while (it->second != worstIndividual) ++it;
		indiv2->indivsPerProximity.erase(it);
	}

	// Freeing memory
	delete worstIndividual;
}

void Population::restart()
{
	if (params.verbose) std::cout << "----- RESET: CREATING A NEW POPULATION -----" << std::endl;
	for (Individual * indiv : feasibleSubpop) delete indiv;
	feasibleSubpop.clear();
	bestSolutionRestart.eval.penalizedCost=888888888;
	generatePopulation();
	std::cout << "restart " << getBestFeasible()->eval.penalizedCost << std::endl;
}
void Population::restartTradiSim()
{
	if (params.verbose) std::cout << "----- RESET: CREATING A NEW POPULATION -----" << std::endl;
	for (Individual * indiv : feasibleSubpop) delete indiv;
	feasibleSubpop.clear();
	bestSolutionRestart.eval.penalizedCost = 88888888888;
	generatePopulationTradiSim();
	std::cout << "restart " << getBestFeasible()->eval.penalizedCost << std::endl;
}

bool Population::manegement()//pr used
{
	bool updated = false;
	SubPopulation & subpop = feasibleSubpop;
	while ((int)subpop.size() > params.ap.mu)
		removeWorstBiasedFitness(subpop);
	for (Individual * indiv : subpop) {
		if (indiv->posi > params.ap.mu) {
			updated = true;
			break;
		}
	}
	if (!updated) {
		while ((int)subpop.size() > (0.5*params.ap.mu)) {
			removeWorstBiasedFitness(subpop);
		}
		generatePopulationPR();
	}
	return updated;
}


const Individual & Population::getBinaryTournament()
{
	// Picking two individuals with uniform distribution over the union of the feasible and infeasible subpopulations
	std::uniform_int_distribution<> distr(0, feasibleSubpop.size() - 1);
	int place1 = distr(params.ran);
	int place2 = distr(params.ran);
	while (place1 == place2) {
		place2 = distr(params.ran);
	}
	Individual * indiv1 = feasibleSubpop[place1];
	Individual * indiv2 = feasibleSubpop[place2];

	// Keeping the best of the two in terms of biased fitness
	updateBiasedFitnesses(feasibleSubpop);
	if (indiv1->biasedFitness < indiv2->biasedFitness) { 
		
		return *indiv1; }
	else { 
		return *indiv2; }
}

Individual * Population::getIndiv(int ith)
{
	if (!feasibleSubpop.empty()) return feasibleSubpop[ith];
	else return NULL;
}

const Individual * Population::getBestFeasible()
{
	if (!feasibleSubpop.empty()) return feasibleSubpop[0];
	else return NULL;
}

const Individual * Population::getBestFound()
{
	if (bestSolutionOverall.eval.penalizedCost < 999999999) return &bestSolutionOverall;
	else return NULL;
}

void Population::printState(int nbIter, int nbIterNoImprovement)
{
	if (params.verbose)
	{
		std::printf("It %6d %6d | T(s) %.2f ", nbIter, nbIterNoImprovement, (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC);
		if (getBestFeasible() != NULL) std::printf(" | Feas %zu %.1f %.2f", feasibleSubpop.size(), getBestFeasible()->eval.penalizedCost, getAverageCost(feasibleSubpop));
		else std::printf(" | NO-FEASIBLE");
		std::printf(" | Div %.2f", getDiversity(feasibleSubpop));
		std::cout << std::endl;
	}
}

double Population::brokenPairsDistance(const Individual & indiv1, const Individual & indiv2)
{
	int differences = 0;
	for (int j = 0; j < params.nbInTrucks; j++)
	{
		if (indiv1.successorsO[j] != indiv2.successorsO[j] && indiv1.successorsO[j] != indiv2.predecessorsO[j]) differences++;
		if (indiv1.predecessorsO[j] == -1 && indiv2.predecessorsO[j] != -1 && indiv2.successorsO[j] != -1) differences++;
	}
	for (int j = 0; j < params.nbOutTrucks; j++) {
		if (indiv1.successorsD[j] != indiv2.successorsD[j] && indiv1.successorsD[j] != indiv2.predecessorsD[j]) differences++;
		if (indiv1.predecessorsD[j] == -1 && indiv2.predecessorsD[j] != -1 && indiv2.successorsD[j] != -1) differences++;
	}
	return (double)differences / (double)(params.nbInTrucks + params.nbOutTrucks);
}

double Population::NewDistance(const Individual & indiv1, const Individual & indiv2)
{
	int mint = 0;
	int count = 0;
	for (int i = 0; i < params.nbInDoors; i++) {
		mint = std::min(indiv1.chromRO[i][0], indiv2.chromRO[i][0]);
		for (int j = 1; j <= mint; j++) {
			if (indiv1.chromRO[i][j]!= indiv2.chromRO[i][j]) { count++; }
		}
		if (indiv1.chromRO[i][0] > indiv2.chromRO[i][0]) { count += indiv1.chromRO[i][0] - indiv2.chromRO[i][0]; }
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		mint = std::min(indiv1.chromRD[i][0], indiv2.chromRD[i][0]);
		for (int j = 1; j <= mint; j++) {
			if (indiv1.chromRD[i][j] != indiv2.chromRD[i][j]) { count++; }
		}
		if (indiv1.chromRD[i][0] > indiv2.chromRD[i][0]) { count += indiv1.chromRD[i][0] - indiv2.chromRD[i][0]; }
	}
	return (double)count / (double)(params.nbInTrucks + params.nbOutTrucks);
}

double Population::averageBrokenPairsDistanceClosest(const Individual & indiv, int nbClosest)
{
	double result = 0.;
	int maxSize = std::min<int>(nbClosest, indiv.indivsPerProximity.size());
	auto it = indiv.indivsPerProximity.begin();
	for (int i = 0; i < maxSize; i++)
	{
		result += it->first;
		++it;
	}
	return result / (double)maxSize;
}

double Population::getDiversity(const SubPopulation & pop)
{
	double average = 0.;
	int size = std::min<int>(params.ap.mu, pop.size()); // Only monitoring the "mu" better solutions to avoid too much noise in the measurements
	for (int i = 0; i < size; i++) average += averageBrokenPairsDistanceClosest(*pop[i], size);
	if (size > 0) return average / (double)size;
	else return -1.0;
}

double Population::getAverageCost(const SubPopulation & pop)
{
	double average = 0.;
	int size = std::min<int>(params.ap.mu, pop.size()); // Only monitoring the "mu" better solutions to avoid too much noise in the measurements
	for (int i = 0; i < size; i++) { average += pop[i]->eval.penalizedCost; }
	if (size > 0) return average / (double)size;
	else return -1.0;
}

void Population::exportSearchProgress(std::string fileName, std::string instanceName)
{
	std::ofstream myfile(fileName, std::ios::app);
	for (SP state : searchProgress)
		myfile << instanceName << ";" << params.ap.seed << ";" << state.SAALB << ";" << state.DECLB << ";" << state.INISimLB << ";" << state.INILB << ";" << state.time<< std::endl;
	myfile << std::endl;
}

void Population::exportSearchTime(std::string fileName)
{
	std::ofstream myfile(fileName, std::ios::app);
	myfile << localSearch.SAATIME << ";" << localSearch.SimulationTime << ";" << localSearch.LSTIME << std::endl;
	myfile << std::endl;
}
void Population::exportAffectedScenarios(std::string fileName)
{
	std::ofstream myfile(fileName, std::ios::app);
	myfile << (double)(saa.RateAverageAffectedScenarios / (params.nbOutTrucks*saa.TotalUpdate))*100 << std::endl;//params.nbOutTrucks*
}
void Population::LargeSampleEvaluation()
{
	
	int cnt = 0;
	for (int i = 0; i < params.nbInDoors; i++) {
		cnt = 0;
		for (int j = 1; j <=bestSolutionOverall.chromRO[i][0]; j++) {
			if (bestSolutionOverall.StimeO[bestSolutionOverall.chromRO[i][j]] >= 0) {
				cnt++;
				saa.SequenceX[i][cnt] = bestSolutionOverall.chromRO[i][j];
				saa.stx[bestSolutionOverall.chromRO[i][j]] = bestSolutionOverall.StimeO[bestSolutionOverall.chromRO[i][j]];
				saa.solx[bestSolutionOverall.chromRO[i][j]] = i;
			}
			else {
				saa.solx[bestSolutionOverall.chromRO[i][j]] = -1;
				saa.stx[bestSolutionOverall.chromRO[i][j]] = -1;
			}
		}
		saa.SequenceX[i][0] = cnt;
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		cnt = 0;
		for (int k = 1; k <= bestSolutionOverall.chromRD[j][0]; k++) {
			if (bestSolutionOverall.StimeD[bestSolutionOverall.chromRD[j][k]] >= 0) {
				cnt++;
				saa.SequenceY[j][cnt] = bestSolutionOverall.chromRD[j][k];
				saa.sty[bestSolutionOverall.chromRD[j][k]] = bestSolutionOverall.StimeD[bestSolutionOverall.chromRD[j][k]];
				saa.soly[bestSolutionOverall.chromRD[j][k]] = j;
				saa.L[bestSolutionOverall.chromRD[j][k]] = cnt;
			}
			else {
				saa.soly[bestSolutionOverall.chromRD[j][k]] = -1;
				saa.L[bestSolutionOverall.chromRD[j][k]] = -1;
				saa.sty[bestSolutionOverall.chromRD[j][k]] = -1;
			}
		}
		saa.SequenceY[j][0] = cnt;
	}
	saa.generate_EST();
	saa.SGS2Small(saa.SmpNum, saa.sty);
	std::cout << "before:" << std::fixed << std::setprecision(2) << bestSolutionOverall.eval.penalizedCost << "      after:" << saa.local_best << std::endl;
	bestSolutionOverall.eval.penalizedCost = saa.local_best;
	for (Individual * indiv : feasibleSubpop) {
		for (int i = 0; i < params.nbInDoors; i++) {
			cnt = 0;
			for (int j = 1; j <= indiv->chromRO[i][0]; j++) {
				if (indiv->StimeO[indiv->chromRO[i][j]] >= 0) {
					cnt++;
					saa.SequenceX[i][cnt] = indiv->chromRO[i][j];
					saa.stx[indiv->chromRO[i][j]] = indiv->StimeO[indiv->chromRO[i][j]];
					saa.solx[indiv->chromRO[i][j]] = i;
				}
				else {
					saa.solx[indiv->chromRO[i][j]] = -1;
					saa.stx[indiv->chromRO[i][j]] = -1;
				}
			}
			saa.SequenceX[i][0] = cnt;
		}
		for (int j = 0; j < params.nbOutDoors; j++) {
			cnt = 0;
			for (int k = 1; k <= indiv->chromRD[j][0]; k++) {
				if (indiv->StimeD[indiv->chromRD[j][k]] >= 0) {
					cnt++;
					saa.SequenceY[j][cnt] = indiv->chromRD[j][k];
					saa.sty[indiv->chromRD[j][k]] = indiv->StimeD[indiv->chromRD[j][k]];
					saa.soly[indiv->chromRD[j][k]] = j;
					saa.L[indiv->chromRD[j][k]] = cnt;
				}
				else {
					saa.soly[indiv->chromRD[j][k]] = -1;
					saa.L[indiv->chromRD[j][k]] = -1;
					saa.sty[indiv->chromRD[j][k]] = -1;
				}
			}
			saa.SequenceY[j][0] = cnt;
		}
		saa.generate_EST();
		saa.SGS2Small(saa.SmpNum, saa.sty);
		indiv->eval.penalizedCost = saa.local_best;
		if (indiv->eval.penalizedCost < bestSolutionOverall.eval.penalizedCost - MY_EPSILON) {
			bestSolutionOverall = *indiv;
		}
	}
}

void Population::exportCVRPLibFormat(const Individual & indiv, std::string fileName, std::string fileName2, std::string filename3)
{
	std::ofstream myfile(fileName,std::ios::app);
	if (myfile.is_open())
	{
		for (int k = 0; k < params.nbInDoors; k++)
		{

			myfile << "InDoor #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (int i = 1; i <= indiv.chromRO[k][0]; i++) myfile << " " << indiv.chromRO[k][i] << "(" << indiv.StimeO[indiv.chromRO[k][i]] << ")";
			myfile << std::endl;
		}
		for (int k = 0; k < params.nbOutDoors; k++)
		{

			myfile << "OuDoor #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (int i = 1; i <= indiv.chromRD[k][0]; i++) myfile << " " << indiv.chromRD[k][i] << "(" << indiv.StimeD[indiv.chromRD[k][i]] << ")";
			myfile << std::endl;
		}
		myfile << "Cost " << std::fixed << std::setprecision(2) << indiv.eval.penalizedCost << std::endl;
		myfile << "DetCost " << indiv.eval.determineCost << std::endl;
		myfile << "IniSim " << indiv.eval.Ini_Sim_LB<< std::endl;
		myfile << "IniDetCost " << indiv.eval.Ini_LB << std::endl;
		myfile << "Time " << std::fixed << std::setprecision(2)<<searchProgress.back().time << std::endl;
		int cnt = 0;
		for (int i = 0; i < params.nbInDoors; i++) {
			cnt = 0;
			for (int j = 1; j <= indiv.chromRO[i][0]; j++) {
				if (indiv.StimeO[indiv.chromRO[i][j]] >= 0) {
					cnt++;
					saa.SequenceX[i][cnt] = indiv.chromRO[i][j];
					saa.stx[indiv.chromRO[i][j]] = indiv.StimeO[indiv.chromRO[i][j]];
					saa.solx[indiv.chromRO[i][j]] = i;
				}
				else {
					saa.solx[indiv.chromRO[i][j]] = -1;
					saa.stx[indiv.chromRO[i][j]] = -1;
				}
			}
			saa.SequenceX[i][0] = cnt;
		}
		for (int j = 0; j < params.nbOutDoors; j++) {
			cnt = 0;
			for (int k = 1; k <= indiv.chromRD[j][0]; k++) {
				if (indiv.StimeD[indiv.chromRD[j][k]] >= 0) {
					cnt++;
					saa.SequenceY[j][cnt] = indiv.chromRD[j][k];
					saa.sty[indiv.chromRD[j][k]] = indiv.StimeD[indiv.chromRD[j][k]];
					saa.soly[indiv.chromRD[j][k]] = j;
					saa.L[indiv.chromRD[j][k]] = cnt;
				}
				else {
					saa.soly[indiv.chromRD[j][k]] = -1;
					saa.L[indiv.chromRD[j][k]] = -1;
					saa.sty[indiv.chromRD[j][k]] = -1;
				}
			}
			saa.SequenceY[j][0] = cnt;
		}
		double average_csot = 0.0;
		double best_simcost = 999999999;
		double worstsimcost = 0.0;
		double testcost = 0.0;
		double GroundTruthTardiness = 0;
		double GroundTruthPenalty = 0;
		std::ofstream myfile2(fileName2, std::ios::app);
		for (int x = 1; x <= 20; x++) {
			long long int _test_cost = 0;
			saa.SmpNum = 1000;
			saa.generate_sample2(x, 0);
			saa.generate_EST();
			saa.sortSenario();
			saa.SGS2_profiles(_test_cost, saa.sty,filename3);
			testcost = (double)(_test_cost*1.0 / saa.SmpNum) + params.TotalPenality;
			if (worstsimcost < testcost - 1e-6) { worstsimcost = testcost; }
			if (best_simcost > testcost + 1e-6) { best_simcost = testcost; }
			average_csot += testcost;
			GroundTruthTardiness += saa.AvgTotalTardiness / (saa.SmpNum * params.alpha);
			GroundTruthPenalty += saa.AvgTotalPenaltyCost / saa.SmpNum;
			myfile2 << "Sim " << std::fixed << std::setprecision(2) << testcost << std::endl;
		}
		myfile2 << "Bst " << std::fixed << std::setprecision(2) << best_simcost << std::endl;
		myfile2 << "Wst " << std::fixed << std::setprecision(2) << worstsimcost << std::endl;
		myfile2 << "Avg " << std::fixed << std::setprecision(2) << average_csot / 20.0 << std::endl;
		myfile2 << "Tardiness " << std::fixed << std::setprecision(2) << GroundTruthTardiness / 20.0 << std::endl;
		myfile2 << "Penalty " << std::fixed << std::setprecision(2) << GroundTruthPenalty / 20.0 << std::endl;
		myfile2.close();
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
}

Population::Population(Params & params, LocalSearch & localSearch,SAA & saa ) 
	: params(params),  localSearch(localSearch), bestSolutionRestart(params), bestSolutionOverall(params),saa(saa)
{}


Population::~Population()
{
	for (int i = 0; i < (int)feasibleSubpop.size(); i++) delete feasibleSubpop[i];
}
#include "Params.h"
Params::Params(
	int * AT_O, int * AT_D,
	int * ES_O, int * ES_D,
	int * EF_O, int * EF_D,
	int * LS_O, int * LS_D,
	int * LF_O, int * LF_D,
	int * GT_O, int * GT_D,
	int ** Dis, int ** _Time,
	int ** PP, int ** Flow,
	int ** Suc, int**Pre,
	int NumTruck_O,										// Number of clients (excluding the depot)
	int NumTruck_D,
	int NumDoor_O,
	int NumDoor_D,
	int MAXDLINE,
	bool verbose,
	const AlgorithmParameters& ap
)
	: ap(ap), verbose(verbose), MAXDLINE(MAXDLINE), nbInTrucks(NumTruck_O), nbOutTrucks(NumTruck_D), nbInDoors(NumDoor_O), nbOutDoors(NumDoor_D)
{
	// This marks the starting time of the algorithm
	startTime = clock();

	penaltyTW = 10;
	gammaTW = 1.0;
	gammaWT = 0.2;
	// Initialize RNG
	ran.seed(ap.seed);

	ITK = std::vector<TruckO>(nbInTrucks);
	OTK = std::vector<TruckD>(nbOutTrucks);
	for (int i = 0; i < nbInTrucks; i++)
	{
		ITK[i].AT = AT_O[i];
		ITK[i].ES = ES_O[i];
		ITK[i].EF = EF_O[i];
		ITK[i].LS = LS_O[i];
		ITK[i].LF = LF_O[i];
		ITK[i].GT = GT_O[i];
		ITK[i].Suc = new int[nbOutTrucks+1];
		ITK[i].Suc[0] = Suc[i][0];
		for (int j = 1; j <= Suc[i][0]; j++) {
			ITK[i].Suc[j] = Suc[i][j];
		}
	}
	for (int i = 0; i < nbOutTrucks; i++)
	{
		OTK[i].AT = AT_D[i];
		OTK[i].ES = ES_D[i];
		OTK[i].EF = EF_D[i];
		OTK[i].LS = LS_D[i];
		OTK[i].LF = LF_D[i];
		OTK[i].GT = GT_D[i];
		OTK[i].Pre = new int[nbInTrucks+1];
		OTK[i].Pre[0] = Pre[i][0];
		for (int j = 1; j <= Pre[i][0]; j++) {
			OTK[i].Pre[j] = Pre[i][j];
		}
	}
	this->Dis = std::vector<std::vector<int> >(nbInDoors);
	for (int i = 0; i < nbInDoors; i++)
		for (int j = 0; j < nbOutDoors; j++)
			this->Dis[i].push_back(Dis[i][j]);
	this->Time = std::vector<std::vector<int> >(nbInTrucks);
	this->Flow = std::vector<std::vector<int> >(nbInTrucks);
	this->PP = std::vector<std::vector<int> >(nbInTrucks);
	for (int i = 0; i < nbInTrucks; i++)
		for (int j = 0; j < nbOutTrucks; j++)
		{
			this->Time[i].push_back(_Time[i][j]);
			this->Flow[i].push_back(Flow[i][j]);
			this->PP[i].push_back(PP[i][j]);
		}
	// Calculation of the correlated vertices for each customer (for the granular restriction)
	correlatedVerticesO = std::vector<std::vector<int> >(nbInTrucks);
	std::vector<std::set<int> > setCorrelatedVertices = std::vector<std::set<int> >(nbInTrucks);//
	std::vector<std::pair<double, int> > orderProximity;
	double tmpCL = 0.;
	for (int i = 0; i < nbInTrucks; i++)
	{
		orderProximity.clear();
		for (int j = 0; j < nbInTrucks; j++)
			if (i != j) {
				tmpCL = 1.0*gammaWT *  std::max(ITK[i].AT - ITK[j].LF, 0) + 1.0*gammaTW *  std::max(ITK[j].AT + ITK[j].GT - ITK[i].LS, 0);
				orderProximity.emplace_back(tmpCL, j);
			}
		std::sort(orderProximity.begin(), orderProximity.end());
		for (int j = 0; j < std::min<int>(ap.nbGranular, nbInTrucks-1); j++)
		{
			setCorrelatedVertices[i].insert(orderProximity[j].second);
		}
	}
	// Filling the vector of correlated vertices
	for (int i = 0; i < nbInTrucks; i++)
		for (int x : setCorrelatedVertices[i])
			correlatedVerticesO[i].push_back(x);

	correlatedVerticesD = std::vector<std::vector<int> >(nbOutTrucks);
	std::vector<std::set<int> > setCorrelatedVerticesD = std::vector<std::set<int> >(nbOutTrucks);
	for (int i = 0; i < nbOutTrucks; i++)
	{
		orderProximity.clear();
		for (int j = 0; j < nbOutTrucks; j++)
			if (i != j) {
				tmpCL = 1.0*gammaTW * std::max(OTK[j].AT - OTK[i].LF, 0) + 1.0*gammaWT * std::max(OTK[i].AT + ITK[i].GT - ITK[j].LS, 0);
				orderProximity.emplace_back(tmpCL, j);
			}
		std::sort(orderProximity.begin(), orderProximity.end());
		for (int j = 0; j < std::min<int>(ap.nbGranular, nbOutTrucks-1); j++)
		{
			setCorrelatedVerticesD[i].insert(orderProximity[j].second);
		}
	}
	// Filling the vector of correlated vertices
	for (int i = 0; i < nbOutTrucks; i++)
		for (int x : setCorrelatedVerticesD[i])
			correlatedVerticesD[i].push_back(x);
	// Safeguards to avoid possible numerical instability in case of instances containing arbitrarily small or large numerical values
	MAXOTime = 0;
	for (int i = 0; i < nbInTrucks; i++) {
		MAXOTime += ITK[i].LF - ITK[i].EF;
	}
	for (int i = 0; i < nbOutTrucks; i++) {
		MAXOTime += OTK[i].LF - OTK[i].EF;
	}
	MAXOTime = MAXOTime * alpha;
	funOTO = new int*[nbInTrucks];
	funOTD = new int*[nbOutTrucks];
	funTrans = new int***[nbInTrucks];
	for (int i = 0; i < nbInTrucks; i++) {
		funOTO[i] = new int[MAXDLINE + 1];
		for (int j = 0; j <= MAXDLINE; j++) {
			if (j <= ITK[i].ES) {
				funOTO[i][j] = 0;
			}
			else if (j <= ITK[i].LS) {
				funOTO[i][j] = j - ITK[i].ES;//The time the truck left over
			}
			else {
				funOTO[i][j] = 0;
			}
			funOTO[i][j] = funOTO[i][j] * alpha;
		}
		funTrans[i] = new int**[nbOutTrucks];
		for (int j = 0; j < nbOutTrucks; j++) {
			funTrans[i][j] = new int*[MAXDLINE + 1];
			for (int k = 0; k < MAXDLINE + 1; k++) {
				funTrans[i][j][k] = new int[MAXDLINE + 1];
				for (int l = 0; l < MAXDLINE + 1; l++) {//k < ITK[i].AT
					if (k + ITK[i].GT > l || k > ITK[i].LS) {
						funTrans[i][j][k][l] = 0;
					}
					else {
						funTrans[i][j][k][l] = 1;
					}
				}
			}
		}
	}
	for (int i = 0; i < nbOutTrucks; i++) {
		funOTD[i] = new int[MAXDLINE + 1];
		for (int j = 0; j < MAXDLINE + 1; j++) {
			if (j <= OTK[i].ES) {
				funOTD[i][j] = 0;
			}
			else if (j <= OTK[i].LS) {
				funOTD[i][j] = j - OTK[i].ES;//The time the truck left over
			}
			else {
				funOTD[i][j] = 1000000;
			}
			funOTD[i][j] = funOTD[i][j] * alpha;
		}
	}
	for (int i = 0; i < nbInTrucks; i++) {
		for (int j = 0; j < nbOutTrucks; j++) {
			TotalPenality += this->PP[i][j] * this->Flow[i][j];
		}
	}
	miniTep = std::min(int(0.05*MAXOTime)+1, 10);
	/************  initial FP=PP[i][j]*Flow[i][j]    **********/
	FP = new int*[nbInTrucks];
	for (int i = 0; i < nbInTrucks; i++) {
		FP[i] = new int[nbOutTrucks];
		for (int j = 0; j < nbOutTrucks; j++) {
			FP[i][j] = 0;
			if (Flow[i][j] > 0) {
				FP[i][j] = Flow[i][j] * PP[i][j];
			}
		}
	}
	if (verbose)
		std::cout << "----- INSTANCE SUCCESSFULLY LOADED WITH " << nbInTrucks << " TRUCKS AND " << nbInDoors << " DOORS" << std::endl;
}



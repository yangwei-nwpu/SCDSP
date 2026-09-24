#include "Individual.h" 

void Individual::evaluateCompleteCost(const Params & params)
{
	eval = EvalIndiv();
	int otk = 0;
	int transtime = 0;
	eval.penalizedTransCost = 0;
	eval.WTime = 0;
	eval.TimeWarp = 0;
	for (int r = 0; r < params.nbInTrucks; r++)
	{
		if (StimeO[r] <= params.ITK[r].LS &&StimeO[r] >= params.ITK[r].AT) {
			eval.WTime += params.funOTO[r][StimeO[r]];

			for (int j = 1; j <= params.ITK[r].Suc[0]; j++) {
				otk = params.ITK[r].Suc[j];
				transtime = params.Time[r][otk] * params.Dis[AssignO[r]][AssignD[otk]];
				//std::cout << r << " " << otk << " " << StimeO[r]<<" "<< StimeD[otk] << " " << transtime << std::endl;
				if (StimeD[otk] >= params.OTK[otk].AT&&StimeD[otk] >= transtime) {

					eval.penalizedTransCost += params.FP[r][otk] * params.funTrans[r][params.ITK[r].Suc[j]][StimeO[r]][StimeD[otk] - transtime];
				}
			}
		}
		else { eval.TimeWarp += params.ITK[r].GT; }
	}
	
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (StimeD[i] >= params.OTK[i].AT) {
			eval.WTime += params.funOTD[i][StimeD[i]];
		}
		else{ eval.TimeWarp += params.OTK[i].GT; }
	}
	//std::cout << eval.WTime << " " << params.TotalPenality << " " << eval.penalizedTransCost << std::endl;
	eval.determineCost = eval.WTime + params.TotalPenality - eval.penalizedTransCost;
	eval.penalizedCost = eval.determineCost;
	//std::cout << "params.TotalPenality " << params.TotalPenality << std::endl;
	eval.isFeasible = (eval.TimeWarp > MY_EPSILON);
}
void Individual::evaluateCompleteCost2(const Params & params)
{
	eval = EvalIndiv();
	//int otk = 0;
	//int transtime = 0;
	eval.penalizedTransCost = 0;
	eval.WTime = 0;
	eval.TimeWarp = 0;
	for (int r = 0; r < params.nbInTrucks; r++)
	{
		eval.WTime += penaltyWTimeO[r];
		if (StimeO[r] <= params.ITK[r].LS) {

			eval.penalizedTransCost += penaltyTrans[r];

		}
		else { eval.TimeWarp += params.ITK[r].GT; }
	}

	for (int i = 0; i < params.nbOutTrucks; i++) {
		eval.WTime += penaltyWTimeD[i];
		if (StimeD[i] < params.OTK[i].AT) { eval.TimeWarp += params.OTK[i].GT; }
	}
	//std::cout << eval.WTime << " " << params.TotalPenality << " " << eval.penalizedTransCost << std::endl;
	eval.determineCost= eval.WTime + params.TotalPenality + eval.penalizedTransCost;
	eval.penalizedCost = eval.determineCost;
	eval.isFeasible = (eval.TimeWarp > MY_EPSILON);
	//system("pause");
}
Individual::Individual(Params & params)//generate rondom assignment of trucks,  the trucks are then sorted by their start time
{
	for (int i = 0; i < params.nbInTrucks; i++) chromTO[i] = params.ran()%params.nbInDoors;//generate the initial solution
	//int *cnt = new int[params.nbInDoors];
	//for (int i = 0; i < params.nbInDoors; i++) { cnt[i] = 0; }
	std::vector<std::pair<int, int> > orderP;
	//std::cout << "in door" << std::endl;
	for (int i = 0; i < params.nbInDoors; i++) {
		orderP.clear();
		for (int j = 0; j < params.nbInTrucks; j++) {
			if (i == chromTO[j]) { orderP.emplace_back(params.ITK[j].AT, j); }
		}
		std::sort(orderP.begin(), orderP.end());
		chromRO[i][0] = orderP.size();
		for (int j = 1; j <= chromRO[i][0]; j++) {
			chromRO[i][j] = orderP[j - 1].second;
			//std::cout << chromRO[i][j] << " "; 
		}
		//std::cout << std::endl;
	}
	//std::shuffle(chromT, chromT+params.nbInTrucks, params.ran);
	for (int i = 0; i < params.nbOutTrucks; i++) chromTD[i] = params.ran() % params.nbOutDoors;
	//std::cout << "out door" << std::endl;
	for (int i = 0; i < params.nbOutDoors; i++) {
		orderP.clear();
		for (int j = 0; j < params.nbOutTrucks; j++) {
			if (i == chromTD[j]) { orderP.emplace_back(params.OTK[j].AT, j); }
		}
		std::sort(orderP.begin(), orderP.end());
		chromRD[i][0] = orderP.size();
		for (int j = 1; j <= chromRD[i][0]; j++) {
			chromRD[i][j] = orderP[j - 1].second;
			//std::cout << chromRD[i][j] << " ";
		}
		//std::cout << std::endl;
	}
	eval.penalizedCost = 999999999;
}

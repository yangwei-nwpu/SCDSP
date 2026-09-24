#include "LocalSearch.h" 
using namespace std;

bool LocalSearch::runO3()
{

	bool isfound = false;
	searchCompleted = false;
	bool founded = false;
	bool bestIntra = false;
	int mindelta = 0;
	int tmpdelta = 0;
	int MaxCnt = params.nbInTrucks*params.ap.nbGranular*params.ap.rato;
	int cnt = 0;
	Node *SltU;
	Node *SltV;
	//shift1
	//moveO 4
	for (loopID = 0; !searchCompleted && (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit; loopID++)
	{
		searchCompleted = true;
		founded = false;
		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		//swap 1
		mindelta = 0;
		cnt = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex)continue;
					tmpdelta = delta_swapO(nodeU, nodeV);
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeU == nodeV->prev || nodeXIndex == -1 || nodeX == nodeV->prev || nodeXIndex == nodeVIndex || nodeYIndex == nodeUIndex) continue;
					tmpdelta = delta_swapO2T1();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded&&cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex || nodeXIndex == -1 || nodeYIndex == -1 || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)continue;
					tmpdelta = delta_swapO2T2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}

		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			nodeY = SltV->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			swapNode(nodeX, nodeY, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					tmpdelta = delta_shiftO();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}

		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex) continue; // RELOCATE
					tmpdelta = delta_shiftO2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO2();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;

		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_2shiftO();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_2shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;


		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_shiftO2();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_2shiftO();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
								/*if (cnt >= MaxCnt) {
									break;
								}*/
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
	}

	if (loopID >= 2) { isfound = true; }
	return isfound;
}
bool LocalSearch::runD3()
{

	bool isfound = false;
	searchCompleted = false;
	bool founded = false;
	bool bestIntra = false;
	int mindelta = 0;
	int tmpdelta = 0;
	int MaxCnt = params.nbOutTrucks*params.ap.nbGranular*params.ap.rato;
	int cnt = 0;
	Node *SltU;
	Node *SltV;
	//shift1
	for (loopID = 0; !searchCompleted && (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit; loopID++)
	{
		searchCompleted = true;
		founded = false;
		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		//swap 1
		mindelta = 0;
		cnt = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex)continue;
					tmpdelta = delta_swapD(nodeU, nodeV);
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeXIndex == -1 || nodeX == nodeV->prev || nodeXIndex == nodeVIndex || nodeYIndex == nodeUIndex) continue;
					tmpdelta = delta_swapD2T1();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded&&cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex || nodeXIndex == -1 || nodeYIndex == -1 || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)continue;
					tmpdelta = delta_swapD2T2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}

		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			nodeY = SltV->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			swapNode(nodeX, nodeY, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		//moveD1
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_shiftD();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		//moveD2
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex) continue; // RELOCATE
					tmpdelta = delta_shiftD2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD2();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		//moveD3
		cnt = 0;
		mindelta = 0;

		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_2shiftD();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_2shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}


		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_shiftD2();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			//cout << "in D8 ";
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_2shiftD();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch();
			continue;
		}
	}
	if (loopID >= 2) { isfound = true; }
	return isfound;
}
bool LocalSearch::runO3TradiSim()
{

	bool isfound = false;
	searchCompleted = false;
	bool founded = false;
	bool bestIntra = false;
	int mindelta = 0;
	int tmpdelta = 0;
	int MaxCnt = params.nbInTrucks*params.ap.nbGranular*params.ap.rato;
	int cnt = 0;
	Node *SltU;
	Node *SltV;
	//shift1
	//moveO 4
	for (loopID = 0; !searchCompleted; loopID++)
	{
		searchCompleted = true;
		founded = false;
		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		//swap 1
		mindelta = 0;
		cnt = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex)continue;
					tmpdelta = delta_swapO(nodeU, nodeV);
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeU == nodeV->prev || nodeXIndex == -1 || nodeX == nodeV->prev || nodeXIndex == nodeVIndex || nodeYIndex == nodeUIndex) continue;
					tmpdelta = delta_swapO2T1();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded&&cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex || nodeXIndex == -1 || nodeYIndex == -1 || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)continue;
					tmpdelta = delta_swapO2T2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}

		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			nodeY = SltV->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			swapNode(nodeX, nodeY, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_shiftO();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}

		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex) continue; // RELOCATE
					tmpdelta = delta_shiftO2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO2();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;

		for (int posU = 0; posU < params.nbInTrucks; posU++)
		{
			nodeU = &trucksO[orderTrucksO[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesO[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksO[params.correlatedVerticesO[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_2shiftO();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_2shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;


		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						//std::cout << "empty door O" << std::endl;
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftO();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_shiftO2();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbInDoors; posDoor++) {
			if (doorsO[posDoor].nbTrucks == 0) {
				nodeV = doorsO[posDoor].head;
				for (int posU = 0; posU < params.nbInTrucks; posU++)
				{
					nodeU = &trucksO[orderTrucksO[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_2shiftO();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewO();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataO(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataO(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
	}

	if (loopID >= 2) { isfound = true; }
	return isfound;
}
bool LocalSearch::runD3TradiSim()
{

	bool isfound = false;
	searchCompleted = false;
	bool founded = false;
	bool bestIntra = false;
	int mindelta = 0;
	int tmpdelta = 0;
	int MaxCnt = params.nbOutTrucks*params.ap.nbGranular*params.ap.rato;
	int cnt = 0;
	Node *SltU;
	Node *SltV;
	//shift1
	for (loopID = 0; !searchCompleted; loopID++)
	{
		//if (loopID > 1) // Allows at least two loops since some moves involving empty routes are not checked at the first loop
		searchCompleted = true;
		founded = false;
		/* CLASSICAL ROUTE IMPROVEMENT (RI) MOVES SUBJECT TO A PROXIMITY RESTRICTION */
		//swap 1
		mindelta = 0;
		cnt = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex)continue;
					tmpdelta = delta_swapD(nodeU, nodeV);
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeXIndex == -1 || nodeX == nodeV->prev || nodeXIndex == nodeVIndex || nodeYIndex == nodeUIndex) continue;
					tmpdelta = delta_swapD2T1();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded&&cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex > nodeVIndex || nodeXIndex == -1 || nodeYIndex == -1 || nodeY == nodeU->prev || nodeU == nodeY || nodeX == nodeV || nodeV == nodeX->next)continue;
					tmpdelta = delta_swapD2T2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}

		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			nodeY = SltV->next;
			routeU = SltU->route;
			routeV = SltV->route;
			swapNode(SltU, SltV, bestIntra);
			swapNode(nodeX, nodeY, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		//moveD1
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_shiftD();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		//moveD2
		cnt = 0;
		mindelta = 0;
		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex) continue; // RELOCATE
					tmpdelta = delta_shiftD2();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD2();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		//moveD3
		cnt = 0;
		mindelta = 0;

		for (int posU = 0; posU < params.nbOutTrucks; posU++)
		{
			nodeU = &trucksD[orderTrucksD[posU]];
			int lastTestRINodeU = nodeU->whenLastTestedRI;
			nodeU->whenLastTestedRI = nbMoves;
			for (int posV = 0; posV < (int)params.correlatedVerticesD[nodeU->cour].size(); posV++)
			{
				nodeV = &trucksD[params.correlatedVerticesD[nodeU->cour][posV]];
				if (loopID == 0 || std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU) // only evaluate moves involving routes that have been modified since last move evaluations for nodeU
				{
					// Randomizing the order of the neighborhoods within this loop does not matter much as we are already randomizing the order of the node pairs (and it's not very common to find improving moves of different types for the same node pair)
					setLocalVariablesRouteU();
					setLocalVariablesRouteV();
					if (nodeUIndex == nodeYIndex || nodeXIndex == -1 || nodeXIndex == nodeVIndex)continue;
					tmpdelta = delta_2shiftD();
					if (tmpdelta < mindelta) {
						mindelta = tmpdelta;
						SltU = nodeU;
						SltV = nodeV;
						bestIntra = intraRouteMove;
						updatechange();
						founded = true;
						if (cnt >= MaxCnt) {
							break;
						}
					}
					cnt++;
					if (nodeV->prev->ishead) {
						nodeV = nodeV->prev;
						setLocalVariablesRouteV();
						tmpdelta = delta_2shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
							if (cnt >= MaxCnt) {
								break;
							}
						}
					}
				}
			}
			if (founded && cnt >= MaxCnt) { break; }
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}


		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();
						tmpdelta = delta_shiftD();
						if (tmpdelta < mindelta) {
							mindelta = tmpdelta;
							SltU = nodeU;
							SltV = nodeV;
							bestIntra = intraRouteMove;
							updatechange();
							founded = true;
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_shiftD2();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(SltU, SltV, bestIntra);
			insertNode(nodeX, SltU, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
		cnt = 0;
		mindelta = 0;
		for (int posDoor = 0; posDoor < params.nbOutDoors; posDoor++) {
			if (doorsD[posDoor].nbTrucks == 0) {
				nodeV = doorsD[posDoor].head;
				for (int posU = 0; posU < params.nbOutTrucks; posU++)
				{
					nodeU = &trucksD[orderTrucksD[posU]];
					int lastTestRINodeU = nodeU->whenLastTestedRI;
					nodeU->whenLastTestedRI = nbMoves;
					if (std::max<int>(nodeU->route->whenLastModified, nodeV->route->whenLastModified) > lastTestRINodeU)//empty door
					{
						setLocalVariablesRouteU();
						setLocalVariablesRouteV();

						if (nodeXIndex != -1) {
							tmpdelta = delta_2shiftD();
							if (tmpdelta < mindelta) {
								mindelta = tmpdelta;
								SltU = nodeU;
								SltV = nodeV;
								bestIntra = intraRouteMove;
								updatechange();
								founded = true;
							}
						}
					}
				}
			}
		}
		if (founded) {
			determinedCost += mindelta;
			nodeX = SltU->next;
			routeU = SltU->route;
			routeV = SltV->route;
			insertNode(nodeX, SltV, bestIntra);
			insertNode(SltU, nodeX, bestIntra);
			updateTrucksNewD();
			nbMoves++; // Increment move counter before updating route data
			searchCompleted = false;
			updateRouteDataD(routeU, bestchangedNbTrucks[routeU->cour]);
			if (!bestIntra) updateRouteDataD(routeV, bestchangedNbTrucks[routeV->cour]);
			narrowSearch2();
			continue;
		}
	}
	if (loopID >= 2) { isfound = true; }
	return isfound;
}

void LocalSearch::run3(Individual & indiv) {

	determinedCost = 0;
	loadIndividual(indiv);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksO.begin(), orderTrucksO.end(), params.ran);
	std::shuffle(orderDoorsO.begin(), orderDoorsO.end(), params.ran);
	for (int i = 0; i < params.nbInTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesO[i].begin(), params.correlatedVerticesO[i].end(), params.ran);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksD.begin(), orderTrucksD.end(), params.ran);
	std::shuffle(orderDoorsD.begin(), orderDoorsD.end(), params.ran);
	for (int i = 0; i < params.nbOutTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesD[i].begin(), params.correlatedVerticesD[i].end(), params.ran);
	runO3();
	for (int r = 0; r < params.nbOutDoors; r++) {
		Node*tmp = headsD[r].next;//start node
		Route *Door = &doorsD[r];
		Door->penalty = 0;
		while (!tmp->ishead) {
			tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
			Door->penalty += tmp->TransCost;
			tmp = tmp->next;
		}
	}
	int cnt = 0;
	while ((double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit) {
		if (!runD3()) {
			break;
		}
		for (int r = 0; r < params.nbInDoors; r++) {
			Node*tmp = headsO[r].next;//start node
			Route *Door = &doorsO[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeIncost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
		if (!runO3()) {
			break;
		}
		for (int r = 0; r < params.nbOutDoors; r++) {
			Node*tmp = headsD[r].next;//start node
			Route *Door = &doorsD[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
	}
	exportIndividualStoch(indiv);
}
void LocalSearch::Iteraterun3(Individual & indiv) {

	determinedCost = 0;
	local_best = 999999999;
	loadIndividual(indiv);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksO.begin(), orderTrucksO.end(), params.ran);
	std::shuffle(orderDoorsO.begin(), orderDoorsO.end(), params.ran);
	for (int i = 0; i < params.nbInTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesO[i].begin(), params.correlatedVerticesO[i].end(), params.ran);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksD.begin(), orderTrucksD.end(), params.ran);
	std::shuffle(orderDoorsD.begin(), orderDoorsD.end(), params.ran);
	for (int i = 0; i < params.nbOutTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesD[i].begin(), params.correlatedVerticesD[i].end(), params.ran);
	runO3();
	for (int r = 0; r < params.nbOutDoors; r++) {
		Node*tmp = headsD[r].next;//start node
		Route *Door = &doorsD[r];
		Door->penalty = 0;
		while (!tmp->ishead) {
			tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
			Door->penalty += tmp->TransCost;
			tmp = tmp->next;
		}
	}
	int cnt = 0;
	while (cnt <= params.ap.maxrestart && (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit) {
		if (!runD3()) {
			perturbD();
			cnt++;
		}
		for (int r = 0; r < params.nbInDoors; r++) {
			Node*tmp = headsO[r].next;//start node
			Route *Door = &doorsO[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeIncost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
		if (!runO3()) {
			perturbO();
			cnt++;
		}
		for (int r = 0; r < params.nbOutDoors; r++) {
			Node*tmp = headsD[r].next;//start node
			Route *Door = &doorsD[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
	}
	exportIndividualStoch(indiv);
}

void LocalSearch::Iteraterun3TradiSim(Individual & indiv) {

	determinedCost = 0;
	local_best = 999999999;
	loadIndividual(indiv);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksO.begin(), orderTrucksO.end(), params.ran);
	std::shuffle(orderDoorsO.begin(), orderDoorsO.end(), params.ran);
	for (int i = 0; i < params.nbInTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesO[i].begin(), params.correlatedVerticesO[i].end(), params.ran);
	// Shuffling the order of the nodes explored by the LS to allow for more diversity in the search
	std::shuffle(orderTrucksD.begin(), orderTrucksD.end(), params.ran);
	std::shuffle(orderDoorsD.begin(), orderDoorsD.end(), params.ran);
	for (int i = 0; i < params.nbOutTrucks; i++)
		if (params.ran() % params.ap.nbGranular == 0)  // O(n/nbGranular) calls to the inner function on average, to achieve linear-time complexity overall
			std::shuffle(params.correlatedVerticesD[i].begin(), params.correlatedVerticesD[i].end(), params.ran);
	runO3TradiSim();
	for (int r = 0; r < params.nbOutDoors; r++) {
		Node*tmp = headsD[r].next;//start node
		Route *Door = &doorsD[r];
		Door->penalty = 0;
		while (!tmp->ishead) {
			tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
			Door->penalty += tmp->TransCost;
			tmp = tmp->next;
		}
	}
	int cnt = 0;
	while (cnt <= params.ap.maxrestart && (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC < params.ap.timeLimit) {
		if (!runD3TradiSim()) {
			perturbD();
			cnt++;
		}
		for (int r = 0; r < params.nbInDoors; r++) {
			Node*tmp = headsO[r].next;//start node
			Route *Door = &doorsO[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeIncost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
		if (!runO3TradiSim()) {
			perturbO();
			cnt++;
			//break;
		}
		for (int r = 0; r < params.nbOutDoors; r++) {
			Node*tmp = headsD[r].next;//start node
			Route *Door = &doorsD[r];
			Door->penalty = 0;
			while (!tmp->ishead) {
				tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
				Door->penalty += tmp->TransCost;
				tmp = tmp->next;
			}
		}
	}
	exportIndividualStoch(indiv);
}

void LocalSearch::setLocalVariablesRouteU()
{
	routeU = nodeU->route;
	nodeX = nodeU->next;
	if (nodeX->ishead) { nodeXIndex = -1; nodeXNextIndex = -1; }
	else {
		nodeXIndex = nodeX->cour;
		if (!nodeX->next->ishead) {
			nodeXNextIndex = nodeX->next->cour;
		}
		else { nodeXNextIndex = -1; }
	}
	nodeUIndex = nodeU->cour;
	if (!nodeU->prev->ishead) { nodeUPrevIndex = nodeU->prev->cour; }
	else { nodeUPrevIndex = -1; }
}

void LocalSearch::setLocalVariablesRouteV()
{
	routeV = nodeV->route;
	nodeY = nodeV->next;
	if (nodeY->ishead) {
		nodeYIndex = -1; nodeYNextIndex = -1;
	}
	else {
		nodeYIndex = nodeY->cour;
		if (!nodeY->next->ishead) {
			nodeYNextIndex = nodeY->next->cour;
		}
		else { nodeYNextIndex = -1; }
	}
	nodeVIndex = nodeV->cour;
	if (!nodeV->prev->ishead) { nodeVPrevIndex = nodeV->prev->cour; }
	else { nodeVPrevIndex = -1; }
	intraRouteMove = (routeU == routeV);
}

int LocalSearch::computeIncost(int index, int stime, int indexdoor) {
	int delta_trans = 0;
	int OutIndex;
	int transtime;
	if (stime > params.ITK[index].LS) {
		return delta_trans = 0;
	}
	//feasible 
	for (int i = 1; i <= params.ITK[index].Suc[0]; i++) {
		OutIndex = params.ITK[index].Suc[i];
		if (trucksD[OutIndex].cumulatedStart >= params.OTK[OutIndex].AT) {
			transtime = params.Time[index][OutIndex] * params.Dis[indexdoor][trucksD[OutIndex].route->cour];
			if (transtime <= trucksD[OutIndex].cumulatedStart) {
				delta_trans += params.FP[index][OutIndex] * params.funTrans[index][OutIndex][stime][trucksD[OutIndex].cumulatedStart - transtime];
			}
		}
	}
	return -delta_trans;
}
int LocalSearch::computeOutcost(int index, int stime, int indexdoor) {
	int delta_trans = 0;
	int InIndex = 0;
	int transtime = 0;

	if (stime < params.OTK[index].AT) {
		return 0;
	}
	//feasible 
	for (int i = 1; i <= params.OTK[index].Pre[0]; i++) {
		InIndex = params.OTK[index].Pre[i];
		if (trucksO[InIndex].cumulatedStart <= params.ITK[InIndex].LS) {
			transtime = params.Time[InIndex][index] * params.Dis[trucksO[InIndex].route->cour][indexdoor];
			if (transtime <= stime) {
				delta_trans += params.FP[InIndex][index] * params.funTrans[InIndex][index][trucksO[InIndex].cumulatedStart][stime - transtime];
			}
		}
	}
	return -delta_trans;
}
int LocalSearch::compute_trial_insert_O(int nodeindex, int _cnt, int &_CurTime, int indexdoor) {
	int delta_cost = 0;
	changedIndex[_cnt] = nodeindex;
	changedStime[nodeindex] = _CurTime;
	changedTrans[nodeindex] = computeIncost(nodeindex, _CurTime, indexdoor);
	changedWTcost[nodeindex] = params.funOTO[nodeindex][_CurTime];
	delta_cost += changedTrans[nodeindex] - trucksO[nodeindex].TransCost + changedWTcost[nodeindex] - trucksO[nodeindex].WTimeCost;
	if (_CurTime <= params.ITK[nodeindex].LS) { _CurTime += params.ITK[nodeindex].GT; }
	return delta_cost;
}
int LocalSearch::compute_trial_insert_D(int nodeindex, int _cnt, int &_CurTime, int indexdoor) {
	int delta_cost = 0;
	changedIndex[_cnt] = nodeindex;
	changedStime[nodeindex] = _CurTime - params.OTK[nodeindex].GT;
	if (changedStime[nodeindex] > 0) {
		changedTrans[nodeindex] = computeOutcost(nodeindex, changedStime[nodeindex], indexdoor);
		changedWTcost[nodeindex] = params.funOTD[nodeindex][changedStime[nodeindex]];
	}
	else { changedWTcost[nodeindex] = 0; changedTrans[nodeindex] = 0; }
	delta_cost += changedTrans[nodeindex] + changedWTcost[nodeindex] - trucksD[nodeindex].TransCost - trucksD[nodeindex].WTimeCost;
	if (changedStime[nodeindex] >= params.OTK[nodeindex].AT) { _CurTime += -params.OTK[nodeindex].GT; }
	return delta_cost;
}
//insert u after v
int LocalSearch::delta_shiftO() {
	int delta_cost = 0;
	int CurTime;
	int preCurTime = 0;
	Node *tmp;
	int cnt = 0;
	//Original HGS:intraRouteMove && nodeXIndex!=nodeVIndex && nodeYIndex!=nodeUIndex
	if (intraRouteMove  && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,...,u,... --->>>  ...,v,u,...,...
			//std::cout << routeU->nbTrucks << std::endl;
			if (nodeVIndex != -1) {
				if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);//nodeU is inserted after nodeV
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);

			tmp = nodeV->next;
			while (tmp->position != nodeU->position) {//< equal to !=
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeUPrevIndex].cumulatedStart <= params.ITK[nodeUPrevIndex].LS) { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeUPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			//update nodeU->next to end
			tmp = nodeU->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,...v,... -->> upre,...,v,u,... 
		{
			if (nodeUPrevIndex != -1) {
				if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
					CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			if (trucksO[nodeUIndex].cumulatedStart <= params.ITK[nodeUIndex].LS) {//feasible insert
				//update U->next,...,v
				tmp = nodeU->next;

				while (tmp->position <= nodeV->position && !tmp->ishead) {
					CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
					if (CurTime == trucksO[tmp->cour].cumulatedStart) {
						if (trucksO[nodeVIndex].cumulatedStart <= params.ITK[nodeVIndex].LS) { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
						else { CurTime = trucksO[nodeVIndex].cumulatedStart; }
						break;
					}
					cnt++;
					delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
					tmp = tmp->next;
				}
			}
			else {//infesaible insert
				if (trucksO[nodeVIndex].cumulatedStart <= params.ITK[nodeVIndex].LS) { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
				else { CurTime = trucksO[nodeVIndex].cumulatedStart; }
			}
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			//update V->next,...end no matter if nodexU is feasible inserted
			tmp = nodeV->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else if (!intraRouteMove) {//...,u,...   ...,v,... -->> ...,v,u,... ...,...
		if (nodeUPrevIndex != -1) {
			if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
				CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		// only the feasible nodexU is necessary to check the remaining node
		if (trucksO[nodeUIndex].cumulatedStart <= params.ITK[nodeUIndex].LS) {
			tmp = nodeU->next;
			//RouteU: remove u, ...,u,... -->> ...,...
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}
		}
		changedNbTrucks[routeU->cour] = -1;
		//RouteV: insert nodeU, ...v,... -->> ...v,u,...
		if (nodeVIndex != -1) {
			if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);//nodeU is inserted after nodeV
		preCurTime = CurTime;
		cnt++;
		delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
		if (CurTime > preCurTime) {//feasible insert, or no need to evaluate
			tmp = nodeV->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;

			}
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 1;
	}
	return delta_cost;
}
//u x is  inserted after v,move2
int LocalSearch::delta_shiftO2() {
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,...,u,x,... --->>>  ...,v,u,x,...,...
			if (nodeVIndex != -1) {
				if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			preCrutime = CurTime;
			//nodeU and nodeX is inserted after nodeV
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			// nodeX is inserted after nodeU
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			// update remaining node V->next,...uprev

			tmp = nodeV->next;
			while (tmp->position < nodeU->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeUPrevIndex].cumulatedStart <= params.ITK[nodeUPrevIndex].LS) { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeUPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}

			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
		else //...,upre,u,x,...v,... -->> upre,...,v,u,x,... 
		{
			if (nodeUPrevIndex != -1) {
				if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
					CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			tmp = nodeX->next;
			// x->next,...,v
			while (tmp->position <= nodeV->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeVIndex].cumulatedStart <= params.ITK[nodeVIndex].LS) { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeVIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			//insert U,X
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			preCrutime = CurTime;
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);

			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);// nodeX is inserted after nodeU
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);


			tmp = nodeV->next;
			while (!tmp->ishead) {//u,x,vnext,...end
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;

			}
			changedNbTrucks[routeV->cour] = 0;
			changedIndex[0] = cnt;
		}
	}
	else if (!intraRouteMove &&nodeXIndex != -1) {//!intraRoute   ....v,...,  ...,u,x,....  -->> ...,v,u,x,... ...,...
		if (nodeUPrevIndex != -1) {
			if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
				CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//RouteU:remove (u,x) ...,(u,x),... -->> ...,...
		if (trucksO[nodeUIndex].cumulatedStart <= params.ITK[nodeUIndex].LS || trucksO[nodeXIndex].cumulatedStart <= params.ITK[nodeXIndex].LS) {//only if and only at least one of the node(nodeU,nodeX) is feasible
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;

			}
		}
		changedNbTrucks[routeU->cour] = -2;
		//RouteV: insert (u,x) ...v,... -->> ...v,(u,x),...
		if (nodeVIndex != -1) {
			if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		preCrutime = CurTime;
		//nodeU is inserted after nodeV
		CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);

		CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);// nodeX is inserted after nodeU
		cnt++;
		delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
		//v->next, ...

		tmp = nodeV->next;
		while (!tmp->ishead) {//u,x,vnext,...
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;

		}
		changedNbTrucks[routeV->cour] = 2;
		changedIndex[0] = cnt;
	}
	return delta_cost;
}
// remove u x, then  insert x u after v,move3
int LocalSearch::delta_2shiftO()
{
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,...,u,x,... --->>>  ...,v,x,u,...(x,u)...
			if (nodeVIndex != -1) {
				if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			preCrutime = CurTime;
			//nodeU and nodeX is inserted after nodeV
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			// nodeU is inserted after nodeX
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			//update node->next to uprev
			tmp = nodeV->next;
			while (tmp->position < nodeU->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeUPrevIndex].cumulatedStart <= params.ITK[nodeUPrevIndex].LS) { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeUPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}//
			//update x->next to end
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}//
			changedNbTrucks[routeV->cour] = 0;
			changedIndex[0] = cnt;
		}
		else //...,upre,u,x,...v,... -->> upre,x->next,...,v,x,u,... 
		{
			if (nodeUPrevIndex != -1) {
				if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
					CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			tmp = nodeX->next;
			//X->next,...,v
			while (tmp->position <= nodeV->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeVIndex].cumulatedStart <= params.ITK[nodeVIndex].LS) { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeVIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			//v,x,u,v->next,...
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			// nodeU is inserted after nodeX
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			//vnext,...,end
			tmp = nodeV->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedNbTrucks[routeV->cour] = 0;
			changedIndex[0] = cnt;
		}
	}
	else if (!intraRouteMove && nodeXIndex != -1) {//!intraRoute   ....v,...,  ...,u,x,....  -->> ...,v,x,u,... ...,...
		if (nodeUPrevIndex != -1) {
			if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
				CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//routeU:remove(u,x) ...,(u,x),... -->> ...,...
		if (trucksO[nodeUIndex].cumulatedStart <= params.ITK[nodeUIndex].LS || trucksO[nodeXIndex].cumulatedStart <= params.ITK[nodeXIndex].LS) {//only if and only one of the node(nodeU,nodeX) is feasible
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}
		}
		changedNbTrucks[routeU->cour] = -2;
		//routeV:insert(x,u)  ...v,... -->> ...v,(x,u),...
		if (nodeVIndex != -1) {
			if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) { CurTime = trucksO[nodeVIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//nodeX is inserted after nodeV
		CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
		// nodeU is inserted after nodeX
		CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);

		tmp = nodeV->next;
		while (!tmp->ishead) {//vnext,...end
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 2;
	}
	return delta_cost;
}
//swap (u,x) and v ,move5
int LocalSearch::delta_swapO2T1() {
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,...,u,x,... --->>>  ...,u,x,...,v,...
			if (nodeVPrevIndex != -1) {
				if (trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF) { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			//nodeU is inserted after nodeVprev
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			// nodeX is inserted after nodeU
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			//feasible insert nodeX or nodeU, update remaining node V->next,...

			tmp = nodeV->next;
			while (tmp->position < nodeU->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeUPrevIndex].cumulatedStart <= params.ITK[nodeUPrevIndex].LS) { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeUPrevIndex].cumulatedStart; }
					break;
				} //"break" is not allowed 
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}//
			CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);// nodeX is inserted after nodeU
			cnt++;
			delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeV->cour);
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
		else //...,upre,u,x,...v,... -->> upre,v,...,u,x,... 
		{
			if (nodeUPrevIndex != -1) {
				if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
					CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);// nodeV is inserted after nodeU-prev
			cnt++;
			delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeV->cour);
			tmp = nodeX->next;
			// x->next,...,(v)
			while (tmp->position < nodeV->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeVPrevIndex].cumulatedStart <= params.ITK[nodeVPrevIndex].LS) { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;

			}
			//insert node U afte vprev
			//nodeU is inserted after nodeVprev
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			// nodeX is inserted after nodeU
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			tmp = nodeV->next;
			while (!tmp->ishead) {//,vnext,...end
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;

			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
	}
	else if (!intraRouteMove && nodeXIndex != -1) {//!intraRoute   ....v,...,  ...,u,x,....  -->> ...,u,x,... ...,v,...
		if (nodeUPrevIndex != -1) {
			if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
				CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }

		//RouteU:remove (u,x) and insert V ...,(u,x),... -->> ...,v,...
		CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);// nodeV is inserted after nodeU-prev
		cnt++;
		delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeU->cour);
		tmp = nodeX->next;
		while (!tmp->ishead) {
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->next;
		}
		changedNbTrucks[routeU->cour] = -1;

		//RouteV: remove V and insert (u,x) ...,v,... -->> ...Vprev,(u,x),...

		if (nodeVPrevIndex != -1) {
			if (trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF) { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//nodeU is inserted after nodeVprev
		CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
		// nodeX is inserted after nodeU
		CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
		//feasible insert nodeX or nodeU

		tmp = nodeV->next;
		while (!tmp->ishead) {//vnext,...end
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;
		}//
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 1;
	}
	return delta_cost;
}
//swap (u,x) and (v,y), move 6
int LocalSearch::delta_swapO2T2()
{
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != -1 && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,y,...,u,x,... --->>>  ...,u,x,...,v,y...
			if (nodeVPrevIndex != -1) {
				if (trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF) { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			//nodeU is inserted after nodeVprev
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			// nodeX is inserted after nodeU
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			tmp = nodeY->next;
			while (tmp->position < nodeU->position) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeUPrevIndex].cumulatedStart <= params.ITK[nodeUPrevIndex].LS) { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeUPrevIndex].cumulatedStart; }
					break;
				} //"break" is  allowed 
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}//
			// nodeV is inserted after nodeU-prev
			CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeU->cour);
			CurTime = std::max(CurTime, params.ITK[nodeYIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeYIndex, cnt, CurTime, routeU->cour);
			tmp = nodeX->next;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
		else //...,upre,u,x,...v,y,... -->> upre,v,y,...,u,x,... 
		{
			if (nodeUPrevIndex != -1) {
				if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
					CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeU->cour);
			CurTime = std::max(CurTime, params.ITK[nodeYIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeYIndex, cnt, CurTime, routeU->cour);
			tmp = nodeX->next;
			// x->next,...,(v,y)
			while (tmp->position < nodeV->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[nodeVPrevIndex].cumulatedStart <= params.ITK[nodeVPrevIndex].LS) { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;

			}
			//nodeU is inserted after nodeVprev
			CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
			// nodeX is inserted after nodeU
			CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
			tmp = nodeY->next;
			while (!tmp->ishead) {//u,x,vnext,...end
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
	}
	//!intraRoute   ....v,y,...,  ...,u,x,....  -->> ...,u,x,... ...,v,y...
	else if (!intraRouteMove && nodeYIndex != -1 && nodeXIndex != -1) {
		if (nodeUPrevIndex != -1) {
			if (trucksO[nodeUPrevIndex].cumulatedEnd > params.ITK[nodeUPrevIndex].LF) {
				CurTime = trucksO[nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//RouteU:remove (u,x) and insert V,Y ...,(u,x),... -->> ...,v,y,...
		// nodeV is inserted after nodeU-prev
		CurTime = std::max(CurTime, params.ITK[nodeVIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeVIndex, cnt, CurTime, routeU->cour);
		CurTime = std::max(CurTime, params.ITK[nodeYIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeYIndex, cnt, CurTime, routeU->cour);
		tmp = nodeX->next;
		while (!tmp->ishead) {
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->next;
		}
		changedNbTrucks[routeU->cour] = 0;
		//RouteV: remove V,Y and insert (u,x) ...,v,y,... -->> ...,(u,x),...
		if (nodeVPrevIndex != -1) {
			if (trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF) { CurTime = trucksO[nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[nodeVPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }

		//nodeU is inserted after nodeVprev
		CurTime = std::max(CurTime, params.ITK[nodeUIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeUIndex, cnt, CurTime, routeV->cour);
		// nodeX is inserted after nodeU
		CurTime = std::max(CurTime, params.ITK[nodeXIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(nodeXIndex, cnt, CurTime, routeV->cour);
		tmp = nodeY->next;
		while (!tmp->ishead) {//ynext,...end
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;
		}//
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 0;
	}
	return delta_cost;
}
//swap u,v, move 4,  U,X must exist
int LocalSearch::delta_swapO(Node*U, Node*V)
{
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	int UIndex = U->cour;
	int VIndex = V->cour;
	int _nodeUPrevIndex;
	int _nodeVPrevIndex;
	if (!U->prev->ishead) { _nodeUPrevIndex = U->prev->cour; }
	else { _nodeUPrevIndex = -1; }
	if (!V->prev->ishead) { _nodeVPrevIndex = V->prev->cour; }
	else { _nodeVPrevIndex = -1; }
	if (intraRouteMove) {

		if (U->position > V->position) {//...,v,...,u,... --->>>  ...,u,...,v,...
			if (_nodeVPrevIndex != -1) {
				if (trucksO[_nodeVPrevIndex].cumulatedEnd > params.ITK[_nodeVPrevIndex].LF) { CurTime = trucksO[_nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksO[_nodeVPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			//nodeU is inserted after nodeVprev
			CurTime = std::max(CurTime, params.ITK[UIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(UIndex, cnt, CurTime, routeV->cour);
			tmp = V->next;
			//update V-next,...,U-prev
			while (tmp->position < U->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[_nodeUPrevIndex].cumulatedStart <= params.ITK[_nodeUPrevIndex].LS) { CurTime = trucksO[_nodeUPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[_nodeUPrevIndex].cumulatedStart; }
					break;
				}//the "break" is  allowed
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;

			}
			// nodeV is inserted after nodeU-prev
			CurTime = std::max(CurTime, params.ITK[VIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(VIndex, cnt, CurTime, routeV->cour);
			tmp = U->next;
			//update U-next,...,end
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					break;
				}//the "break" is  allowed
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
		else //...,upre,u,...v,... -->> upre,v,...,u,... 
		{
			if (_nodeUPrevIndex != -1) {
				if (trucksO[_nodeUPrevIndex].cumulatedEnd > params.ITK[_nodeUPrevIndex].LF) {
					CurTime = trucksO[_nodeUPrevIndex].cumulatedStart;
				}
				else { CurTime = trucksO[_nodeUPrevIndex].cumulatedEnd; }
			}
			else { CurTime = 0; }
			//v
			CurTime = std::max(CurTime, params.ITK[VIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(VIndex, cnt, CurTime, routeV->cour);
			tmp = U->next;
			//update U->next,...,V->prev
			while (tmp->position < V->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) {
					if (trucksO[_nodeVPrevIndex].cumulatedStart <= params.ITK[_nodeVPrevIndex].LS) { CurTime = trucksO[_nodeVPrevIndex].cumulatedEnd; }
					else { CurTime = trucksO[_nodeVPrevIndex].cumulatedStart; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			CurTime = std::max(CurTime, params.ITK[UIndex].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(UIndex, cnt, CurTime, routeV->cour);
			tmp = V->next;
			//update V->next,...end no matter if U is feasible inserted
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedNbTrucks[routeV->cour] = 0;
			changedIndex[0] = cnt;
		}
	}
	else {//routeU: ...,u,...   routeV: ...,v,... -->>routeU: ...,v,... routeV: ...,u,...
		if (_nodeUPrevIndex != -1) {
			if (trucksO[_nodeUPrevIndex].cumulatedEnd > params.ITK[_nodeUPrevIndex].LF) {
				CurTime = trucksO[_nodeUPrevIndex].cumulatedStart;
			}
			else { CurTime = trucksO[_nodeUPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//routeU: insert V
		CurTime = std::max(CurTime, params.ITK[VIndex].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(VIndex, cnt, CurTime, routeU->cour);
		//update U->next,...end
		tmp = U->next;
		while (!tmp->ishead) {
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) {
				break;
			}//the "break" is  allowed
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->next;
		}
		changedNbTrucks[routeU->cour] = 0;
		//RouteV: insert U, ...,... -->> ...u,...
		if (_nodeVPrevIndex != -1) {
			if (trucksO[_nodeVPrevIndex].cumulatedEnd > params.ITK[_nodeVPrevIndex].LF) { CurTime = trucksO[_nodeVPrevIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksO[_nodeVPrevIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		CurTime = std::max(CurTime, params.ITK[UIndex].AT);//nodeU is inserted after nodeV
		cnt++;
		delta_cost += compute_trial_insert_O(UIndex, cnt, CurTime, routeV->cour);
		tmp = V->next;
		while (!tmp->ishead) {
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;

		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 0;
	}
	return delta_cost;
}
//routeU and routeV is not empty
int LocalSearch::delta_2_optO() {
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	if (intraRouteMove) {// ...u,x,x->next,,v->prev,v,y,... ->>...u,v,v-prev,...,x->next,x,y,...
		if (nodeU->position > nodeV->position || nodeXIndex == nodeVIndex)return 0;
		else {
			if (trucksO[nodeUIndex].cumulatedEnd > params.ITK[nodeUIndex].LF) {
				CurTime = trucksO[nodeUIndex].cumulatedStart;
			}
			else { CurTime = trucksO[nodeUIndex].cumulatedEnd; }
			//insert V,Vprev,...X
			tmp = nodeV;
			while (tmp->position >= nodeX->position && !tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
			tmp = nodeY;
			while (!tmp->ishead) {
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->next;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else {//routeU; ....U,V,V->prev,...,start   routeV: end...,X->next,X,Y,...,end

		if (trucksO[nodeUIndex].cumulatedEnd > params.ITK[nodeUIndex].LF) {
			CurTime = trucksO[nodeUIndex].cumulatedStart;
		}
		else { CurTime = trucksO[nodeUIndex].cumulatedEnd; }
		//routeU: insert V,V->prev,..start after node U

		int numU = 0;
		int numV = 0;
		tmp = nodeV;
		while (!tmp->ishead) {
			numU++;
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->prev;
		}

		//routeV: insert end,...,X->next,X,before node Y
		CurTime = 0;
		if (!nodeX->ishead) {
			tmp = routeU->head->prev;//end node

			while (tmp->position >= nodeX->position) {
				numV++;
				CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
				cnt++;
				delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}

		}
		//insert ndoeY after ndoe X
		tmp = nodeY;// could be the head(end) node
		while (!tmp->ishead) {
			CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
			if (CurTime == trucksO[tmp->cour].cumulatedStart) { break; }
			cnt++;
			delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeU->cour] = numU - numV;
		changedNbTrucks[routeV->cour] = numV - numU;

	}
	return delta_cost;
}
//replace(U, X) and (V, Y) by(U, Y) and (V, X)
int LocalSearch::delta_2_optStarO() {
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	int numU = 0, numV = 0;
	//routeU: insert nodeY,..,end after nodeU
	if (trucksO[nodeUIndex].cumulatedEnd > params.ITK[nodeUIndex].LF) {
		CurTime = trucksO[nodeUIndex].cumulatedStart;
	}
	else { CurTime = trucksO[nodeUIndex].cumulatedEnd; }
	tmp = nodeY;
	while (!tmp->ishead) {
		numU++;
		CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeU->cour);
		tmp = tmp->next;
	}
	//routeV: insert nodeX,..,end after nodeV
	if (nodeVIndex != -1) {
		if (trucksO[nodeVIndex].cumulatedEnd > params.ITK[nodeVIndex].LF) {
			CurTime = trucksO[nodeVIndex].cumulatedStart;
		}
		else { CurTime = trucksO[nodeVIndex].cumulatedEnd; }
	}
	else { CurTime = 0; }
	tmp = nodeX;
	while (!tmp->ishead) {
		numV++;
		CurTime = std::max(CurTime, params.ITK[tmp->cour].AT);
		cnt++;
		delta_cost += compute_trial_insert_O(tmp->cour, cnt, CurTime, routeV->cour);
		tmp = tmp->next;
	}
	changedIndex[0] = cnt;
	changedNbTrucks[routeU->cour] = numU - numV;
	changedNbTrucks[routeV->cour] = numV - numU;
	return delta_cost;
}
//!!!!!!!!
int LocalSearch::delta_shiftD() {
	int delta_cost = 0;
	int CurTime;//活动的endtime
	Node *tmp;
	int cnt = 0;
	//Original HGS:intraRouteMove && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex
	if (intraRouteMove  && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,y,...,u,x... --->>>  ...,v,u,y...,x...
			if (nodeXIndex != -1) {
				if (trucksD[nodeXIndex].cumulatedStart >= params.OTK[nodeXIndex].AT) { CurTime = trucksD[nodeXIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[nodeXIndex].cumulatedEnd; }//nodeX is infeasible, trucksD[nodeXIndex].cumulatedEnd=rucksD[nodeXNextIndex].cumulatedStart
			}
			else { CurTime = 9999; }
			//update nodeU->prev to nodeY;
			tmp = nodeU->prev;
			if (trucksD[nodeUIndex].cumulatedStart >= params.OTK[nodeUIndex].AT) {//feasible nodeU
				while (tmp->position >= nodeY->position && !tmp->ishead) {
					CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
					if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
						if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
							CurTime = trucksD[nodeYIndex].cumulatedStart;
						}
						else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
						break;
					}//the "break" is allowed
					cnt++;
					delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
					tmp = tmp->prev;
				}
			}
			else {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			//insert node U
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);//nodeU  is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);
			//update nodeV to start
			tmp = nodeV;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
		else //...,upre,u,x...v,y,... -->> upre,x...,v,u,y,... 
		{
			if (nodeYIndex != -1) {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }
			//insert nodeU before y
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);//nodeU  is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);

			//update V,...,x(include)
			tmp = nodeV;
			while (tmp->position >= nodeX->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeXIndex].cumulatedStart >= params.OTK[nodeXIndex].AT) {
						CurTime = trucksD[nodeXIndex].cumulatedStart;
					}
					else { CurTime = trucksD[nodeXIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;

			}
			tmp = nodeU->prev;
			//update nodeU->prev,...start, no matter if nodexU is feasible inserted
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
	}
	else if (!intraRouteMove) {//routeU: ...,u,x,...   routeV:...,v,y,... -->> routeV:...,v,u,y,...  routeU: ...,x,...
		if (nodeXIndex != -1) {
			if (trucksD[nodeXIndex].cumulatedStart >= params.OTK[nodeXIndex].AT) {
				CurTime = trucksD[nodeXIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeXIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		// only the feasible nodexU is necessary to check the remaining node
		if (trucksD[nodeUIndex].cumulatedStart >= params.OTK[nodeUIndex].AT) {
			tmp = nodeU->prev;
			//RouteU: remove u and update node before X: ...,u,x... -->> ...,x...
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}

		}
		changedNbTrucks[routeU->cour] = -1;
		//RouteV: insert nodeU, ...v,y,... -->> ...v,u,y,...
		if (nodeYIndex != -1) {//is not the head node
			if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) { CurTime = trucksD[nodeYIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
		}
		else { CurTime = 99999; }
		CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);//nodeU is inserted after nodeV (before nodeY)
		cnt++;
		delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);
		//update nodeV to start
		tmp = nodeV;
		while (!tmp->ishead) {//==tmp->position!=-1
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;

		}
		changedNbTrucks[routeV->cour] = 1;
		changedIndex[0] = cnt;
	}
	return delta_cost;
}
//u x is  inserted after v,move2
int LocalSearch::delta_shiftD2() {
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,y,...,u->prev,u,x,x->next... --->>>  ...,v,u,x,y,...u->prev,x->next,...
			if (nodeXNextIndex != -1) {
				if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }
			//preCrutime = CurTime;
			//update U->prev to Y
			tmp = nodeU->prev;
			while (tmp->position >= nodeY->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
						CurTime = trucksD[nodeYIndex].cumulatedStart;
					}
					else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//
			//insert u,x before Y
			//preCrutime = CurTime;
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);//nodeU and nodeX is inserted after nodeV
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			// insert U before nodeX
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);
			tmp = nodeV;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;

			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,x,x->next,...v,y,... -->> ...,upre,(u,x),x->next,...,v,u,x,y... 
		{
			if (nodeYIndex != -1) {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			else { CurTime = 99999; }
			// insert X before y,

			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted after nodeU
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			//else { CurTime = preCrutime; }

			//insert nodeU before X
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);

			//update V to x->next
			tmp = nodeV;

			while (tmp->position > nodeX->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) {//feasible
						CurTime = trucksD[nodeXNextIndex].cumulatedStart;
					}
					else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}

			//update uprev to start
			tmp = nodeU->prev;
			while (!tmp->ishead) {//upre,...start
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeV->cour] = 0;
		}
	}
	else if (!intraRouteMove &&nodeXIndex != -1) {//!intraRoute   ....v,y,...,  ...u->prev,u,x,x->next....  -->> ...,v,u,x,y,... ...u->prev,x->next...

		if (nodeXNextIndex != -1) {
			if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) {
				CurTime = trucksD[nodeXNextIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
		}
		else { CurTime = 999999; }
		//RouteU:remove (u,x) ...,(u,x),... -->> ...,...
		if (trucksD[nodeXIndex].cumulatedStart >= params.OTK[nodeXIndex].AT || trucksD[nodeUIndex].cumulatedStart >= params.ITK[nodeUIndex].AT) {//only if and only at least one of the node(nodeU,nodeX) is feasible
			tmp = nodeU->prev;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
		}
		changedNbTrucks[routeU->cour] = -2;
		//RouteV: insert (u,v) ...v,y,... -->> ...v,(u,x),y,...
		if (nodeYIndex != -1) {
			if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) { CurTime = trucksD[nodeYIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		//insert X
		CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
		cnt++;
		delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
		//insert U
		CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);

		//update V to start
		tmp = nodeV;
		while (!tmp->ishead) {//...,v
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 2;
	}
	return delta_cost;
}
// remove u x, then  insert x u after v,move3
int LocalSearch::delta_2shiftD()
{
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,y,...,u,x,... --->>>  ...,v,x,u,,y,...u->prev,(u,x),x->next,...
			if (nodeXNextIndex != -1) {
				if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
			}
			else { CurTime = 99999; }
			preCrutime = CurTime;
			//update U->prev to Y
			tmp = nodeU->prev;
			while (tmp->position >= nodeY->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
						CurTime = trucksD[nodeYIndex].cumulatedStart;
					}
					else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//

			//insert u,x before Y
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);//nodeU and nodeX is inserted after nodeV
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeU->cour);

			// insert X before nodeU
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeU->cour);

			//update nodeV to start
			tmp = nodeV;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);

				tmp = tmp->prev;

			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,x,x->next,...v,y,... -->> ...,upre,(u,x),x->next,...,v,x,u,y... 
		{
			if (nodeYIndex != -1) {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {//feasible
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			else { CurTime = 999999; }

			// insert U before y,
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeX is inserted after nodeU
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeU->cour);


			//insert nodeX before U
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeU->cour);

			//update V to x->next
			tmp = nodeV;
			while (tmp->position > nodeX->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) {//feasible
						CurTime = trucksD[nodeXNextIndex].cumulatedStart;
					}
					else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;

			}

			//update uprev to start
			tmp = nodeU->prev;
			while (!tmp->ishead) {//upre,...end
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else if (!intraRouteMove &&nodeXIndex != -1) {//!intraRoute   ....v,y,...,  ...u->prev,u,x,x->next....  -->> ...,v,x,u,y,... ...u->prev,x->next...
		if (nodeXNextIndex != -1) {
			if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) {
				CurTime = trucksD[nodeXNextIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
		}
		else { CurTime = 0; }
		//RouteU:remove (u,x) ...u->prev,(u,x),x->next... -->> ...,u->prev,x->next,...
		if (trucksD[nodeUIndex].cumulatedStart >= params.OTK[nodeUIndex].AT || trucksD[nodeXIndex].cumulatedStart >= params.ITK[nodeXIndex].AT) {//only if and only at least one of the node(nodeU,nodeX) is feasible
			tmp = nodeU->prev;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
		}
		changedNbTrucks[routeU->cour] = -2;
		//RouteV: insert (x,u) ...v,y,... -->> ...v,(x,u),y,...
		if (nodeYIndex != -1) {
			if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) { CurTime = trucksD[nodeYIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }

		//insert U
		CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeX is inserted before nodeY
		cnt++;
		delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);

		//insert X
		CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);

		//update ndoeV to start
		tmp = nodeV;
		while (!tmp->ishead) {//...,vprev,...
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 2;
	}
	return delta_cost;
}
//swap (u,x) and v ,move5
int LocalSearch::delta_swapD2T1() {
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...,v,y...,u,x,x->next... --->>>  ...,v->prev,u,x,y,...,u->prev,v,x->next,...
			if (nodeXNextIndex != -1) {
				if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }
			//nodeV  is inserted before x->next
			CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);

			//update  u-prev to Y
			tmp = nodeU->prev;
			while (tmp->position >= nodeY->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) { CurTime = trucksD[nodeYIndex].cumulatedStart; }
					else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
			//insert node X before Y

			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			//insert nodeU before X
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);
			//update node V->prev to start
			tmp = nodeV->prev;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,x,x->next,...,v-Prev,v,y... -->> upre,v,x->next...v->prev,u,x,y,... 
		{
			if (nodeYIndex != -1) {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			else { CurTime = 99999; }
			// nodeX is inserted before nodeY

			//insert node X before Y
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			//insert nodeU before X
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);


			// update x->next,...,(vpre)
			tmp = nodeV->prev;
			while (tmp->position >= nodeX->next->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }
					else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
			//insert V before X->next
			CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);
			//update uprev to start
			tmp = nodeU->prev;
			while (!tmp->ishead) {//uprev,...start
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else if (!intraRouteMove && nodeXIndex != -1) {//!intraRoute   ....,v->prev,v,y,...,  ...u->prev,u,x,x->next....  -->> ...,v->prev,u,x,y,... ...u-prev,v,x->next,...
		if (nodeXNextIndex != -1) {
			if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		//RouteU:remove (u,x) and insert V ...,(u,x),... -->> ...u-prev,v,x->next,...
		//insert V before X->next
		CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);
		//update uprev to start
		tmp = nodeU->prev;
		while (!tmp->ishead) {//...uprev
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->prev;
		}//
		changedNbTrucks[routeU->cour] = -1;
		//RouteV: remove V and insert (u,x) ...,(v),... -->> ...v-prev,u,x,y...
		if (nodeYIndex != -1) {
			if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {
				CurTime = trucksD[nodeYIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
		}
		else { CurTime = 99999; }
		//insert node X before Y

		CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
		cnt++;
		delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
		//insert nodeU before X
		CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);


		// update start,...,(vpre)
		tmp = nodeV->prev;
		while (!tmp->ishead) {
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 1;
	}
	return delta_cost;
}
int LocalSearch::delta_swapD2T2()//swap (u,x) and (v,y), move 6
{
	int delta_cost = 0;
	int preCrutime = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	bool inserted = false;
	if (intraRouteMove && nodeXIndex != -1 && nodeXIndex != nodeVIndex && nodeYIndex != -1 && nodeYIndex != nodeUIndex) {
		if (nodeU->position > nodeV->position) {//...v->prev,v,y,y->next...u->prev,u,x,x->next... --->>>  ...,v->prev,u,x,y->next...u->prev,v,y,x->next...
			if (nodeXNextIndex != -1) {
				if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }

			//nodeY  is inserted before x->next
			CurTime = std::min(CurTime, params.OTK[nodeYIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeYIndex, cnt, CurTime, routeU->cour);

			//nodeV  is inserted before nodeY
			CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);

			//update  u-prev to Y->next
			tmp = nodeU->prev;
			while (tmp->position > nodeY->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeYNextIndex].cumulatedStart >= params.OTK[nodeYNextIndex].AT) { CurTime = trucksD[nodeYNextIndex].cumulatedStart; }
					else { CurTime = trucksD[nodeYNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}

			//insert node X before Y->next
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			//insert nodeU before X
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);
			//update node V->prev to start
			tmp = nodeV->prev;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}//

			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,x,x->next...v->prev,v,y,y->next... -->> upre,v,y,x->next...v-prev,u,x,y->next... 
		{
			if (nodeYNextIndex != -1) {
				if (trucksD[nodeYNextIndex].cumulatedStart >= params.OTK[nodeYNextIndex].AT) {
					CurTime = trucksD[nodeYNextIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYNextIndex].cumulatedEnd; }
			}
			else { CurTime = 99999; }
			// nodeX is inserted before nodeY

			//insert node X before Y->next
			CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
			//insert nodeU before X
			CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);


			// update x->next,...,vpre
			tmp = nodeV->prev;
			while (tmp->position >= nodeX->next->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }
					else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}
			//insert Y before X->next
			CurTime = std::min(CurTime, params.OTK[nodeYIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeYIndex, cnt, CurTime, routeU->cour);
			//insert V before Y
			CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);// nodeU is inserted before nodeX
			cnt++;
			delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);
			//update uprev to start
			tmp = nodeU->prev;
			while (!tmp->ishead) {//uprev,...,start
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}//
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	//!intraRoute   ....vprev,v,y,y->next,...,  ...uprev,u,x,x->next....  -->> ...vprev,u,x,y->next... ...,uprev,v,y,x->next...
	else if (!intraRouteMove && nodeYIndex != -1 && nodeXIndex != -1) {
		if (nodeXNextIndex != -1) {
			if (trucksD[nodeXNextIndex].cumulatedStart >= params.OTK[nodeXNextIndex].AT) { CurTime = trucksD[nodeXNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[nodeXNextIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		//RouteU:remove (u,x) and insert V,Y ...,(u,x),... -->> ...u-prev,v,y,x->next,...

		//insert Y before X->next
		CurTime = std::min(CurTime, params.OTK[nodeYIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeYIndex, cnt, CurTime, routeU->cour);
		//insert V before Y
		CurTime = std::min(CurTime, params.OTK[nodeVIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeVIndex, cnt, CurTime, routeU->cour);
		//update uprev to start
		tmp = nodeU->prev;
		while (!tmp->ishead) {//uprev,...,start
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->prev;
		}//
		changedNbTrucks[routeU->cour] = 0;
		//RouteV: remove V and insert (u,x) ...,(v),... -->> ...v-prev,u,x,y->next...
		if (nodeYNextIndex != -1) {
			if (trucksD[nodeYNextIndex].cumulatedStart >= params.OTK[nodeYNextIndex].AT) {
				CurTime = trucksD[nodeYNextIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeYNextIndex].cumulatedEnd; }
		}
		else { CurTime = 99999; }
		//insert node X before Y->next

		CurTime = std::min(CurTime, params.OTK[nodeXIndex].LF);// nodeX is inserted before nodeY
		cnt++;
		delta_cost += compute_trial_insert_D(nodeXIndex, cnt, CurTime, routeV->cour);
		//insert nodeU before X
		CurTime = std::min(CurTime, params.OTK[nodeUIndex].LF);// nodeU is inserted before nodeX
		cnt++;
		delta_cost += compute_trial_insert_D(nodeUIndex, cnt, CurTime, routeV->cour);

		// update start,...,(vpre)
		tmp = nodeV->prev;
		while (!tmp->ishead) {
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 0;
	}
	return delta_cost;
}
//swap u,v, move 4,  U,X must exist
int LocalSearch::delta_swapD(Node*U, Node*V)
{
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	int UIndex = U->cour;
	int VIndex = V->cour;
	int _nodeUNextIndex;
	int _nodeVNextIndex;
	if (!U->next->ishead) { _nodeUNextIndex = U->next->cour; }
	else { _nodeUNextIndex = -1; }
	if (!V->next->ishead) { _nodeVNextIndex = V->next->cour; }
	else { _nodeVNextIndex = -1; }
	if (intraRouteMove) {
		if (U->position > V->position) {//...v->prev,v,v->next...,uprev,u,u->next... --->>>  ...v->prev,u,v->next...uprev,v,u->next,...
			if (_nodeUNextIndex != -1) {
				if (trucksD[_nodeUNextIndex].cumulatedStart >= params.OTK[_nodeUNextIndex].AT) { CurTime = trucksD[_nodeUNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[_nodeUNextIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }
			//V is inserted before u->next
			CurTime = std::min(CurTime, params.OTK[VIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(VIndex, cnt, CurTime, routeV->cour);
			tmp = U->prev;
			//update V-next,...,U-prev
			while (tmp->position >= V->next->position) {
				//std::cout << tmp->position << std::endl;
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[_nodeVNextIndex].cumulatedStart >= params.OTK[_nodeVNextIndex].AT) { CurTime = trucksD[_nodeVNextIndex].cumulatedStart; }
					else { CurTime = trucksD[_nodeVNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}

			//U is inserted before V->next
			CurTime = std::min(CurTime, params.OTK[UIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(UIndex, cnt, CurTime, routeV->cour);
			tmp = V->prev;
			//update start,...,V->prev
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
		else //...,upre,u,u->next...v,v->next... -->> upre,v,u->next...vprev,u,v->next... 
		{
			if (_nodeVNextIndex != -1) {
				if (trucksD[_nodeVNextIndex].cumulatedStart >= params.OTK[_nodeVNextIndex].AT) { CurTime = trucksD[_nodeVNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
				else { CurTime = trucksD[_nodeVNextIndex].cumulatedEnd; }
			}
			else { CurTime = 9999; }
			//U is inserted before V->next
			CurTime = std::min(CurTime, params.OTK[UIndex].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(UIndex, cnt, CurTime, routeV->cour);
			tmp = V->prev;
			//update U-next,...,V-prev
			while (tmp->position >= U->next->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					if (trucksD[_nodeUNextIndex].cumulatedStart >= params.OTK[_nodeUNextIndex].AT) { CurTime = trucksD[_nodeUNextIndex].cumulatedStart; }
					else { CurTime = trucksD[_nodeUNextIndex].cumulatedEnd; }
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->prev;
			}

			//V is inserted before U->next
			CurTime = std::min(CurTime, params.OTK[VIndex].LF);// nodeX is inserted before nodeY
			cnt++;
			delta_cost += compute_trial_insert_D(VIndex, cnt, CurTime, routeV->cour);
			tmp = U->prev;
			//update start,...,U->prev
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
					break;
				}
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else {//routeU: ...,u,...   routeV: ...,v,... -->>routeU: ...,v,... routeV: ...,u,...
		if (_nodeUNextIndex != -1) {
			if (trucksD[_nodeUNextIndex].cumulatedStart >= params.OTK[_nodeUNextIndex].AT) { CurTime = trucksD[_nodeUNextIndex].cumulatedStart; }
			else { CurTime = trucksD[_nodeUNextIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		//routeU: insert V before u->next
		CurTime = std::min(CurTime, params.OTK[VIndex].LF);
		cnt++;
		delta_cost += compute_trial_insert_D(VIndex, cnt, CurTime, routeU->cour);
		//update start,...Uprev
		tmp = U->prev;
		while (!tmp->ishead) {
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->prev;
		}
		changedNbTrucks[routeU->cour] = 0;

		//RouteV: insert U, ...,... -->> ...u,v->next...
		if (_nodeVNextIndex != -1) {
			if (trucksD[_nodeVNextIndex].cumulatedStart >= params.OTK[_nodeVNextIndex].AT) { CurTime = trucksD[_nodeVNextIndex].cumulatedStart; }// we assume theat params.ITK[nodeVPrevIndex].AT+params.ITK[nodeVPrevIndex].GT<=params.ITK[nodeVPrevIndex].LF, so if trucksO[nodeVPrevIndex].cumulatedEnd > params.ITK[nodeVPrevIndex].LF, trucksO[nodeVPrevIndex].cumulatedStart>params.ITK[nodeVPrevIndex].AT
			else { CurTime = trucksD[_nodeVNextIndex].cumulatedEnd; }
		}
		else { CurTime = 9999; }
		//U is inserted before V->next
		CurTime = std::min(CurTime, params.OTK[UIndex].LF);
		cnt++;
		delta_cost += compute_trial_insert_D(UIndex, cnt, CurTime, routeV->cour);
		//update start,...Vprev
		tmp = V->prev;
		while (!tmp->ishead) {
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) {
				break;
			}
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeV->cour] = 0;
	}
	return delta_cost;
}
int LocalSearch::delta_2_optD() {
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	if (intraRouteMove) {// ...u,x,x->next,,v->prev,v,y,... ->>...u,v,v-prev,...,x->next,x,y,...
		if (nodeU->position > nodeV->position || nodeXIndex == nodeVIndex)return 0;
		else {
			if (!nodeY->ishead) {
				if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {
					CurTime = trucksD[nodeYIndex].cumulatedStart;
				}
				else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
			}
			else { CurTime = 99999; }
			//insert V,Vprev,...X before node Y  <<<---
			tmp = nodeX;
			while (tmp->position <= nodeV->position && !tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}
			//update start,...,nodeU
			tmp = nodeU;
			while (!tmp->ishead) {
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
				tmp = tmp->prev;
			}
			changedIndex[0] = cnt;
			changedNbTrucks[routeU->cour] = 0;
		}
	}
	else {//routeU; startU....U,V,V->prev,...,startV   routeV: endU...,X->next,X,Y,...,endV

		//routeU: insert V,V->prev,..start after node U
		CurTime = 9999;
		int numU = 0;
		int numV = 0;
		if (!nodeV->ishead) {
			tmp = routeV->head->next;//startV truck node of routeV
			while (tmp->position <= nodeV->position && !tmp->ishead) {
				numU++;
				CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
				cnt++;
				delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
				tmp = tmp->next;
			}
		}
		else { CurTime = 999999; }
		tmp = nodeU;
		//std::cout << "in routeU U" << std::endl;
		while (!tmp->ishead) {

			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
			tmp = tmp->prev;
		}
		//routeV: insert end,...,X->next,X,before node Y
		if (nodeYIndex != -1) {
			if (trucksD[nodeYIndex].cumulatedStart >= params.OTK[nodeYIndex].AT) {
				CurTime = trucksD[nodeYIndex].cumulatedStart;
			}
			else { CurTime = trucksD[nodeYIndex].cumulatedEnd; }
		}
		else { CurTime = 999999; }
		//update end ,...,X
		tmp = nodeX;
		while (!tmp->ishead) {
			numV++;
			//cout << CurTime << endl;
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->next;
		}
		changedIndex[0] = cnt;
		changedNbTrucks[routeU->cour] = numU - numV;
		changedNbTrucks[routeV->cour] = numV - numU;

	}
	return delta_cost;
}
//replace(U, X) and (V, Y) by(U, Y) and (V, X)
int LocalSearch::delta_2_optStarD() {
	int delta_cost = 0;
	int CurTime;
	Node *tmp;
	int cnt = 0;
	int numU = 0, numV = 0;

	//routeU: insert nodeY,..,endV after nodeU
	CurTime = 9999;
	if (!nodeY->ishead) {
		tmp = routeV->head->prev;//end node
		while (tmp->position >= nodeY->position) {
			numU++;
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);

			tmp = tmp->prev;
		}
	}
	else { CurTime = 999999; }
	tmp = nodeU;
	while (!tmp->ishead) {
		CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
		if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
		cnt++;
		delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeU->cour);
		tmp = tmp->prev;
	}

	//routeV: insert nodeX,..,endU after nodeV
	CurTime = 9999;
	if (!nodeX->ishead) {
		tmp = routeU->head->prev;//end ndoe
		while (tmp->position >= nodeX->position) {
			numV++;
			CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
			cnt++;
			delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
			tmp = tmp->prev;
		}
	}
	tmp = nodeV;
	while (!tmp->ishead) {
		CurTime = std::min(CurTime, params.OTK[tmp->cour].LF);
		if (CurTime == trucksD[tmp->cour].cumulatedEnd) { break; }
		cnt++;
		delta_cost += compute_trial_insert_D(tmp->cour, cnt, CurTime, routeV->cour);
		tmp = tmp->prev;
	}

	changedIndex[0] = cnt;
	changedNbTrucks[routeU->cour] = numU - numV;
	changedNbTrucks[routeV->cour] = -changedNbTrucks[routeU->cour];
	return delta_cost;
}


//link node and update position
void LocalSearch::insertNode(Node * U, Node * V, bool intraRouteMove)
{
	int tmpp;
	Node *tmp;
	Node *Unext;
	//remove nodeU
	Unext = U->next;//record the u->next before insert nodeU //1 4 5 head->1 5 4 head  u=4,v=5//head 1 4 5 head->head 4 1 5 head u=4,v=head
	U->prev->next = U->next;//1 -> 5//1->5
	U->next->prev = U->prev;//1 -< 5//1<-5
	//insert U after V
	V->next->prev = U;//4<-head    //4<-1
	U->prev = V;     //5<-4        //head<-4
	U->next = V->next;//4->head    //4->1
	V->next = U;      //5->4       //head->4
	U->route = V->route;//
	//std::cout << intraRouteMove << std::endl;
	if (intraRouteMove) {
		//....v,...,u,... -->> ...,v,u,...,...
		if (U->position > V->position) {
			tmpp = U->position;
			U->position = V->position + 1;
			tmp = U->next;//V->next
			while (tmp->position < tmpp && !tmp->ishead) {
				tmp->position += 1;
				tmp = tmp->next;
			}
		}
		//....u,...,v,... -->> ...,Unext,,...v,u...
		else {
			tmpp = V->position;
			U->position = tmpp;
			tmp = Unext;
			//Unext,...,V's postion substract 1
			while (tmp->position < V->position && !tmp->ishead) {
				tmp->position += -1;
				tmp = tmp->next;
			}
			V->position += -1;
		}
	}
	else {//....u,...   ...,v,... -->> ...,Unext,...   ....v,u...
		//route U
		tmp = Unext;
		while (!tmp->ishead) {
			tmp->position += -1;
			tmp = tmp->next;
		}
		//routeV
		U->position = V->position + 1;
		tmp = U->next;//V->next
		while (!tmp->ishead) {
			tmp->position += 1;
			tmp = tmp->next;
		}
	}
}
// ,,,u,...v,...        ...v,...,u,... note: the nodeU and nodeV cannot be adjacent
void LocalSearch::swapNode(Node * U, Node * V, bool intraRouteMove)
{
	Node * Vpre = V->prev;//7 //head
	Node * Vnext = V->next;//head//4
	Node * Upre = U->prev;//5 //4
	Node * Unext = U->next;//0 //head
	Route * myRouteU = U->route;
	Route * myRouteV = V->route;
	int tmp;
	if (U->cour == V->next->cour) {//...V,U,...-> ...,U,V,...
		Vpre->next = U;
		U->prev = Vpre;
		Unext->prev = V;
		V->next = Unext;
		U->next = V;
		V->prev = U;
		tmp = U->position;
		U->position = V->position;
		V->position = tmp;
	}
	else if (V->cour == U->next->cour) {//...,U,V,... ->  ...V,U,...
		Upre->next = V;
		V->prev = Upre;
		Vnext->prev = U;
		U->next = Vnext;
		V->next = U;
		U->prev = V;
		tmp = U->position;
		U->position = V->position;
		V->position = tmp;
	}
	else {

		Upre->next = V;//5->0   //4->1
		Unext->prev = V;//0<-0  //1<-head
		Vpre->next = U;//7->7   //head->5
		Vnext->prev = U;//7<-head//5<-4
		U->prev = Vpre;//7<-7    //head<-5
		U->next = Vnext;//7->head//5->4
		V->prev = Upre;//5<-0     //4<-1
		V->next = Unext;//0->0    //1->head
		U->route = myRouteV;
		V->route = myRouteU;
		tmp = U->position;
		U->position = V->position;
		V->position = tmp;
	}
}
//update the start time, end time, isfeasible, cost
void LocalSearch::updateTrucksO() {
	for (int i = 1; i <= changedIndex[0]; i++) {
		trucksO[changedIndex[i]].cumulatedStart = changedStime[changedIndex[i]];
		if (trucksO[changedIndex[i]].cumulatedStart <= params.ITK[changedIndex[i]].LS) {
			trucksO[changedIndex[i]].isfeasible = true;
		}
		else { trucksO[changedIndex[i]].isfeasible = false; }
		trucksO[changedIndex[i]].cumulatedEnd = changedStime[changedIndex[i]] + params.ITK[changedIndex[i]].GT;
		trucksO[changedIndex[i]].WTimeCost = changedWTcost[changedIndex[i]];
		trucksO[changedIndex[i]].TransCost = changedTrans[changedIndex[i]];
	}
}
//update the start time, end time, isfeasible, cost
void LocalSearch::updateTrucksD() {
	for (int i = 1; i <= changedIndex[0]; i++) {
		trucksD[changedIndex[i]].cumulatedStart = changedStime[changedIndex[i]];
		if (trucksD[changedIndex[i]].cumulatedStart >= params.OTK[changedIndex[i]].AT) {
			trucksD[changedIndex[i]].isfeasible = true;
		}
		else { trucksD[changedIndex[i]].isfeasible = false; }
		trucksD[changedIndex[i]].cumulatedEnd = changedStime[changedIndex[i]] + params.OTK[changedIndex[i]].GT;
		trucksD[changedIndex[i]].WTimeCost = changedWTcost[changedIndex[i]];
		trucksD[changedIndex[i]].TransCost = changedTrans[changedIndex[i]];
	}
}
//update the door's time warp, cost and the number when the door was modified last.
void LocalSearch::updateRouteDataD(Route * myRoute, int changeNB)
{
	Node * mynode = myRoute->head->next;
	myRoute->whenLastModified = nbMoves;
	myRoute->nbTrucks += changeNB;
	myRoute->WTime = 0;
	myRoute->penalty = 0;
	TotalTimeWarp += -myRoute->TimeWarp;
	myRoute->TimeWarp = 0;
	while (!mynode->ishead) {
		myRoute->WTime += mynode->WTimeCost;
		myRoute->penalty += mynode->TransCost;
		if (!mynode->isfeasible) {
			myRoute->TimeWarp += params.OTK[mynode->cour].GT;
		}
		mynode = mynode->next;
	}
	TotalTimeWarp += myRoute->TimeWarp;
	mynode = myRoute->head->next;
	int myposition = 0;
	while (!mynode->ishead) {
		if (mynode->position != myposition)
		{
			cout << "D forward scheduling: " << mynode->cour << "(" << mynode->position << ") and myposition " << myposition << endl;
			system("pause");
		}
		if (mynode->cumulatedEnd < mynode->prev->cumulatedEnd) { cout << mynode->cumulatedEnd << "(" << mynode->position << " " << mynode->cour << ") " << mynode->prev->cumulatedEnd << "(" << mynode->prev->position << " " << mynode->prev->cour << ") " << endl; system("pause"); }
		mynode = mynode->next;
		myposition++;
	}
	mynode = myRoute->head->prev;
	myposition = myRoute->nbTrucks - 1;
	while (!mynode->ishead) {
		if (mynode->position != myposition)
		{
			cout << "D back scheduling: " << myRoute->nbTrucks << ": " << mynode->cour << "(" << mynode->position << ") and myposition " << myposition << endl;
			system("pause");
		}
		mynode = mynode->prev;
		myposition += -1;;
	}
}
//update the door's time warp, cost and the number when the door was modified last.
void LocalSearch::updateRouteDataO(Route * myRoute, int changeNB)
{
	Node * mynode = myRoute->head->next;
	myRoute->whenLastModified = nbMoves;
	myRoute->nbTrucks += changeNB;
	myRoute->WTime = 0;
	myRoute->penalty = 0;
	TotalTimeWarp += -myRoute->TimeWarp;
	myRoute->TimeWarp = 0;
	while (!mynode->ishead) {
		//std::cout << mynode->cour << " ";
		myRoute->WTime += mynode->WTimeCost;
		myRoute->penalty += mynode->TransCost;
		if (!mynode->isfeasible) {
			myRoute->TimeWarp += params.ITK[mynode->cour].GT;
		}
		mynode = mynode->next;
	}
	TotalTimeWarp += myRoute->TimeWarp;
	mynode = myRoute->head->next;
	int myposition = 0;
	while (!mynode->ishead) {
		if (mynode->position != myposition)
		{
			cout << "O forward scheduling: " << mynode->cour << "(" << mynode->position << ") and myposition " << myposition << endl;
			system("pause");
		}
		mynode = mynode->next;
		myposition++;
	}
	mynode = myRoute->head->prev;
	myposition = myRoute->nbTrucks - 1;
	while (!mynode->ishead) {
		if (mynode->position != myposition)
		{
			cout << "O back scheduling: " << mynode->cour << "(" << mynode->position << ") and myposition " << myposition << endl;
			system("pause");
		}
		mynode = mynode->prev;
		myposition += -1;
	}
}
//scheduling the trucks of current door, i.e, initial start time, cost and isfeasible 
void LocalSearch::initial_DoorsO(Route*myDoor) {
	//int myplace = 0;
	int  mytime = 0;

	Node * mynode = myDoor->head;
	mynode->position = -1;
	mynode->cumulatedStart = 0.;
	mynode->cumulatedEnd = 0.;
	mynode->WTimeCost = 0;
	mynode->TransCost = 0;
	//mynode->whenLastTestedRI = -1;
	mynode = mynode->next;
	myDoor->penalty = 0;
	myDoor->WTime = 0;
	myDoor->TimeWarp = 0;

	while (!mynode->ishead)
	{
		mytime = std::max(mytime, params.ITK[mynode->cour].AT);
		mynode->cumulatedStart = mytime;
		mynode->cumulatedEnd = mytime + params.ITK[mynode->cour].GT;
		if (mynode->cumulatedEnd <= params.ITK[mynode->cour].LF) { mytime = mynode->cumulatedEnd; mynode->isfeasible = true; }//feasible
		else { mynode->isfeasible = false; myDoor->TimeWarp += params.ITK[mynode->cour].GT; }//infeasible
		mynode->WTimeCost = params.funOTO[mynode->cour][mynode->cumulatedStart];
		myDoor->WTime += mynode->WTimeCost;
		mynode = mynode->next;

	}
	myDoor->whenLastModified = nbMoves;

}
//back scheduling the trucks of current door, i.e, initial start time, cost and isfeasible 
void LocalSearch::initial_DoorsD(Route*myDoor) {//back scheduling
	//int myplace = 0;
	int  mytime = 99999;
	Node * mynode = myDoor->head;
	mynode->position = -1;
	mynode->cumulatedStart = 0.;
	mynode->cumulatedEnd = 0.;
	mynode->WTimeCost = 0;
	mynode->TransCost = 0;
	mynode = mynode->prev;//end node
	myDoor->penalty = 0;
	myDoor->WTime = 0;
	myDoor->TimeWarp = 0;
	while (!mynode->ishead)
	{
		mytime = std::min(mytime, params.OTK[mynode->cour].LF);
		mynode->cumulatedEnd = mytime;
		mynode->cumulatedStart = mytime - params.OTK[mynode->cour].GT;
		if (mynode->cumulatedStart >= params.OTK[mynode->cour].AT) { mytime += -params.OTK[mynode->cour].GT; mynode->isfeasible = true; }
		else { mynode->isfeasible = false; myDoor->TimeWarp += params.OTK[mynode->cour].GT; }
		mynode->WTimeCost = params.funOTD[mynode->cour][mynode->cumulatedStart];
		myDoor->WTime += mynode->WTimeCost;
		mynode = mynode->prev;

	}
	// Remember "when" this route has been last modified (will be used to filter unnecessary move evaluations)
	myDoor->whenLastModified = nbMoves;
}
void LocalSearch::schedulingY()
{
	int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	int comcost = ComputeCost(saa.sty);
	if (refineBest != comcost) {
		cout << "initial determinCost =" << refineBest << " Com" << comcost << endl;
		cout << "SequenceX before schedulingY" << endl;
		for (int i = 0; i < params.nbInDoors; i++) {
			for (int j = 1; j <= saa.SequenceX[i][0]; j++) {
				cout << "(" << saa.SequenceX[i][j] << "," << saa.stx[saa.SequenceX[i][j]] << ") ";
			}
			cout << endl;
		}
		cout << "SequenceY before schedulingY" << endl;
		for (int i = 0; i < params.nbOutDoors; i++) {
			for (int j = 1; j <= saa.SequenceY[i][0]; j++) {
				cout << "(" << saa.SequenceY[i][j] << "," << saa.sty[saa.SequenceY[i][j]] << ") ";
			}
			cout << endl;
		}
	}
	initialTrans();
	iniDltVector(refineBest);
	while (1) {
		select_j = -1;
		min_delta = 0;
		ff = -1;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			if (trucksD[i].isfeasible) {
				if (min_delta > DltVec[0][i]) {
					min_delta = DltVec[0][i];
					select_j = i;
					buf = 1;
					ff = 0;
				}
				if (min_delta > DltVec[1][i]) {
					min_delta = DltVec[1][i];
					select_j = i;
					buf = -1;
					ff = 1;
				}
			}
		}
		if (select_j != -1) {
			refineBest += DltVec[ff][select_j];
			MoveFastUpdate(select_j, buf);
		}
		else {
			break;
		}
	}

	int Comlocal = 0;
	Comlocal = ComputeCost(saa.sty);
	if (Comlocal != refineBest) {
		cout << "Scheduling Deterlocal " << refineBest << " and Comlocal " << Comlocal << endl;
		cout << "SequenceX after schedulingY" << endl;
		for (int i = 0; i < params.nbInDoors; i++) {
			for (int j = 1; j <= saa.SequenceX[i][0]; j++) {
				cout << "(" << saa.SequenceX[i][j] << "," << saa.stx[saa.SequenceX[i][j]] << ") ";
			}
			cout << endl;
		}
		cout << "SequenceY after schedulingY" << endl;
		for (int i = 0; i < params.nbOutDoors; i++) {
			for (int j = 1; j <= saa.SequenceY[i][0]; j++) {
				cout << "(" << saa.SequenceY[i][j] << "," << saa.sty[saa.SequenceY[i][j]] << ") ";
			}
			cout << endl;
		}
		system("pause");
	}
	int doox;
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (saa.soly[i] >= 0 && (saa.sty[i]<params.OTK[i].AT || saa.sty[i]>params.OTK[i].LS)) {
			saa.soly[i] = -1;
			saa.sty[i] = -1;
			doox = trucksD[i].route->cour;
			for (int k = saa.L[i] + 1; k <= saa.SequenceY[doox][0]; k++) {
				saa.SequenceY[doox][k - 1] = saa.SequenceY[doox][k];
				saa.L[saa.SequenceY[doox][k - 1]] = k - 1;
			}
			saa.SequenceY[doox][0] += -1;
		}
	}
}
void LocalSearch::iniDltVector(int _refineBest)
{
	int total;
	int  *tmpsty;
	int postid = 0;
	int preid = 0;
	tmpsty = new int[params.nbOutTrucks];
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (trucksD[i].isfeasible) {
			DltVec[0][i] = FastComputeDlt(i, 1);
			DltVec[1][i] = FastComputeDlt(i, -1);
		}
	}
	delete[] tmpsty;
}
int LocalSearch::FastComputeDlt(int kth, int buf)
{
	//Node *tmp;
	int delta_cost = 0;
	int tmpsty;
	int transtime;
	int tmptime1;
	int tmptime2;
	int preid;
	int door = 0;
	int postid;
	if (buf > 0) {
		tmpsty = saa.sty[kth] + buf;
		door = trucksD[kth].route->cour;
		delta_cost += params.funOTD[kth][tmpsty] - params.funOTD[kth][saa.sty[kth]];
		for (int i = 1; i <= params.OTK[kth].Pre[0]; i++) {
			preid = params.OTK[kth].Pre[i];
			if (trucksO[preid].isfeasible) {
				transtime = params.Time[preid][kth] * params.Dis[trucksO[preid].route->cour][door];
				tmptime2 = max(tmpsty - transtime, 0);
				if (tmptime2 >= 0 && tmpsty >= params.OTK[kth].AT&&tmpsty <= params.OTK[kth].LS) {
					delta_cost += -(params.funTrans[preid][kth][saa.stx[preid]][tmptime2] - transcost[preid][kth])* params.FP[preid][kth];
				}
				else {
					delta_cost += transcost[preid][kth] * params.FP[preid][kth];
				}
			}
		}
		tmpsty += params.OTK[kth].GT;
		for (int l = saa.L[kth] + 1; l <= saa.SequenceY[door][0]; l++) {//2 5 7 8, if kth=2,then L[th]=1, 5，7，8 all need to be evaluated
			postid = saa.SequenceY[door][l];
			if ((tmpsty > saa.sty[postid]) && (tmpsty <= params.OTK[saa.SequenceY[door][l - 1]].LF)) {//the truck postid was affected

				delta_cost += params.funOTD[postid][tmpsty] - params.funOTD[postid][saa.sty[postid]];
				for (int j = 1; j <= params.OTK[postid].Pre[0]; j++) {
					preid = params.OTK[postid].Pre[j];// 出站卡车postid的入站卡车id
					if (trucksO[preid].isfeasible) {
						transtime = params.Time[preid][postid] * params.Dis[trucksO[preid].route->cour][door];
						tmptime2 = tmpsty - transtime;
						if (tmptime2 >= 0 && tmpsty >= params.OTK[postid].AT&&tmpsty <= params.OTK[postid].LS) {
							delta_cost += -(params.funTrans[preid][postid][saa.stx[preid]][tmptime2] - transcost[preid][postid])* params.FP[preid][postid];
						}
						else {
							delta_cost += transcost[preid][postid] * params.FP[preid][postid];
						}
					}
				}
				tmpsty += params.OTK[postid].GT;
			}
			else break;
		}
	}
	else {
		tmpsty = saa.sty[kth] + buf;
		door = trucksD[kth].route->cour;
		delta_cost += params.funOTD[kth][tmpsty] - params.funOTD[kth][saa.sty[kth]];//delta_OverTime

		for (int j = 1; j <= params.OTK[kth].Pre[0]; j++) {
			preid = params.OTK[kth].Pre[j];// 出站卡车kth的入站卡车id
			if (trucksO[preid].isfeasible) {
				transtime = params.Time[preid][kth] * params.Dis[trucksO[preid].route->cour][door];
				tmptime2 = tmpsty - transtime;
				if (tmptime2 >= 0 && tmpsty >= params.OTK[kth].AT&&tmpsty <= params.OTK[kth].LS) {
					delta_cost += -(params.funTrans[preid][kth][saa.stx[preid]][tmptime2] - transcost[preid][kth])* params.FP[preid][kth];
				}
				else {
					delta_cost += transcost[preid][kth] * params.FP[preid][kth];
				}
			}
		}
		for (int l = saa.L[kth] - 1; l >= 1; l--) {
			postid = saa.SequenceY[door][l];
			//if:next truck's tmp start time < current truck's end time and next truck was feasible, the current truck need to be evaluated.
			if ((tmpsty < saa.sty[postid] + params.OTK[postid].GT) && (tmpsty >= params.OTK[saa.SequenceY[door][l + 1]].AT)) {//the truck postid was affected
				tmpsty += -params.OTK[postid].GT;// current truck start time
				delta_cost += params.funOTD[postid][tmpsty] - params.funOTD[postid][saa.sty[postid]];
				for (int j = 1; j <= params.OTK[postid].Pre[0]; j++) {
					preid = params.OTK[postid].Pre[j];// 出站卡车postid的入站卡车id
					if (trucksO[preid].isfeasible) {
						transtime = params.Time[preid][postid] * params.Dis[trucksO[preid].route->cour][door];
						tmptime2 = tmpsty - transtime;
						if (tmptime2 >= 0 && tmpsty >= params.OTK[postid].AT&&tmpsty <= params.OTK[postid].LS) {
							delta_cost += -(params.funTrans[preid][postid][saa.stx[preid]][tmptime2] - transcost[preid][postid])* params.FP[preid][postid];
						}
						else {
							delta_cost += transcost[preid][postid] * params.FP[preid][postid];
						}
					}
				}
			}
			else break;
		}
	}
	return delta_cost;
}
void LocalSearch::MoveFastUpdate(int kth, int buf)
{
	int door = trucksD[kth].route->cour;
	int tmpst = 0;
	int endindex;
	int *tmpsty;
	int total;
	int preid;
	int transtime;
	int postid;
	int tmptime;
	tmpsty = new int[params.nbOutTrucks];

	saa.sty[kth] += buf;
	endindex = saa.L[kth];
	for (int j = 1; j <= params.OTK[kth].Pre[0]; j++) {
		preid = params.OTK[kth].Pre[j];
		transtime = params.Time[preid][kth] * params.Dis[trucksO[preid].route->cour][door];
		tmptime = max(saa.sty[kth] - transtime, 0);
		if (trucksO[preid].isfeasible && saa.sty[kth] <= params.OTK[kth].LS &&saa.sty[kth] >= params.OTK[kth].AT)
			transcost[preid][kth] = params.funTrans[preid][kth][saa.stx[preid]][tmptime];
		else transcost[preid][kth] = 0;
	}
	if (buf > 0) {//update solution sty
		tmpst = saa.sty[kth] - buf;
		for (int i = saa.L[kth] - 1; i >= 1; i--) {
			if (tmpst == saa.sty[saa.SequenceY[door][i]] + params.OTK[saa.SequenceY[door][i]].GT) {//tmpst >= sty[SequenceY[door][i]] + g[SequenceY[door][i] + M]
				DltVec[0][saa.SequenceY[door][i]] += -DltVec[0][saa.SequenceY[door][saa.L[kth]]];
				tmpst = saa.sty[saa.SequenceY[door][i]];
			}
			else {
				break;
			}
		}
		bool isfind = true;
		for (int l = saa.L[kth] + 1; l <= saa.SequenceY[door][0]; l++) {
			postid = saa.SequenceY[door][l];
			tmpst = saa.sty[saa.SequenceY[door][l - 1]] + params.OTK[saa.SequenceY[door][l - 1]].GT;//END TIME OF previous truck
			if (isfind && tmpst > saa.sty[postid] && tmpst <= params.OTK[saa.SequenceY[door][l - 1]].LF) {
				saa.sty[postid] = tmpst;
				for (int j = 1; j <= params.OTK[postid].Pre[0]; j++) {
					preid = params.OTK[postid].Pre[j];
					transtime = params.Time[preid][postid] * params.Dis[trucksO[preid].route->cour][door];
					tmptime = max(saa.sty[postid] - transtime, 0);
					if (trucksO[preid].isfeasible && saa.sty[postid] <= params.OTK[postid].LS &&saa.sty[postid] >= params.OTK[postid].AT)
						transcost[preid][postid] = params.funTrans[preid][postid][saa.stx[preid]][tmptime];
					else transcost[preid][postid] = 0;
				}
			}
			else { isfind = false; }
			if (tmpst >= saa.sty[postid]) {
				endindex = l;
			}
			else break;
		}
		for (int l = saa.L[kth]; l <= endindex; l++) {
			DltVec[0][saa.SequenceY[door][l]] = FastComputeDlt(saa.SequenceY[door][l], 1);
			DltVec[1][saa.SequenceY[door][l]] = FastComputeDlt(saa.SequenceY[door][l], -1);
		}
	}
	else {
		tmpst = saa.sty[kth] - buf;
		for (int i = saa.L[kth] + 1; i <= saa.SequenceY[door][0]; i++) {
			if (tmpst + params.OTK[saa.SequenceY[door][i - 1]].GT == saa.sty[saa.SequenceY[door][i]]) {//tmpst >= sty[SequenceY[door][i]] + g[SequenceY[door][i] + M]
				DltVec[1][saa.SequenceY[door][i]] += -DltVec[1][saa.SequenceY[door][saa.L[kth]]];
				tmpst = saa.sty[saa.SequenceY[door][i]];//previous truck's start time
			}
			else {
				break;
			}
		}
		bool isfind = true;
		for (int l = saa.L[kth] - 1; l >= 1; l--) {
			postid = saa.SequenceY[door][l];
			tmpst = saa.sty[saa.SequenceY[door][l + 1]] - params.OTK[postid].GT;
			if (isfind&&tmpst < saa.sty[postid] && saa.sty[saa.SequenceY[door][l + 1]] >= params.OTK[saa.SequenceY[door][l + 1]].AT) {
				saa.sty[postid] = tmpst;
				for (int j = 1; j <= params.OTK[postid].Pre[0]; j++) {
					preid = params.OTK[postid].Pre[j];
					transtime = params.Time[preid][postid] * params.Dis[trucksO[preid].route->cour][door];
					tmptime = max(saa.sty[postid] - transtime, 0);
					if (trucksO[preid].isfeasible && saa.sty[postid] <= params.OTK[postid].LS &&saa.sty[postid] >= params.OTK[postid].AT)
						transcost[preid][postid] = params.funTrans[preid][postid][saa.stx[preid]][tmptime];
					else transcost[preid][postid] = 0;
				}
			}
			else { isfind = false; }
			if (tmpst <= saa.sty[postid]) {
				endindex = l;
			}
			else break;
		}
		for (int l = endindex; l <= saa.L[kth]; l++) {
			DltVec[0][saa.SequenceY[door][l]] = FastComputeDlt(saa.SequenceY[door][l], 1);
			DltVec[1][saa.SequenceY[door][l]] = FastComputeDlt(saa.SequenceY[door][l], -1);
		}
	}
	delete[] tmpsty;
}
int LocalSearch::ComputeCost(int * st)
{
	int obj = 0;
	int truck_id = 0;
	int pre_id = 0;
	int transtime;
	int tmpsty = 0;
	for (int i = 0; i < params.nbInTrucks; i++) {
		if (trucksO[i].isfeasible) {
			obj += params.funOTO[i][saa.stx[i]];
		}
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		for (int l = 1; l <= saa.SequenceY[j][0]; l++) {
			truck_id = saa.SequenceY[j][l];
			if (st[truck_id] <= params.OTK[truck_id].LS&&st[truck_id] >= params.OTK[truck_id].AT) {
				obj += params.funOTD[truck_id][st[truck_id]];
				for (int k = 1; k <= params.OTK[truck_id].Pre[0]; k++) {
					pre_id = params.OTK[truck_id].Pre[k];
					if (trucksO[pre_id].isfeasible) {
						transtime = params.Time[pre_id][truck_id] * params.Dis[trucksO[pre_id].route->cour][trucksD[truck_id].route->cour];
						if (st[truck_id] >= transtime)
							obj += -params.FP[pre_id][truck_id] * params.funTrans[pre_id][truck_id][saa.stx[pre_id]][st[truck_id] - transtime];
					}
				}
			}
		}
	}
	return obj + params.TotalPenality;
}


void LocalSearch::initialTrans()
{
	int preid;
	int transtime;
	int door;
	int tmptime;
	for (int i = 0; i < params.nbOutTrucks; i++) {
		door = trucksD[i].route->cour;
		if (trucksD[i].isfeasible) {
			for (int j = 1; j <= params.OTK[i].Pre[0]; j++) {
				preid = params.OTK[i].Pre[j];
				transtime = params.Time[preid][i] * params.Dis[trucksO[preid].route->cour][door];
				tmptime = max(saa.sty[i] - transtime, 0);
				if (trucksO[preid].isfeasible)
					transcost[preid][i] = params.funTrans[preid][i][saa.stx[preid]][tmptime];
				else transcost[preid][i] = 0;
			}
		}
	}
}
void LocalSearch::loadIndividual(const Individual & indiv)//link the trucks and head ndoe
{
	int mypalce;
	nbMoves = 0;
	TotalTimeWarp = 0;
	local_best = 999999999;
	for (int r = 0; r < params.nbInDoors; r++)
	{
		Node * myhead = &headsO[r];
		Route * myDoor = &doorsO[r];
		myhead->next = myhead;
		myhead->prev = myhead;//当作空门处理
		mypalce = 0;
		if (indiv.chromRO[r][0] > 0) {
			Node * myTruck = &trucksO[indiv.chromRO[r][1]];
			myTruck->route = myDoor;
			myTruck->prev = myhead;
			myhead->next = myTruck;
			myTruck->position = mypalce;
			mypalce++;
			for (int i = 2; i <= (int)indiv.chromRO[r][0]; i++)
			{

				Node * myClientPred = myTruck;
				myTruck = &trucksO[indiv.chromRO[r][i]];
				myTruck->prev = myClientPred;
				myClientPred->next = myTruck;
				myTruck->route = myDoor;
				myTruck->position = mypalce;
				mypalce++;
			}
			myTruck->next = myhead;
			myhead->prev = myTruck;
		}
		doorsO[r].whenLastTestedSWAPStar = -1;
		doorsO[r].whenLastModified = -1;
		myDoor->nbTrucks = mypalce;
		initial_DoorsO(&doorsO[r]);//need doorsD
		TotalTimeWarp += doorsO[r].TimeWarp;
	}
	for (int i = 0; i < params.nbInTrucks; i++) // Initializing memory structures
		trucksO[i].whenLastTestedRI = -1;

	for (int r = 0; r < params.nbOutDoors; r++)
	{
		Node * myhead = &headsD[r];
		Route * myDoor = &doorsD[r];
		myhead->next = myhead;
		myhead->prev = myhead;//当作空门处理
		mypalce = 0;
		if (indiv.chromRD[r][0] > 0) {
			Node * myTruck = &trucksD[indiv.chromRD[r][1]];
			myTruck->route = myDoor;
			myTruck->prev = myhead;
			myhead->next = myTruck;
			myTruck->position = mypalce;
			mypalce++;
			for (int i = 2; i <= (int)indiv.chromRD[r][0]; i++)
			{

				Node * myClientPred = myTruck;
				myTruck = &trucksD[indiv.chromRD[r][i]];
				myTruck->prev = myClientPred;
				myClientPred->next = myTruck;
				myTruck->route = myDoor;
				myTruck->position = mypalce;
				mypalce++;
			}
			myTruck->next = myhead;
			myhead->prev = myTruck;

		}
		myDoor->nbTrucks = mypalce;
		doorsD[r].whenLastTestedSWAPStar = -1;
		doorsD[r].whenLastModified = -1;
		initial_DoorsD(&doorsD[r]);
		TotalTimeWarp += doorsD[r].TimeWarp;
	}
	for (int i = 0; i < params.nbOutTrucks; i++) // Initializing memory structures
		trucksD[i].whenLastTestedRI = -1;

	for (int r = 0; r < params.nbInDoors; r++) {
		Node*tmp = headsO[r].next;//start node
		Route *Door = &doorsO[r];
		Door->penalty = 0;
		while (!tmp->ishead) {
			tmp->TransCost = computeIncost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
			Door->penalty += tmp->TransCost;
			tmp = tmp->next;

		}
	}
	for (int r = 0; r < params.nbOutDoors; r++) {
		Node*tmp = headsD[r].next;//start node
		Route *Door = &doorsD[r];
		Door->penalty = 0;
		while (!tmp->ishead) {
			tmp->TransCost = computeOutcost(tmp->cour, tmp->cumulatedStart, tmp->route->cour);
			Door->penalty += tmp->TransCost;
			tmp = tmp->next;
		}
	}
	for (int i = 0; i < params.nbInDoors; i++) {
		determinedCost += doorsO[i].WTime + doorsO[i].penalty;
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		determinedCost += doorsD[j].WTime;
	}
	determinedCost += params.TotalPenality;
}

int LocalSearch::ComputeDis_O(const Individual & Parent2)
{
	int mint = 0;
	int maxt = 0;
	int count = 0;
	Node *tmp;
	for (int i = 0; i < params.nbInDoors; i++) {
		mint = std::min(doorsO[i].nbTrucks, Parent2.chromRO[i][0]);
		tmp = doorsO[i].head;
		for (int j = 1; j <= mint; j++) {
			tmp = tmp->next;
			if (tmp->cour != Parent2.chromRO[i][j]) { count++; }
		}
	}
	return count;
}

int LocalSearch::ComputeDis_D(const Individual & Parent2)
{
	int mint = 0;
	int maxt = 0;
	int count = 0;
	Node *tmp;
	for (int i = 0; i < params.nbOutDoors; i++) {
		mint = std::min(doorsD[i].nbTrucks, Parent2.chromRD[i][0]);
		tmp = doorsD[i].head;
		for (int j = 1; j <= mint; j++) {
			tmp = tmp->next;
			if (tmp->cour != Parent2.chromRD[i][j]) { count++; }
		}
	}
	return count;
}

void LocalSearch::exportIndividual(Individual & indiv)
{

	int pos = 0;
	int cnt = 0;
	for (int r = 0; r < params.nbInDoors; r++)
	{
		Node * node = doorsO[r].head->next;//thr first truck node
		cnt = 0;
		while (!node->ishead)
		{
			cnt++;
			indiv.chromTO[pos] = node->cour;
			indiv.chromRO[r][cnt] = node->cour;
			indiv.AssignO[node->cour] = r;
			indiv.StimeO[node->cour] = node->cumulatedStart;
			indiv.successorsO[node->cour] = node->next->cour;//if node->next->ishead, then the cour is -1
			indiv.predecessorsO[node->cour] = node->prev->cour;//if node->prev->ishead, then the cour is -1
			indiv.penaltyWTimeO[node->cour] = node->WTimeCost;
			indiv.penaltyTrans[node->cour] = node->TransCost;
			node = node->next;
			pos++;
		}
		indiv.chromRO[r][0] = cnt;
		indiv.eval.TimeWarp += doorsO[r].TimeWarp;
	}
	pos = 0;
	for (int r = 0; r < params.nbOutDoors; r++)
	{
		Node * node = doorsD[r].head->next;
		cnt = 0;
		while (!node->ishead)
		{
			cnt++;
			indiv.chromTD[pos] = node->cour;
			indiv.chromRD[r][cnt] = node->cour;
			indiv.AssignD[node->cour] = r;
			indiv.StimeD[node->cour] = node->cumulatedStart;
			indiv.successorsD[node->cour] = node->next->cour;
			indiv.predecessorsD[node->cour] = node->prev->cour;
			indiv.penaltyWTimeD[node->cour] = node->WTimeCost;
			node = node->next;
			pos++;
		}
		indiv.chromRD[r][0] = cnt;
		indiv.eval.TimeWarp += doorsD[r].TimeWarp;
	}

	indiv.evaluateCompleteCost(params);
	std::cout << "evaluateCompleteCost " << indiv.eval.penalizedCost << " indiv.eval.timewarp " << indiv.eval.TimeWarp << std::endl;
	indiv.evaluateCompleteCost2(params);
	std::cout << "evaluateCompleteCost2 " << indiv.eval.penalizedCost << " indiv.eval.timewarp " << indiv.eval.TimeWarp << std::endl;
}
//local_sequence to invidual
void LocalSearch::exportIndividualStoch(Individual & indiv)
{
	int pos = 0;
	int cnt = 0;
	int tmp;
	for (int r = 0; r < params.nbInDoors; r++) {
		cnt = 0;
		for (int j = 1; j <= Local_SequenceX[r][0]; j++) {
			cnt++;
			tmp = Local_SequenceX[r][j];
			indiv.chromTO[pos] = tmp;
			indiv.chromRO[r][cnt] = tmp;
			indiv.AssignO[tmp] = r;
			indiv.StimeO[tmp] = local_StimeO[tmp];
			if (j + 1 <= Local_SequenceX[r][0]) {
				indiv.successorsO[tmp] = Local_SequenceX[r][j + 1];//if node->next->ishead, then the cour is -1
			}
			else { indiv.successorsO[tmp] = -1; }
			if (j != 1) {
				indiv.predecessorsO[tmp] = Local_SequenceX[r][j - 1];
			}//if node->prev->ishead, then the cour is -1
			else { indiv.predecessorsO[tmp] = -1; }
			pos++;
		}
		indiv.chromRO[r][0] = cnt;
	}
	pos = 0;
	for (int r = 0; r < params.nbOutDoors; r++) {
		cnt = 0;
		for (int j = 1; j <= Local_SequenceY[r][0]; j++) {
			cnt++;
			tmp = Local_SequenceY[r][j];
			indiv.chromTD[pos] = tmp;
			indiv.chromRD[r][cnt] = tmp;
			indiv.AssignD[tmp] = r;
			indiv.StimeD[tmp] = local_StimeD[tmp];
			if (j + 1 <= Local_SequenceY[r][0]) {
				indiv.successorsD[tmp] = Local_SequenceY[r][j + 1];//if node->next->ishead, then the cour is -1
			}
			else { indiv.successorsD[tmp] = -1; }
			if (j != 1) {
				indiv.predecessorsD[tmp] = Local_SequenceY[r][j - 1];
			}//if node->prev->ishead, then the cour is -1
			else { indiv.predecessorsD[tmp] = -1; }
			pos++;
		}
		indiv.chromRD[r][0] = cnt;
	}

	indiv.evaluateCompleteCost(params);//computer determinedCost
	indiv.eval.penalizedCost = local_best;
	indiv.eval.Ini_LB = refineBest;
	indiv.eval.Ini_Sim_LB = ini_sim_LB;
	
}
void LocalSearch::exportSAAData() {
	Node*tmp;
	int cnt = 0;
	for (int i = 0; i < params.nbInDoors; i++) {
		tmp = headsO[i].next;//start node
		cnt = 0;
		while (!tmp->ishead) {
			if (tmp->isfeasible) {
				cnt++;
				saa.solx[tmp->cour] = i;
				saa.stx[tmp->cour] = tmp->cumulatedStart;
				saa.SequenceX[i][cnt] = tmp->cour;
			}
			else {
				saa.solx[tmp->cour] = -1;
				saa.stx[tmp->cour] = -1;
			}
			tmp = tmp->next;
		}
		saa.SequenceX[i][0] = cnt;
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		tmp = headsD[i].next;//start node
		cnt = 0;

		while (!tmp->ishead) {
			if (tmp->isfeasible) {
				cnt++;
				saa.soly[tmp->cour] = i;
				saa.sty[tmp->cour] = tmp->cumulatedStart;
				saa.SequenceY[i][cnt] = tmp->cour;
				saa.L[tmp->cour] = cnt;
			}
			else {
				saa.L[tmp->cour] = -1;
				saa.soly[tmp->cour] = -1;
				saa.sty[tmp->cour] = -1;
			}
			tmp = tmp->next;
		}
		saa.SequenceY[i][0] = cnt;
	}
}
//update
void LocalSearch::loadSAA() {
	local_best = saa.local_best;
	ini_sim_LB = saa.ini_sim_local;
	Node *tmp;
	for (int i = 0; i < params.nbInDoors; i++) {
		tmp = headsO[i].next;//start node
		while (!tmp->ishead) {
			Local_SequenceX[i][tmp->position + 1] = tmp->cour;
			tmp = tmp->next;
		}
		Local_SequenceX[i][0] = doorsO[i].nbTrucks;
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		tmp = headsD[i].next;//start node
		while (!tmp->ishead) {
			Local_SequenceY[i][tmp->position + 1] = tmp->cour;
			tmp = tmp->next;
		}
		Local_SequenceY[i][0] = doorsD[i].nbTrucks;
	}
	for (int i = 0; i < params.nbInTrucks; i++) {
		local_StimeO[i] = saa.stx[i];
	}
	for (int i = 0; i < params.nbOutTrucks; i++) {
		local_StimeD[i] = saa.sty[i];
	}

}
void LocalSearch::loadScheduling() {
	local_best = refineBest;
	Node *tmp;
	for (int i = 0; i < params.nbInDoors; i++) {
		tmp = headsO[i].next;//start node
		while (!tmp->ishead) {
			Local_SequenceX[i][tmp->position + 1] = tmp->cour;
			tmp = tmp->next;
		}
		Local_SequenceX[i][0] = doorsO[i].nbTrucks;
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		tmp = headsD[i].next;//start node
		while (!tmp->ishead) {
			Local_SequenceY[i][tmp->position + 1] = tmp->cour;
			tmp = tmp->next;
		}
		Local_SequenceY[i][0] = doorsD[i].nbTrucks;
	}
	for (int i = 0; i < params.nbInTrucks; i++) {
		local_StimeO[i] = saa.stx[i];
	}
	for (int i = 0; i < params.nbOutTrucks; i++) {
		local_StimeD[i] = saa.sty[i];
	}

}
void LocalSearch::checkOBJO()
{
	int local_best1 = 0;
	for (int i = 0; i < params.nbInDoors; i++) {
		local_best1 += doorsO[i].WTime + doorsO[i].penalty;
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		local_best1 += doorsD[j].WTime;
	}
	local_best1 += params.TotalPenality;
	if (local_best1 != determinedCost) {
		cout << "In " << local_best1 << " " << determinedCost << endl;
		system("pause");
	}
}

void LocalSearch::checkOBJD(int delta)
{
	int local_best1 = 0;
	for (int i = 0; i < params.nbInDoors; i++) {
		local_best1 += doorsO[i].WTime;
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		local_best1 += doorsD[j].WTime + doorsD[j].penalty;
	}
	local_best1 += params.TotalPenality;
	if (local_best1 != determinedCost) {
		cout << "Out " << local_best1 << " " << determinedCost << " " << delta << endl;
		system("pause");
	}
}
void LocalSearch::computeCostO(int delta, int x)
{
	int tempcost = 0;
	Node*tmp;
	for (int i = 0; i < params.nbInDoors; i++) {
		tmp = headsO[i].next;
		while (!tmp->ishead) {
			tempcost += tmp->WTimeCost + tmp->TransCost;
			tmp = tmp->next;
		}
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		tmp = headsD[j].next;
		while (!tmp->ishead) {
			tempcost += tmp->WTimeCost;
			tmp = tmp->next;
		}
	}
	tempcost += params.TotalPenality;
	if (tempcost != determinedCost) {
		cout << "In1 (" << x << ") " << tempcost << " " << determinedCost << " " << delta << endl;
		system("pause");
	}
	tempcost = 0;
	int otk;
	int transtime;
	for (int r = 0; r < params.nbInTrucks; r++)
	{
		tempcost += params.funOTO[r][trucksO[r].cumulatedStart];
		//cout<<
		if (trucksO[r].cumulatedStart <= params.ITK[r].LS) {
			for (int j = 1; j <= params.ITK[r].Suc[0]; j++) {
				otk = params.ITK[r].Suc[j];
				transtime = params.Time[r][otk] * params.Dis[trucksO[r].route->cour][trucksD[otk].route->cour];
				if (trucksD[otk].cumulatedStart >= params.OTK[otk].AT&&trucksD[otk].cumulatedStart >= transtime) {
					tempcost += -params.FP[r][otk] * params.funTrans[r][otk][trucksO[r].cumulatedStart][trucksD[otk].cumulatedStart - transtime];
				}
			}
		}
	}

	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (trucksD[i].isfeasible) {
			tempcost += params.funOTD[i][trucksD[i].cumulatedStart];
		}
	}
	tempcost += params.TotalPenality;
	if (tempcost != determinedCost) {
		cout << "In2 (" << x << ") " << tempcost << " " << determinedCost << " " << delta << endl;
		system("pause");
	}
}

void LocalSearch::computeCostD(int delta, int x)
{
	int tempcost = 0;
	Node*tmp;
	for (int i = 0; i < params.nbInDoors; i++) {
		tmp = headsO[i].next;
		while (!tmp->ishead) {
			tempcost += tmp->WTimeCost;
			tmp = tmp->next;
		}
	}
	for (int j = 0; j < params.nbOutDoors; j++) {
		tmp = headsD[j].next;
		while (!tmp->ishead) {
			tempcost += tmp->WTimeCost + tmp->TransCost;
			tmp = tmp->next;
		}
	}
	tempcost += params.TotalPenality;
	if (tempcost != determinedCost) {
		cout << "Out1 (" << x << ") " << tempcost << " " << determinedCost << " " << delta << endl;
		system("pause");
	}
	tempcost = 0;
	int otk;
	int transtime;
	for (int r = 0; r < params.nbInTrucks; r++)
	{
		tempcost += params.funOTO[r][trucksO[r].cumulatedStart];
	}
	int itk = 0;
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (trucksD[i].cumulatedStart >= params.OTK[i].AT) {
			tempcost += params.funOTD[i][trucksD[i].cumulatedStart];
			for (int j = 1; j <= params.OTK[i].Pre[0]; j++) {
				itk = params.OTK[i].Pre[j];
				transtime = params.Time[itk][i] * params.Dis[trucksO[itk].route->cour][trucksD[i].route->cour];
				if (trucksO[itk].cumulatedStart <= params.ITK[itk].LS&&trucksD[i].cumulatedStart >= transtime) {
					tempcost += -params.FP[itk][i] * params.funTrans[itk][i][trucksO[itk].cumulatedStart][trucksD[i].cumulatedStart - transtime];
				}
			}
		}
	}
	tempcost += params.TotalPenality;
	if (tempcost != determinedCost) {
		cout << "Out2 (" << x << ") " << tempcost << " " << determinedCost << " " << delta << endl;
		system("pause");
	}
}
void LocalSearch::perturb() {
	int sli, slj;
	int delta;
	for (int i = 0; i < 3; i++) {
		sli = rand() % params.nbInTrucks;
		slj = rand() % params.nbInTrucks;
		while (sli == slj) { slj = rand() % params.nbInTrucks; }
		nodeU = &trucksO[sli];
		nodeV = &trucksO[slj];
		setLocalVariablesRouteU();
		setLocalVariablesRouteV();
		nodeU->whenLastTestedRI = nbMoves;
		if (rand() % 2 == 1 || nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex) {
			delta = delta_swapO(nodeU, nodeV);
			determinedCost += delta;
			swapNode(nodeU, nodeV, intraRouteMove);
		}
		else {
			delta = delta_shiftO();
			determinedCost += delta;
			insertNode(nodeU, nodeV, intraRouteMove);
		}
		updateTrucksO();
		updateRouteDataO(routeU, changedNbTrucks[routeU->cour]);
		if (!intraRouteMove) updateRouteDataO(routeV, changedNbTrucks[routeV->cour]);
	}
	for (int i = 0; i < 3; i++) {
		sli = rand() % params.nbOutTrucks;
		slj = rand() % params.nbOutTrucks;
		while (sli == slj) { slj = rand() % params.nbOutTrucks; }
		nodeU = &trucksD[sli];
		nodeV = &trucksD[slj];
		setLocalVariablesRouteU();
		setLocalVariablesRouteV();
		nodeU->whenLastTestedRI = nbMoves;
		if (rand() % 2 == 1 || nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex) {
			delta = delta_swapD(nodeU, nodeV);
			determinedCost += delta;
			swapNode(nodeU, nodeV, intraRouteMove);
		}
		else {
			delta = delta_shiftD();
			determinedCost += delta;
			insertNode(nodeU, nodeV, intraRouteMove);
		}
		updateTrucksD();
		updateRouteDataD(routeU, changedNbTrucks[routeU->cour]);
		if (!intraRouteMove) updateRouteDataD(routeV, changedNbTrucks[routeV->cour]);
	}
}
void LocalSearch::perturbO() {
	int sli, slj;
	int delta;
	for (int i = 0; i < 3; i++) {
		sli = rand() % params.nbInTrucks;
		slj = rand() % params.nbInTrucks;
		while (sli == slj) { slj = rand() % params.nbInTrucks; }
		nodeU = &trucksO[sli];
		nodeV = &trucksO[slj];
		setLocalVariablesRouteU();
		setLocalVariablesRouteV();
		nodeU->whenLastTestedRI = nbMoves;
		if (rand() % 2 == 1 || nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex) {
			delta = delta_swapO(nodeU, nodeV);
			determinedCost += delta;
			swapNode(nodeU, nodeV, intraRouteMove);
		}
		else {
			delta = delta_shiftO();
			determinedCost += delta;
			insertNode(nodeU, nodeV, intraRouteMove);
		}
		updateTrucksO();
		updateRouteDataO(routeU, changedNbTrucks[routeU->cour]);
		if (!intraRouteMove) updateRouteDataO(routeV, changedNbTrucks[routeV->cour]);
	}
}
void LocalSearch::perturbD() {
	int sli, slj;
	int delta;
	for (int i = 0; i < 3; i++) {
		sli = rand() % params.nbOutTrucks;
		slj = rand() % params.nbOutTrucks;
		while (sli == slj) { slj = rand() % params.nbOutTrucks; }
		nodeU = &trucksD[sli];
		nodeV = &trucksD[slj];
		setLocalVariablesRouteU();
		setLocalVariablesRouteV();
		nodeU->whenLastTestedRI = nbMoves;
		if (rand() % 2 == 1 || nodeUIndex == nodeYIndex || nodeXIndex == nodeVIndex) {
			delta = delta_swapD(nodeU, nodeV);
			determinedCost += delta;
			swapNode(nodeU, nodeV, intraRouteMove);
		}
		else {
			delta = delta_shiftD();
			determinedCost += delta;
			insertNode(nodeU, nodeV, intraRouteMove);
		}
		updateTrucksD();
		updateRouteDataD(routeU, changedNbTrucks[routeU->cour]);
		if (!intraRouteMove) updateRouteDataD(routeV, changedNbTrucks[routeV->cour]);
	}
}
void LocalSearch::narrowSearch() {//*params.ap.tollerance
	double STARTtime, ENDtime;
	if (((int)(determinedCost*params.ap.tollerance) <= historicalBest + params.MAXOTime)) {
		refineBest = determinedCost;
		exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
		STARTtime = clock();
		schedulingY();//improve the start time  saa.sty with refine best
		ENDtime = clock();
		LSTIME += (double)(ENDtime - STARTtime) / CLOCKS_PER_SEC;
		double xxx = 0.0; //||xxx> double(rand()%100/100)
		if (refineBest < local_best) {//be possible to find improved local_best;
			STARTtime = clock();
			if (params.ap.useSAA) {
				saa.SAA_LS();//improving the local_best
				//saa.SAA_LS_SmpNumTest();
			}
			else {
				saa.SBD(params.ap.useBest);
				//saa.SBD_Fast_Update();
			}
			ENDtime = clock();
			SimulationTime += saa.SimTIME;
			SAATIME+= (double)(ENDtime - STARTtime) / CLOCKS_PER_SEC;
			if (local_best - 1e-5 > saa.local_best) {//find the improved local_best
				loadSAA();//only the stx and sty is involved in load. 
				localDetCost = ComputeCost(saa.sty);
			}
		}
		if (refineBest < historicalBest) { historicalBest = refineBest; }
	}
}
void LocalSearch::narrowSearch2() {//*params.ap.tollerance
	if (((int)(determinedCost*params.ap.tollerance) <= historicalBest + params.MAXOTime)) {
		refineBest = determinedCost;
		exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
		schedulingY();//improve the start time  saa.sty with refine best
		saa.generate_EST();
		saa.SGS2Small(saa.SmallSmpNum, saa.sty);//return value saa.local_best
		if (local_best - 1e-5 > saa.local_best) {//find the improved local_best
			loadSAA();//only the stx and sty is involved in load. 
			localDetCost = ComputeCost(saa.sty);
		}
		if (refineBest < historicalBest) { historicalBest = refineBest; }
	}
}
void LocalSearch::MultiStart(Individual & indiv, int iter, std::string file) {

	double st;
	double endt;

	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.SAA_LS();
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile1(file + "SAA.txt", ios::app);
	//myfile1 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile1.close();
	//std::cout << 1 << endl;
	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.SAA_LSF();
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile4(file + "SAAF.txt", ios::app);
	//myfile4 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile4.close();
	//std::cout << 2 << endl;

	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	long long int _local_best = 0;
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.generate_EST();
	//	//saa.SGS2(_local_best,saa.sty);
	//	saa.SGS2Small(saa.SmpNum, saa.sty);
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile6(file + "Simu.txt", ios::app);
	//myfile6 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile6.close();
	//std::cout << 6 << endl;

	srand(iter);
	st = clock();
	loadIndividual(indiv);
	for (int i = 0; i < 1000; i++) {
		long long int _local_best = 0;
		exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
		saa.generate_EST();
		saa.sortSenario();
		saa.SGS2(_local_best, saa.sty);
		//saa.SGS2Small(saa.SmpNum, saa.sty);
		perturb();
	}
	endt = clock();
	ofstream myfile7(file + "SimuN.txt", ios::app);
	myfile7 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	myfile7.close();
	std::cout << 7 << endl;

	srand(iter);
	st = clock();
	loadIndividual(indiv);
	for (int i = 0; i < 1000; i++) {
		long long int _local_best = 0;
		exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
		//saa.generate_EST();
		//saa.sortSenario();
		saa.SGST(_local_best, saa.sty);
		//saa.SGS2Small(saa.SmpNum, saa.sty);
		perturb();
	}
	endt = clock();
	ofstream myfile8(file + "SimuT.txt", ios::app);
	myfile8 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	myfile8.close();
	std::cout << 8 << endl;

	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.SBD(0);
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile2(file + "Fst.txt", ios::app);
	//myfile2 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile2.close();
	//std::cout << 3 << endl;
	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.SBD(1);
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile3(file + "Bst.txt", ios::app);
	//myfile3 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile3.close();
	//std::cout << 4 << endl;

	//srand(iter);
	//st = clock();
	//loadIndividual(indiv);
	//for (int i = 0; i < 1000; i++) {
	//	exportSAAData();//initial SAA data,i.e.,stx sty, saa.sequencex, the sequenceX and sequenceY don't take part in loadSAA() provedure
	//	saa.SBD_Fast_Update();
	//	perturb();
	//}
	//endt = clock();
	//ofstream myfile5(file + "BstF.txt", ios::app);
	//myfile5 << std::fixed << std::setprecision(1) << (double)(endt - st) / (double)CLOCKS_PER_SEC << endl;
	//myfile5.close();
	//std::cout << 5 << endl;

}
void LocalSearch::updatechange() {
	int id;
	bestchangedIndex[0] = changedIndex[0];
	for (int i = 1; i <= changedIndex[0]; i++) {
		id = changedIndex[i];
		bestchangedIndex[i] = id;
		bestchangedStime[id] = changedStime[id];
		bestchangedTrans[id] = changedTrans[id];
		bestchangedWTcost[id] = changedWTcost[id];
	}
	bestchangedNbTrucks[routeU->cour] = changedNbTrucks[routeU->cour];
	bestchangedNbTrucks[routeV->cour] = changedNbTrucks[routeV->cour];
}
void LocalSearch::updateTrucksNewO() {
	for (int i = 1; i <= bestchangedIndex[0]; i++) {
		trucksO[bestchangedIndex[i]].cumulatedStart = bestchangedStime[bestchangedIndex[i]];
		if (trucksO[bestchangedIndex[i]].cumulatedStart <= params.ITK[bestchangedIndex[i]].LS) {
			trucksO[bestchangedIndex[i]].isfeasible = true;
		}
		else { trucksO[bestchangedIndex[i]].isfeasible = false; }
		trucksO[bestchangedIndex[i]].cumulatedEnd = bestchangedStime[bestchangedIndex[i]] + params.ITK[bestchangedIndex[i]].GT;
		trucksO[bestchangedIndex[i]].WTimeCost = bestchangedWTcost[bestchangedIndex[i]];
		trucksO[bestchangedIndex[i]].TransCost = bestchangedTrans[bestchangedIndex[i]];
	}
}
void LocalSearch::updateTrucksNewD() {
	for (int i = 1; i <= bestchangedIndex[0]; i++) {
		trucksD[bestchangedIndex[i]].cumulatedStart = bestchangedStime[bestchangedIndex[i]];
		if (trucksD[bestchangedIndex[i]].cumulatedStart >= params.OTK[bestchangedIndex[i]].AT) {
			trucksD[bestchangedIndex[i]].isfeasible = true;
		}
		else { trucksD[bestchangedIndex[i]].isfeasible = false; }
		trucksD[bestchangedIndex[i]].cumulatedEnd = bestchangedStime[bestchangedIndex[i]] + params.OTK[bestchangedIndex[i]].GT;
		trucksD[bestchangedIndex[i]].WTimeCost = bestchangedWTcost[bestchangedIndex[i]];
		trucksD[bestchangedIndex[i]].TransCost = bestchangedTrans[bestchangedIndex[i]];
	}
}


LocalSearch::LocalSearch(Params & params, SAA & saa) : params(params), saa(saa)
{
	historicalBest = 99999999;
	LSTIME = 0;
	SAATIME = 0;
	SimulationTime = 0;
	VNDTIME = 0;
	trucksO = std::vector < Node >(params.nbInTrucks);
	trucksD = std::vector < Node >(params.nbOutTrucks);
	doorsO = std::vector < Route >(params.nbInDoors);
	doorsD = std::vector < Route >(params.nbOutDoors);
	headsO = std::vector < Node >(params.nbInDoors);
	headsD = std::vector < Node >(params.nbOutDoors);
	for (int i = 0; i < params.nbInTrucks; i++)
	{
		trucksO[i].cour = i;
		trucksO[i].ishead = false;
	}
	for (int i = 0; i < params.nbOutTrucks; i++)
	{
		trucksD[i].cour = i;
		trucksD[i].ishead = false;
	}

	for (int i = 0; i < params.nbInDoors; i++)
	{
		doorsO[i].cour = i;
		doorsO[i].head = &headsO[i];
		headsO[i].cour = -1;
		headsO[i].position = -1;
		headsO[i].ishead = true;
		headsO[i].route = &doorsO[i];
	}
	for (int i = 0; i < params.nbOutDoors; i++)
	{
		doorsD[i].cour = i;
		doorsD[i].head = &headsD[i];
		headsD[i].cour = -1;
		headsD[i].position = -1;
		headsD[i].ishead = true;
		headsD[i].route = &doorsD[i];
	}
	for (int i = 0; i < params.nbInTrucks; i++) orderTrucksO.push_back(i);
	for (int i = 0; i < params.nbOutTrucks; i++) orderTrucksD.push_back(i);
	for (int r = 0; r < params.nbInDoors; r++) orderDoorsO.push_back(r);
	for (int r = 0; r < params.nbOutDoors; r++) orderDoorsD.push_back(r);
}


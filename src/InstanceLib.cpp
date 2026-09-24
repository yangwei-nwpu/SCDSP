
#include <fstream>
#include <cmath>
#include "InstanceLib.h"
#include<iostream>
InstanceLIB::InstanceLIB(std::string pathToInstance, bool isRoundingInteger = true)
{
	std::string content, content2, content3;
	double serviceTimeData = 0.;
	MAXDLINE = 0;
	// Read INPUT dataset
	std::ifstream inputFile(pathToInstance);
	if (inputFile.is_open())
	{
		inputFile >> NumTruck_O >> NumTruck_D >> NumDoor_O >> NumDoor_D;
		inputFile >> velocity;
		AT_O = new int[NumTruck_O];
		ES_O = new int[NumTruck_O];
		EF_O = new int[NumTruck_O];
		LS_O = new int[NumTruck_O];
		LF_O = new int[NumTruck_O];
		GT_O = new int[NumTruck_O];
		AT_D = new int[NumTruck_D];
		ES_D = new int[NumTruck_D];
		EF_D = new int[NumTruck_D];
		LS_D = new int[NumTruck_D];
		LF_D = new int[NumTruck_D];
		GT_D = new int[NumTruck_D];
		Dis = new int*[NumDoor_O];
		Pre = new int*[NumTruck_D];
		for (int i = 0; i < NumDoor_D; i++) { Dis[i] = new int[NumDoor_D];}
		T = new int*[NumTruck_O];
		Flow = new int*[NumTruck_O];
		PP = new int*[NumTruck_O];
		Suc = new int*[NumTruck_O];
		for (int i = 0; i < NumTruck_O; i++) { T[i] = new int[NumTruck_D]; Flow[i] = new int[NumTruck_D]; PP[i] = new int[NumTruck_D]; Suc[i] = new int[NumTruck_D+1];}
		for (int i = 0; i < NumDoor_O; i++) {
			for (int j = 0; j < NumDoor_D; j++) {
				inputFile >> Dis[i][j];
			}
		}
		for (int i = 0; i < NumTruck_O; i++) {
			inputFile >> AT_O[i] >> EF_O[i] >> LF_O[i] >> GT_O[i];
			ES_O[i] = EF_O[i] - GT_O[i];
			LS_O[i] = LF_O[i] - GT_O[i];
			if (MAXDLINE < LF_O[i]) {
				MAXDLINE = LF_O[i];
			}
		}
		for (int i = 0; i < NumTruck_D; i++) {
			inputFile >> AT_D[i] >> EF_D[i] >> LF_D[i] >> GT_D[i];
			ES_D[i] = EF_D[i] - GT_D[i];
			LS_D[i] = LF_D[i] - GT_D[i];
			if (MAXDLINE < LF_D[i]) {
				MAXDLINE = LF_D[i];
			}
		}
		int cnts = 0, *cntp;
		cntp = new int[NumTruck_D];
		for (int i = 0; i < NumTruck_D; i++) { cntp[i] = 0;  Pre[i] = new int[NumTruck_O+1];}
		for (int i = 0; i < NumTruck_O; i++) {
			cnts = 0;
			for (int j = 0; j < NumTruck_D; j++) {
				inputFile >> Flow[i][j];
				T[i][j] = 0;
				if (Flow[i][j] != 0) {
					cnts++;
					cntp[j]++;
					Suc[i][cnts] = j;
					Pre[j][cntp[j]] = i;
					T[i][j] = Flow[i][j] / velocity + 1;
				}
			}
			Suc[i][0] = cnts;
		}
		for(int i=0;i<NumTruck_D;i++){ Pre[i][0] = cntp[i]; }
		for (int i = 0; i < NumTruck_O; i++) {
			for (int j = 0; j < NumTruck_D; j++)
			{
				inputFile >> PP[i][j];
			}
		}
		inputFile.close();
	}
	else
		throw std::string("Impossible to open instance file: " + pathToInstance);
}

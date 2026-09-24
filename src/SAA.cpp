#include "SAA.h"
#include <cstring>
#include<vector>
#include <queue>
//only generate once 
using namespace std;
std::piecewise_linear_distribution<double> triangular_distribution(double min, double peak, double max)
{
	std::array<double, 3> II{ min, peak, max };
	std::array<double, 3> w{ 0, 1, 0 };
	return std::piecewise_linear_distribution<double>{II.begin(), II.end(), w.begin()};
}
double sampleTriangular(double a, double b, double c, std::mt19937 &gen) {
	std::uniform_real_distribution<> dist(0.0, 1.0);
	double F = (c - a) / (b - a); // 左侧的累计分布
	double u = dist(gen);

	if (u < F) {
		return a + std::sqrt(u * (b - a) * (c - a));
	}
	else {
		return b - std::sqrt((1 - u) * (b - a) * (b - c));
	}
}

int sampleTriangularConditional(double a, double b, double c, double U, std::mt19937 &gen) {
	if (U >= b) {
		return std::lround(b); // 如果输入 U 已经达到 b，则返回 b，因为该事件概率为 100%
	}
	double sample;
	do {
		sample = sampleTriangular(a, b, c, gen);
	} while (sample <= U - 0.5 - 1e-6); // 保证采样值在 U-0.5 之后,比如U=4,那么sample>3.5 即可。 
	return std::lround(sample); // 将连续值四舍五入为整数 满足整数sample>=U
}
void SAA::generate_sample2(int seed, int mode) {
	if (mode == 0) {
		cout << "triangular distribution" << endl;
		double a = -6, b = 6, c = 0; // 三角分布的参数
		std::random_device rd;
		// create a mersenne twister PRNG seeded from some implementation-defined random source
		std::mt19937 gen(seed);//seed=rd
		for (int i = 0; i < SmpNum; i++) {
			for (int j = 0; j < params.nbInTrucks; j++) {
				sampleX[j][i] = std::lround(sampleTriangular(a, b, c, gen)) + params.ITK[j].AT;
			}
			for (int j = 0; j < params.nbOutTrucks; j++) {
				sampleY[j][i] = std::lround(sampleTriangular(a, b, c, gen)) + params.OTK[j].AT;
			}
		}
	}
	else {
		cout << "unidorm distribution" << endl;
		std::default_random_engine random(seed);
		std::uniform_int_distribution<int> SmpGenerator(-4, 4);
		for (int i = 0; i < SmpNum; i++) {
			for (int j = 0; j < params.nbInTrucks; j++) {
				sampleX[j][i] = std::lround(SmpGenerator(random)) + params.ITK[j].AT;
			}
			for (int j = 0; j < params.nbOutTrucks; j++) {
				sampleY[j][i] = std::lround(SmpGenerator(random)) + params.OTK[j].AT;
			}
		}
	}
}
void SAA::generate_sample(int seed) {
	std::default_random_engine random(seed);
	std::uniform_int_distribution<int> SmpGenerator(-4, 4);
	for (int i = 0; i < SmpNum; i++) {
		for (int j = 0; j < params.nbInTrucks; j++) {
			sampleX[j][i] = std::lround(SmpGenerator(random)) + params.ITK[j].AT;
		}
		for (int j = 0; j < params.nbOutTrucks; j++) {
			sampleY[j][i] = std::lround(SmpGenerator(random)) + params.OTK[j].AT;
		}
	}
}
void SAA::generate_EST() {
	int tmp = 0;
	int preT = 0;
	for (int w = 0; w < SmpNum; w++) {
		for (int k = 0; k < params.nbInDoors; k++) {
			for (int i = 1; i <= SequenceX[k][0]; i++) {
				tmp = SequenceX[k][i];//inbound truck id
				if (i == 1) {
					ESTX[tmp][w] = max(sampleX[tmp][w], 0);
				}
				else {
					preT = SequenceX[k][i - 1];
					ESTX[tmp][w] = max(sampleX[tmp][w], min(ESTX[preT][w] + params.ITK[preT].GT, params.ITK[preT].LF));
				}
			}
		}
		for (int l = 0; l < params.nbOutDoors; l++) {
			for (int j = 1; j <= SequenceY[l][0]; j++) {
				tmp = SequenceY[l][j];//outbound truck id
				if (j == 1) {
					ESTY[tmp][w] = sampleY[tmp][w];
					SenarioY[tmp][w].id = w;
					SenarioY[tmp][w].EST = ESTY[tmp][w];
				}
				else {
					preT = SequenceY[l][j - 1];
					ESTY[tmp][w] = max(sampleY[tmp][w], min(ESTY[preT][w] + params.OTK[preT].GT, params.OTK[preT].LF));
					SenarioY[tmp][w].id = w;
					SenarioY[tmp][w].EST = ESTY[tmp][w];
				}
			}
		}
	}
}
void SAA::generate_ESTP(int _SmpNum) {
	int tmp = 0;
	int preT = 0;
	for (int w = 0; w < _SmpNum; w++) {
		for (int k = 0; k < params.nbInDoors; k++) {
			for (int i = 1; i <= SequenceX[k][0]; i++) {
				tmp = SequenceX[k][i];//inbound truck id
				if (i == 1) {
					ESTX[tmp][w] = max(sampleX[tmp][w], 0);
				}
				else {
					preT = SequenceX[k][i - 1];
					ESTX[tmp][w] = max(sampleX[tmp][w], min(ESTX[preT][w] + params.ITK[preT].GT, params.ITK[preT].LF));
				}
			}
		}
		for (int l = 0; l < params.nbOutDoors; l++) {
			for (int j = 1; j <= SequenceY[l][0]; j++) {
				tmp = SequenceY[l][j];//outbound truck id
				if (j == 1) {
					ESTY[tmp][w] = sampleY[tmp][w];
					SenarioY[tmp][w].id = w;
					SenarioY[tmp][w].EST = ESTY[tmp][w];
				}
				else {
					preT = SequenceY[l][j - 1];
					ESTY[tmp][w] = max(sampleY[tmp][w], min(ESTY[preT][w] + params.OTK[preT].GT, params.OTK[preT].LF));
					SenarioY[tmp][w].id = w;
					SenarioY[tmp][w].EST = ESTY[tmp][w];
				}
			}
		}
	}
}
void SAA::sortSenario() {
	for (int i = 0; i < params.nbInTrucks; i++) {
		for (int j = 0; j <= params.MAXDLINE; j++) {
			H[i][j] = -1;
		}
	}
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			sort(SenarioY[i], SenarioY[i] + SmpNum);
			for (int j = 0; j < SmpNum; j++) {
				H[i][SenarioY[i][j].EST] = j;
			}
		}
	}
	int tmp = 0;
	int tmpmin = 0;
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			tmp = SenarioY[i][0].EST;//最大开始时间
			tmpmin = SenarioY[i][SmpNum - 1].EST;
			for (int j = 0; j < tmpmin; j++) {
				H[i][j] = SmpNum - 1;
			}
			for (int j = tmpmin + 1; j <= tmp; j++) {
				if (H[i][j] == -1) {
					H[i][j] = H[i][j - 1];
				}
			}
			for (int j = tmp + 1; j <= params.MAXDLINE; j++) {
				H[i][j] = -1;
			}
		}
	}
}
void SAA::sortSenarioP(int _SmpNum) {
	for (int i = 0; i < params.nbInTrucks; i++) {
		for (int j = 0; j <= params.MAXDLINE; j++) {
			H[i][j] = -1;
		}
	}
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			sort(SenarioY[i], SenarioY[i] + _SmpNum);
			for (int j = 0; j < _SmpNum; j++) {
				H[i][SenarioY[i][j].EST] = j;
			}
		}
	}
	int tmp = 0;
	int tmpmin = 0;
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			tmp = SenarioY[i][0].EST;//最大开始时间
			tmpmin = SenarioY[i][_SmpNum - 1].EST;
			for (int j = 0; j < tmpmin; j++) {
				H[i][j] = _SmpNum - 1;
			}
			for (int j = tmpmin + 1; j <= tmp; j++) {
				if (H[i][j] == -1) {
					H[i][j] = H[i][j - 1];
				}
			}
			for (int j = tmp + 1; j <= params.MAXDLINE; j++) {
				H[i][j] = -1;
			}
		}
	}
}
int SAA::fast_compute_delta(int kth, int buf) {
	int tmpst = 0;
	int index = 0;
	int delta_cost = 0;
	int delta_trans = 0;
	int sid = -1;//senario id
	int preid = -1;//truck id
	int transtime = 0;
	int postid = -1;
	int door = soly[kth];//the door which truck kth was assigned
	int tmptime2, tmptime1;
	NumAffectedScenarios = 0;
	if (buf > 0) {
		tmpst = sty[kth] + buf;
		index = H[kth][tmpst];//the start time tmpst's index in H
		//************   senarios should be evaluated  are {H[kth][tmpst]+1,H[kth][tmpst]+2,...,SmpNum}   ***************//
		delta_cost += (SmpNum - index - 1)*(params.funOTD[kth][tmpst] - params.funOTD[kth][sty[kth]]);//delta_OverTime
		NumAffectedScenarios += SmpNum - index - 1;
		for (int j = 1; j <= params.OTK[kth].Pre[0]; j++) {
			preid = params.OTK[kth].Pre[j];// 出站卡车kth的入站卡车id
			if (solx[preid] >= 0) {
				transtime = params.Time[preid][kth] * params.Dis[solx[preid]][door];
				tmptime1 = sty[kth] - transtime;
				tmptime2 = tmptime1 + buf;
				delta_trans = 0;
				if (tmptime1 >= 0) {
					if (tmpst <= params.OTK[kth].LS) {
						for (int i = index + 1; i < SmpNum; i++) {
							sid = SenarioY[kth][i].id;//根据场景id确定入站卡车的开始时间
							delta_trans += (params.funTrans[preid][kth][ESTX[preid][sid]][tmptime2] - params.funTrans[preid][kth][ESTX[preid][sid]][tmptime1]);//delta_Transcost}
						}
					}
					else if (sty[kth] > params.OTK[kth].LS) {
						delta_trans += 0;
					}
					else {//tmpst>dd[kth+M]-g[kth + M] && sty[kth]<=dd[kth+M]-g[kth + M]
						for (int i = index + 1; i < SmpNum; i++) {
							sid = SenarioY[kth][i].id;//根据场景id确定入站卡车的开始时间
							delta_trans += -params.funTrans[preid][kth][ESTX[preid][sid]][tmptime1];//delta_Transcost}
						}
					}
				}
				delta_cost += -delta_trans * params.FP[preid][kth];
			}
		}

		tmpst += params.OTK[kth].GT;
		for (int l = L[kth] + 1; l <= SequenceY[door][0]; l++) {//2 5 7 8, if kth=2,then L[th]=1, 5，7，8 all need to be evaluated
			postid = SequenceY[door][l];
			if ((tmpst > sty[postid]) && (tmpst <= params.OTK[SequenceY[door][l - 1]].LF)) {//the truck postid was affected
				index = H[postid][tmpst];
				delta_cost += (SmpNum - index - 1)*(params.funOTD[postid][tmpst] - params.funOTD[postid][sty[postid]]);
				NumAffectedScenarios += SmpNum - index - 1;
				for (int j = 1; j <= params.OTK[postid].Pre[0]; j++) {
					preid = params.OTK[postid].Pre[j];// 出站卡车postid的入站卡车id
					if (solx[preid] >= 0) {
						transtime = params.Time[preid][postid] * params.Dis[solx[preid]][door];
						tmptime1 = sty[postid] - transtime;
						tmptime2 = tmptime1 + buf;
						delta_trans = 0;
						if (tmptime1 >= 0) {
							if (tmpst <= params.OTK[postid].LS) {
								for (int i = index + 1; i < SmpNum; i++) {
									sid = SenarioY[postid][i].id;//根据场景id确定入站卡车的开始时间
									delta_trans += (params.funTrans[preid][postid][ESTX[preid][sid]][tmptime2] - params.funTrans[preid][postid][ESTX[preid][sid]][tmptime1]);//delta_Transcost}
								}

							}
							else if (sty[postid] > params.OTK[postid].LS) {
								delta_trans += 0;
							}
							else {//tmpst>dd[postid+M] && sty[kth]<=dd[postid+M]
								for (int i = index + 1; i < SmpNum; i++) {
									sid = SenarioY[postid][i].id;//根据场景id确定入站卡车的开始时间
									delta_trans += -params.funTrans[preid][postid][ESTX[preid][sid]][tmptime1];//delta_Transcost

								}
							}
						}
						delta_cost += -delta_trans * params.FP[preid][postid];
					}
				}
				tmpst += params.OTK[postid].GT;
			}
			else break;
		}
	}
	else {
		tmpst = sty[kth] + buf;
		int endtime = 0;
		if (L[kth] > 1) {
			if (sty[SequenceY[door][L[kth] - 1]] > params.OTK[SequenceY[door][L[kth] - 1]].LS) { endtime = params.OTK[SequenceY[door][L[kth] - 1]].LF; }
			else { endtime = sty[SequenceY[door][L[kth] - 1]] + params.OTK[SequenceY[door][L[kth] - 1]].GT; }
		}
		if (L[kth] == 1 || tmpst >= endtime) {
			index = H[kth][sty[kth]];//the start time tmpst's index in H

			//************   senarios should be evaluated  are {H[kth][sty[kth]]+1,H[kth][sty[kth]]+2,...,SmpNum}   ***************//
			delta_cost += (SmpNum - index - 1)*(params.funOTD[kth][tmpst] - params.funOTD[kth][sty[kth]]);//delta_OverTime
			NumAffectedScenarios += SmpNum - index - 1;
			for (int j = 1; j <= params.OTK[kth].Pre[0]; j++) {
				preid = params.OTK[kth].Pre[j];// 出站卡车kth的入站卡车id
				if (solx[preid] >= 0) {
					transtime = params.Time[preid][kth] * params.Dis[solx[preid]][door];
					tmptime1 = sty[kth] - transtime;
					tmptime2 = tmptime1 + buf;
					delta_trans = 0;
					if (tmptime2 >= 0) {
						if (sty[kth] <= params.OTK[kth].LS) {
							for (int i = index + 1; i < SmpNum; i++) {
								sid = SenarioY[kth][i].id;//根据场景id确定入站卡车的开始时间
								delta_trans += (params.funTrans[preid][kth][ESTX[preid][sid]][tmptime2] - params.funTrans[preid][kth][ESTX[preid][sid]][tmptime1]);//delta_Transcost}
							}
						}
						else if (tmpst > params.OTK[kth].LS) {
							delta_trans += 0;
						}
						else {//tmpst<=dd[kth+M] && sty[kth]> dd[kth + M]
							for (int i = index + 1; i < SmpNum; i++) {
								sid = SenarioY[kth][i].id;//根据场景id确定入站卡车的开始时间
								delta_trans += params.funTrans[preid][kth][ESTX[preid][sid]][tmptime2];//delta_Transcost}
							}
						}
					}
					delta_cost += -delta_trans * params.FP[preid][kth];
				}
			}
		}
	}
	RateAverageAffectedScenarios += (double)NumAffectedScenarios / SmpNum;//params.nbOutTrucks*
	TotalUpdate++;
	return delta_cost;
}
void SAA::initial_DeltaVector() {
	int total;
	int  *tmpsty;
	tmpsty = new int[params.nbOutTrucks];
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			delta_vector[0][i] = fast_compute_delta(i, 1);
			delta_vector[1][i] = fast_compute_delta(i, -1);
		}
	}
}
void SAA::initial_DeltaVector_No_FastEval(long long int _local) {
	long long int total;
	int  *tmpsty;
	tmpsty = new int[params.nbOutTrucks];
	for (int i = 0; i < params.nbOutTrucks; i++) {
		if (soly[i] >= 0) {
			sty[i] += 1;
			for (int l = 0; l < params.nbOutTrucks; l++) {

				tmpsty[l] = sty[l];
			}
			SGS2(total, tmpsty);
			delta_vector[0][i] = total - _local;
			sty[i] += -2;
			for (int l = 0; l < params.nbOutTrucks; l++) {
				tmpsty[l] = sty[l];
			}
			SGS2(total, tmpsty);
			delta_vector[1][i] = total - _local;
			sty[i] += 1;

		}
	}
	delete[] tmpsty;
}
void SAA::move_and_fast_update(int kth, int buf) {
	int door = soly[kth];
	int tmpst = 0;
	int endindex;
	bool isfind = true;
	int *tmpstx, *tmpsty;
	int total;
	tmpsty = new int[params.nbOutTrucks];

	sty[kth] += buf;
	endindex = L[kth];

	if (buf > 0) {//update solution sty
		tmpst = sty[kth] - buf;
		for (int i = L[kth] - 1; i >= 1; i--) {
			if (tmpst == sty[SequenceY[door][i]] + params.OTK[SequenceY[door][i]].GT) {//tmpst >= sty[SequenceY[door][i]] + g[SequenceY[door][i] + M]
				delta_vector[0][SequenceY[door][i]] += -delta_vector[0][SequenceY[door][L[kth]]];
				tmpst = sty[SequenceY[door][i]];
			}
			else {
				break;
			}

		}
		isfind = true;
		for (int l = L[kth] + 1; l <= SequenceY[door][0]; l++) {
			tmpst = sty[SequenceY[door][l - 1]] + params.OTK[SequenceY[door][l - 1]].GT;
			if (isfind && tmpst > sty[SequenceY[door][l]] && tmpst <= params.OTK[SequenceY[door][l - 1]].LF) {
				sty[SequenceY[door][l]] = tmpst;
			}
			else { isfind = false; }
			if (tmpst >= sty[SequenceY[door][l - 1]]) {
				endindex = l;
			}
			else break;
		}
		for (int l = L[kth]; l <= endindex; l++) {
			delta_vector[0][SequenceY[door][l]] = fast_compute_delta(SequenceY[door][l], 1);
			delta_vector[1][SequenceY[door][l]] = fast_compute_delta(SequenceY[door][l], -1);
		}
	}
	else {
		delta_vector[0][kth] = fast_compute_delta(kth, 1);
		delta_vector[1][kth] = fast_compute_delta(kth, -1);
	}
	delete[] tmpsty;
}
void SAA::move_and_fast_update2(int kth, int buf) {
	int door = soly[kth];
	int tmpst = 0;
	int endindex;
	bool isfind = true;
	int *tmpstx, *tmpsty;
	int total;
	tmpsty = new int[params.nbOutTrucks];

	sty[kth] += buf;
	endindex = L[kth];

	if (buf > 0) {//update solution sty
		tmpst = sty[kth] - buf;
		for (int i = L[kth] - 1; i >= 1; i--) {
			if (tmpst == sty[SequenceY[door][i]] + params.OTK[SequenceY[door][i]].GT) {//tmpst >= sty[SequenceY[door][i]] + g[SequenceY[door][i] + M]
				delta_vector[0][SequenceY[door][i]] += -delta_vector[0][SequenceY[door][L[kth]]];
				tmpst = sty[SequenceY[door][i]];
			}
			else {
				break;
			}

		}
		isfind = true;
		for (int l = L[kth] + 1; l <= SequenceY[door][0]; l++) {
			tmpst = sty[SequenceY[door][l - 1]] + params.OTK[SequenceY[door][l - 1]].GT;
			if (isfind && tmpst > sty[SequenceY[door][l]] && tmpst <= params.OTK[SequenceY[door][l - 1]].LF) {
				sty[SequenceY[door][l]] = tmpst;
			}
			else { isfind = false; }
			if (tmpst >= sty[SequenceY[door][l - 1]]) {
				endindex = l;
			}
			else break;
		}
		for (int l = L[kth]; l <= endindex; l++) {
			delta_vector[0][SequenceY[door][l]] = fast_compute_delta(SequenceY[door][l], 1);
			delta_vector[1][SequenceY[door][l]] = fast_compute_delta(SequenceY[door][l], -1);
		}

	}
	else {
		delta_vector[0][kth] = -delta_vector[1][kth];
		tmpst = sty[kth] - buf;//original start time
		for (int l = L[kth] + 1; l <= SequenceY[door][0]; l++) {
			if (tmpst + params.OTK[SequenceY[door][l - 1]].GT == sty[SequenceY[door][l]] && tmpst <= params.OTK[SequenceY[door][l - 1]].LF) {
				endindex = l;
				tmpst = sty[SequenceY[door][l]];
			}
			else break;
		}
		for (int l = L[kth]; l <= endindex; l++) {
			delta_vector[1][SequenceY[door][l]] = fast_compute_delta(SequenceY[door][l], -1);
		}
	}

	delete[] tmpsty;
}
void SAA::move_and_update(int kth, int buf, long long int _local) {
	int door = soly[kth];
	int tmpst = 0;
	int endindex;
	bool isfind = true;
	int *tmpstx, *tmpsty;
	long long int total;
	tmpsty = new int[params.nbOutTrucks];
	sty[kth] += buf;
	endindex = L[kth];
	if (buf > 0) {//update solution sty
		tmpst = sty[kth] - buf;
		for (int i = L[kth] - 1; i >= 1; i--) {
			if (tmpst == sty[SequenceY[door][i]] + params.OTK[SequenceY[door][i]].GT) {//tmpst >= sty[SequenceY[door][i]] + g[SequenceY[door][i] + M]
				delta_vector[0][SequenceY[door][i]] += -delta_vector[0][SequenceY[door][L[kth]]];
				tmpst = sty[SequenceY[door][i]];
			}
			else {
				break;
			}
		}
		isfind = true;
		for (int l = L[kth] + 1; l <= SequenceY[door][0]; l++) {
			tmpst = sty[SequenceY[door][l - 1]] + params.OTK[SequenceY[door][l - 1]].GT;
			if (isfind && tmpst > sty[SequenceY[door][l]] && tmpst <= params.OTK[SequenceY[door][l - 1]].LF) {
				sty[SequenceY[door][l]] = tmpst;
			}
			else { isfind = false; }
			if (tmpst >= sty[SequenceY[door][l - 1]]) {
				endindex = l;
			}
			else break;
		}
		for (int l = L[kth]; l <= endindex; l++) {
			sty[SequenceY[door][l]] += 1;
			for (int i = 0; i < params.nbOutTrucks; i++) {
				tmpsty[i] = sty[i];
			}
			SGS2(total, tmpsty);
			delta_vector[0][SequenceY[door][l]] = total - _local;
			sty[SequenceY[door][l]] += -2;
			for (int i = 0; i < params.nbOutTrucks; i++) {
				tmpsty[i] = sty[i];
			}
			SGS2(total, tmpsty);
			delta_vector[1][SequenceY[door][l]] = total - _local;
			sty[SequenceY[door][l]] += 1;
		}
	}
	else {
		sty[kth] += 1;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			tmpsty[i] = sty[i];
		}
		SGS2(total, tmpsty);
		delta_vector[0][kth] = total - _local;
		sty[kth] += -2;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			tmpsty[i] = sty[i];
		}
		SGS2(total, tmpsty);
		delta_vector[1][kth] = total - _local;
		sty[kth] += 1;
	}
	delete[] tmpsty;
}
//sequenceX, Y, solx, soly, stx ans sty should be initialied in advance.
void SAA::SAA_LS() {
	//int local_best;
	long long int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	long long int _local_best = 0;//not averaged yet
	double STARTTIME = clock();
	generate_EST();
	sortSenario();
	SGS2(_local_best, sty);//initial local best
	double ENDTIME = clock();
	SimTIME = (double)(ENDTIME - STARTTIME) / CLOCKS_PER_SEC;
	ini_sim_local = (double)(_local_best*1.0 / SmpNum) + params.TotalPenality;
	initial_DeltaVector();
	while (1) {
		select_j = -1;
		min_delta = 0;
		ff = -1;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			if (soly[i] >= 0) {
				if (min_delta > delta_vector[0][i]) {
					min_delta = delta_vector[0][i];
					select_j = i;
					buf = 1;
					ff = 0;
				}
				if (min_delta > delta_vector[1][i]) {
					min_delta = delta_vector[1][i];
					select_j = i;
					buf = -1;
					ff = 1;
				}
			}
		}
		if (select_j != -1) {
			_local_best += delta_vector[ff][select_j];
			if (sty[select_j] + buf > params.OTK[select_j].LS || sty[select_j] + buf < params.OTK[select_j].AT)
			{
				cout << "delta: " << delta_vector[ff][select_j] << " sty: " << sty[select_j] << " sty+buf: " << sty[select_j] + buf << " LS: " << params.OTK[select_j].LS << " AT:" << params.OTK[select_j].AT << endl;
				system("pause");
			}
			move_and_fast_update2(select_j, buf);
		}
		else {
			break;
		}
	}
	local_best = (double)(_local_best*1.0 / SmpNum);
	local_best += params.TotalPenality;
}
void SAA::SAA_LS_SmpNumTest() {
	//int local_best;
	long long int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	long long int _local_best = 0;//not averaged yet
	generate_ESTP(TestSmpNum);
	sortSenario();
	SGS2(_local_best, sty);//initial local best
	ini_sim_local = (double)(_local_best*1.0 / SmpNum) + params.TotalPenality;
	initial_DeltaVector();
	int *stytmp = new int[params.nbOutTrucks];
	double TrueValueofCurrent, TrueValueofBest;
	long long TrueBest,TrueCur;
	while (1) {
		select_j = -1;
		min_delta = 0;
		ff = -1;
		/***** found true best*****/
		SGS2Small(TestSmpNum, sty);
		TrueValueofBest = local_best;
		for (int ii = 0; ii < params.nbOutTrucks; ii++) {
			if (soly[ii] >= 0) {
				for (int k = 0; k < params.nbOutTrucks; k++) {
					stytmp[k] = sty[k];
				}
				stytmp[ii] += 1;
				SGS2Small(TestSmpNum, stytmp);
				if (TrueValueofBest-local_best>1e-5) {
					TrueValueofBest = local_best;
				}
				for (int k = 0; k < params.nbOutTrucks; k++) {
					stytmp[k] = sty[k];
				}
				stytmp[ii] += -1;
				SGS2Small(TestSmpNum, stytmp);
				if (TrueValueofBest - local_best > 1e-5) {
					TrueValueofBest = local_best;
				}
			}
		}

		for (int i = 0; i < params.nbOutTrucks; i++) {
			if (soly[i] >= 0) {
				if (min_delta > delta_vector[0][i]) {
					min_delta = delta_vector[0][i];
					select_j = i;
					buf = 1;
					ff = 0;
				}
				if (min_delta > delta_vector[1][i]) {
					min_delta = delta_vector[1][i];
					select_j = i;
					buf = -1;
					ff = 1;
				}
			}
		}
		if (select_j != -1) {
			_local_best += delta_vector[ff][select_j];
			move_and_fast_update2(select_j, buf);
			SGS2P(TrueBest, sty, TestSmpNum);
			TrueValueofCurrent = (double)(TrueBest*1.0 / TestSmpNum) + params.TotalPenality;
			double Regret = TrueValueofCurrent - TrueValueofBest;
			if (Regret > 1e-5) {
				TotalFalseIter++;
				TotalRegret += Regret;
				AveRateRegret += Regret / (TrueValueofBest+ 1e-5);
				DetailRegret.push_back(Regret);
				DetailAveRate.push_back(Regret / (TrueValueofBest + 1e-5));
			}
			TotalIter++;
		}
		else {
			break;
		}
	}
	local_best = (double)(_local_best*1.0 / SmpNum);
	local_best += params.TotalPenality;
	
}
void SAA::SAA_LSF() {
	int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	bool isfound = true;
	long long  _local_best = 0;//not averaged yet
	generate_EST();
	sortSenario();
	SGS2(_local_best, sty);//initial local best
	ini_sim_local = (double)(_local_best*1.0 / SmpNum) + params.TotalPenality;
	initial_DeltaVector();
	while (isfound) {
		select_j = -1;
		min_delta = 0;
		ff = -1;
		isfound = false;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			if (soly[i] >= 0) {
				if (delta_vector[0][i] < 0) {
					select_j = i;
					buf = 1;
					ff = 0;
					isfound = true;
					_local_best += delta_vector[ff][select_j];
					move_and_fast_update2(select_j, buf);
					break;
				}
				else if (delta_vector[1][i] < 0) {
					select_j = i;
					buf = -1;
					ff = 1;
					isfound = true;
					_local_best += delta_vector[ff][select_j];
					move_and_fast_update2(select_j, buf);
					break;
				}
			}
		}
	}
	local_best = (double)(_local_best*1.0 / SmpNum);
	local_best += params.TotalPenality;
}
void SAA::SBD(bool bestor)
{
	long long int tempcost;
	long long int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	bool found = true;
	int door;
	int *stytmp = new int[params.nbOutTrucks];
	long long int _local_best = 0;//not averaged yet
	generate_ESTP(SmallSmpNum);
	sortSenarioP(SmallSmpNum);
	SGS2P(_local_best, sty, SmallSmpNum);//initial local best
	ini_sim_local = (double)(_local_best*1.0 / SmallSmpNum) + params.TotalPenality;
	if (bestor) {//first
		found = true;
		while (found) {
			found = false;
			select_j = -1;
			min_delta = 0;
			ff = -1;
			for (int i = 0; i < params.nbOutTrucks; i++) {
				if (soly[i] >= 0) {
					for (int k = 0; k < params.nbOutTrucks; k++) {
						stytmp[k] = sty[k];
					}
					stytmp[i] += 1;
					SGS2P(tempcost, stytmp, SmallSmpNum);
					if (tempcost < _local_best) {
						_local_best = tempcost;
						found = true;

						for (int k = 0; k < params.nbOutTrucks; k++) {
							sty[k] = stytmp[k];
						}
						break;
					}
					for (int k = 0; k < params.nbOutTrucks; k++) {
						stytmp[k] = sty[k];
					}
					stytmp[i] += -1;
					SGS2P(tempcost, stytmp, SmallSmpNum);
					if (tempcost < _local_best) {
						_local_best = tempcost;
						found = true;
						for (int k = 0; k < params.nbOutTrucks; k++) {
							sty[k] = stytmp[k];
						}
						break;
					}
				}
			}
		}
	}
	else {//best
		while (1) {
			select_j = -1;
			min_delta = 0;
			ff = 0;
			for (int i = 0; i < params.nbOutTrucks; i++) {
				if (soly[i] >= 0) {
					for (int k = 0; k < params.nbOutTrucks; k++) {
						stytmp[k] = sty[k];
					}
					stytmp[i] += 1;
					SGS2P(tempcost, stytmp, SmallSmpNum);
					if (min_delta > tempcost - _local_best) {
						min_delta = tempcost - _local_best;
						ff = 1;
						select_j = i;
					}
					for (int k = 0; k < params.nbOutTrucks; k++) {
						stytmp[k] = sty[k];
					}
					stytmp[i] += -1;
					SGS2P(tempcost, stytmp, SmallSmpNum);
					if (min_delta > tempcost - _local_best) {
						min_delta = tempcost - _local_best;
						ff = -1;
						select_j = i;
					}
				}
			}
			if (select_j != -1) {
				sty[select_j] += ff;
				_local_best += min_delta;
				if (ff > 0) {//update solution sty
					door = soly[select_j];
					int tmpst = 0;
					for (int l = L[select_j] + 1; l <= SequenceY[door][0]; l++) {
						tmpst = sty[SequenceY[door][l - 1]] + params.OTK[SequenceY[door][l - 1]].GT;
						if (tmpst > sty[SequenceY[door][l]] && tmpst <= params.OTK[SequenceY[door][l - 1]].LF) {
							sty[SequenceY[door][l]] = tmpst;
						}
						else break;
					}
				}
			}
			else { break; }
		}
	}
	local_best = (double)(_local_best*1.0 / SmallSmpNum);
	local_best += params.TotalPenality;
	delete[] stytmp;
}

void SAA::SBD_Fast_Update()
{
	long long int min_delta = 0;
	int select_j = -1;
	int ff = 0;
	int buf = 0;
	long long int _local_best = 0;//not averaged yet
	generate_EST();
	sortSenario();
	SGS2(_local_best, sty);//initial local best
	ini_sim_local = (double)(_local_best*1.0 / SmpNum) + params.TotalPenality;
	initial_DeltaVector_No_FastEval(_local_best);
	while (1) {
		select_j = -1;
		min_delta = 0;
		ff = -1;
		for (int i = 0; i < params.nbOutTrucks; i++) {
			if (soly[i] >= 0) {
				if (min_delta > delta_vector[0][i]) {
					min_delta = delta_vector[0][i];
					select_j = i;
					buf = 1;
					ff = 0;
				}
				if (min_delta > delta_vector[1][i]) {
					min_delta = delta_vector[1][i];
					select_j = i;
					buf = -1;
					ff = 1;
				}
			}
		}
		if (select_j != -1) {
			_local_best += delta_vector[ff][select_j];
			move_and_update(select_j, buf, _local_best);
		}
		else {
			break;
		}
	}
	local_best = (double)(_local_best*1.0 / SmpNum);
	local_best += params.TotalPenality;
}
void SAA::SGS2(long long int &total_cost, int *sty) {
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	int index = 0;
	int sid = 0;
	long long int tmpcost = 0;
	total_cost = 0;
	for (int i = 0; i < params.nbInTrucks; i++) {
		if (solx[i] >= 0) {
			for (int j = 0; j < SmpNum; j++) {
				total_cost += params.funOTO[i][ESTX[i][j]];
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {

			tmp = SequenceY[l][j];//outbound truck id
			if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
				sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
			}
			else if (j != 1 && (sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT > params.OTK[SequenceY[l][j - 1]].LF)) {
				sty[tmp] = max(sty[tmp], params.OTK[SequenceY[l][j - 1]].LF);
			}
			index = H[tmp][sty[tmp] + 1];
			for (int w = 0; w <= index; w++) {
				total_cost += params.funOTD[tmp][ESTY[tmp][SenarioY[tmp][w].id]];//delta_OverTime
			}
			for (int w = index + 1; w < SmpNum; w++) {
				total_cost += params.funOTD[tmp][sty[tmp]];//delta_OverTime
			}
			if (sty[tmp] + params.OTK[tmp].GT <= params.OTK[tmp].LF) {
				for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
					preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
					if (solx[preid] >= 0) {//被安排
						transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
						delta_trans = 0;

						for (int w = 0; w <= index; w++) {
							sid = SenarioY[tmp][w].id;
							if (ESTY[tmp][sid] + params.OTK[tmp].GT <= params.OTK[tmp].LF &&ESTY[tmp][sid] >= transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][sid]][ESTY[tmp][sid] - transtime];
							}
						}
						for (int w = index + 1; w < SmpNum; w++) {
							sid = SenarioY[tmp][w].id;
							if (sty[tmp] >= transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][sid]][sty[tmp] - transtime];//delta_Transcost}
							}
						}
						total_cost += -delta_trans * params.FP[preid][tmp];
					}
				}
			}
		}
	}
}
void SAA::SGS2P(long long int &total_cost, int *sty, int _SmpNum) {
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	int index = 0;
	int sid = 0;
	long long int tmpcost = 0;
	total_cost = 0;
	for (int i = 0; i < params.nbInTrucks; i++) {
		if (solx[i] >= 0) {
			for (int j = 0; j < _SmpNum; j++) {
				total_cost += params.funOTO[i][ESTX[i][j]];
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {

			tmp = SequenceY[l][j];//outbound truck id
			if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
				sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
			}
			else if (j != 1 && (sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT > params.OTK[SequenceY[l][j - 1]].LF)) {
				sty[tmp] = max(sty[tmp], params.OTK[SequenceY[l][j - 1]].LF);
			}
			index = H[tmp][sty[tmp] + 1];
			for (int w = 0; w <= index; w++) {
				total_cost += params.funOTD[tmp][ESTY[tmp][SenarioY[tmp][w].id]];//delta_OverTime
			}
			for (int w = index + 1; w < _SmpNum; w++) {
				total_cost += params.funOTD[tmp][sty[tmp]];//delta_OverTime
			}
			if (sty[tmp] + params.OTK[tmp].GT <= params.OTK[tmp].LF) {
				for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
					preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
					if (solx[preid] >= 0) {//被安排
						transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
						delta_trans = 0;

						for (int w = 0; w <= index; w++) {
							sid = SenarioY[tmp][w].id;
							if (ESTY[tmp][sid] + params.OTK[tmp].GT <= params.OTK[tmp].LF &&ESTY[tmp][sid] >= transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][sid]][ESTY[tmp][sid] - transtime];
							}
						}
						for (int w = index + 1; w < _SmpNum; w++) {
							sid = SenarioY[tmp][w].id;
							if (sty[tmp] >= transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][sid]][sty[tmp] - transtime];//delta_Transcost}
							}
						}
						total_cost += -delta_trans * params.FP[preid][tmp];
					}
				}
			}
		}
	}
}
void SAA::SGS2_profiles(long long int & _Total_Cost, int * sty, std::string filename)
{
	AvgTotalTardiness = 0;
	AvgTotalPenaltyCost = 0;
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	_Total_Cost = 0;
	int total_cost = 0;
	int total_tardiness = 0;
	int total_penaltycost = 0;
	std::ofstream myfile(filename, std::ios::app);
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//outbound truck id
			if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
				sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
			}
			for (int w = 0; w < SmpNum; w++) {
				realSTY[tmp][w] = max(ESTY[tmp][w], sty[tmp]);
			}
		}
	}
	for (int w = 0; w < SmpNum; w++) {
		total_cost = 0;
		total_tardiness = 0;
		total_penaltycost = 0;
		transtime = 0;
		for (int i = 0; i < params.nbInTrucks; i++) {
			if (solx[i] >= 0 && ESTX[i][w] <= params.ITK[i].LS&&ESTX[i][w] >= params.ITK[i].AT) {
				total_tardiness += params.funOTO[i][ESTX[i][w]];
			}
		}

		for (int l = 0; l < params.nbOutDoors; l++) {
			for (int j = 1; j <= SequenceY[l][0]; j++) {
				tmp = SequenceY[l][j];//outbound truck id
				if (realSTY[tmp][w] >= params.OTK[tmp].AT&&realSTY[tmp][w] <= params.OTK[tmp].LS) {
					total_tardiness += params.funOTD[tmp][realSTY[tmp][w]];//delta_OverTime
					for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
						preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
						if (solx[preid] >= 0) {//被安排
							transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
							delta_trans = 0;
							if (realSTY[tmp][w] + params.OTK[tmp].GT <= params.OTK[tmp].LF&&realSTY[tmp][w] > transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][w]][realSTY[tmp][w] - transtime];//delta_Transcost}
							}
							total_penaltycost += -delta_trans * params.FP[preid][tmp];
						}
					}
				}
			}
		}
		_Total_Cost += total_tardiness + total_penaltycost;
		AvgTotalTardiness += total_tardiness;
		AvgTotalPenaltyCost += total_penaltycost + params.TotalPenality;
		total_cost = total_tardiness + total_penaltycost + params.TotalPenality;//current simulation cost under senario \omega
		myfile << total_cost << endl;
	}
	myfile.close();
}
void SAA::SGS2Small(int _SmpNum, int * sty)
{
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	//compute the realSTY
	long long int _Total_Cost = 0;
	int total_cost = 0;

	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//outbound truck id
			if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
				sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
			}
			for (int w = 0; w < _SmpNum; w++) {
				realSTY[tmp][w] = max(ESTY[tmp][w], sty[tmp]);
			}
		}
	}
	for (int w = 0; w < _SmpNum; w++) {
		total_cost = 0;
		transtime = 0;
		for (int i = 0; i < params.nbInTrucks; i++) {
			if (solx[i] >= 0 && ESTX[i][w] <= params.ITK[i].LS&&ESTX[i][w] >= params.ITK[i].AT) {
				total_cost += params.funOTO[i][ESTX[i][w]];
			}
		}

		for (int l = 0; l < params.nbOutDoors; l++) {
			for (int j = 1; j <= SequenceY[l][0]; j++) {
				tmp = SequenceY[l][j];//outbound truck id
				if (realSTY[tmp][w] >= params.OTK[tmp].AT&&realSTY[tmp][w] <= params.OTK[tmp].LS) {
					total_cost += params.funOTD[tmp][realSTY[tmp][w]];//delta_OverTime
					for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
						preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
						if (solx[preid] >= 0) {//被安排
							transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
							delta_trans = 0;
							if (realSTY[tmp][w] + params.OTK[tmp].GT <= params.OTK[tmp].LF&&realSTY[tmp][w] > transtime) {
								delta_trans += params.funTrans[preid][tmp][ESTX[preid][w]][realSTY[tmp][w] - transtime];//delta_Transcost}
							}
							total_cost += -delta_trans * params.FP[preid][tmp];
						}
					}
				}
			}
		}
		_Total_Cost += total_cost;
		total_cost += params.TotalPenality;//current simulation cost under senario \omega
	}
	local_best = (double)(_Total_Cost*1.0 / _SmpNum) + params.TotalPenality;
}

void SAA::SGS(long long int &total_cost, int *sty) {
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	total_cost = 0;
	for (int i = 0; i < params.nbInTrucks; i++) {
		if (solx[i] >= 0) {
			for (int j = 0; j < SmpNum; j++) {
				total_cost += params.funOTO[i][ESTX[i][j]];
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//inbound truck id
			if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
				sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
			}
			for (int w = 0; w < SmpNum; w++) {
				realSTY[tmp][w] = max(ESTY[tmp][w], sty[tmp]);
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//inbound truck id
			for (int w = 0; w < SmpNum; w++) {
				total_cost += params.funOTD[tmp][realSTY[tmp][w]];//delta_OverTime
			}

			for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
				preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
				if (solx[preid] >= 0) {//被安排
					transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
					delta_trans = 0;

					for (int w = 0; w < SmpNum; w++) {
						if (realSTY[tmp][w] + params.OTK[tmp].GT <= params.OTK[tmp].LF) {
							delta_trans += params.funTrans[preid][tmp][ESTX[preid][w]][realSTY[tmp][w] - transtime];//delta_Transcost}
						}
					}
					total_cost += -delta_trans * params.FP[preid][tmp];

				}
			}
		}
	}
}

void SAA::SGST(long long int &total_cost, int *sty) {
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	total_cost = 0;

	int preT = 0;

	for (int k = 0; k < params.nbInDoors; k++) {
		for (int i = 1; i <= SequenceX[k][0]; i++) {
			tmp = SequenceX[k][i];//inbound truck id
			for (int w = 0; w < SmpNum; w++) {
				if (i == 1) {
					ESTX[tmp][w] = max(sampleX[tmp][w], 0);

				}
				else {
					preT = SequenceX[k][i - 1];
					ESTX[tmp][w] = max(sampleX[tmp][w], min(ESTX[preT][w] + params.ITK[preT].GT, params.ITK[preT].LF));
				}
				total_cost += params.funOTO[tmp][ESTX[tmp][w]];
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//outbound truck id
			for (int w = 0; w < SmpNum; w++) {
				if (j == 1) {
					ESTY[tmp][w] = sampleY[tmp][w];

				}
				else {
					preT = SequenceY[l][j - 1];
					ESTY[tmp][w] = max(sampleY[tmp][w], min(ESTY[preT][w] + params.OTK[preT].GT, params.OTK[preT].LF));

				}
				if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
					sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
				}

				realSTY[tmp][w] = max(ESTY[tmp][w], sty[tmp]);
				total_cost += params.funOTD[tmp][realSTY[tmp][w]];
			}
			for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
				preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
				if (solx[preid] >= 0) {//被安排
					transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
					delta_trans = 0;
					for (int w = 0; w < SmpNum; w++) {
						if (realSTY[tmp][w] + params.OTK[tmp].GT <= params.OTK[tmp].LF) {
							delta_trans += params.funTrans[preid][tmp][ESTX[preid][w]][realSTY[tmp][w] - transtime];//delta_Transcost}
						}

					}
					total_cost += -delta_trans * params.FP[preid][tmp];
				}
			}
		}
	}
}

void SAA::SGSTSmpNum(long long int & total_cost, int _SmpNum, int * sty)
{
	int tmp = 0;
	int tmpre = 0;
	int preid = 0;
	int transtime = 0;
	int delta_trans = 0;
	total_cost = 0;
	int preT = 0;

	for (int k = 0; k < params.nbInDoors; k++) {
		for (int i = 1; i <= SequenceX[k][0]; i++) {
			tmp = SequenceX[k][i];//inbound truck id
			for (int w = 0; w < _SmpNum; w++) {
				if (i == 1) {
					ESTX[tmp][w] = max(sampleX[tmp][w], 0);

				}
				else {
					preT = SequenceX[k][i - 1];
					ESTX[tmp][w] = max(sampleX[tmp][w], min(ESTX[preT][w] + params.ITK[preT].GT, params.ITK[preT].LF));
				}
				total_cost += params.funOTO[tmp][ESTX[tmp][w]];
			}
		}
	}
	for (int l = 0; l < params.nbOutDoors; l++) {
		for (int j = 1; j <= SequenceY[l][0]; j++) {
			tmp = SequenceY[l][j];//outbound truck id
			for (int w = 0; w < _SmpNum; w++) {
				if (j == 1) {
					ESTY[tmp][w] = sampleY[tmp][w];

				}
				else {
					preT = SequenceY[l][j - 1];
					ESTY[tmp][w] = max(sampleY[tmp][w], min(ESTY[preT][w] + params.OTK[preT].GT, params.OTK[preT].LF));

				}
				if (j != 1 && sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT <= params.OTK[SequenceY[l][j - 1]].LF) {
					sty[tmp] = max(sty[tmp], sty[SequenceY[l][j - 1]] + params.OTK[SequenceY[l][j - 1]].GT);
				}

				realSTY[tmp][w] = max(ESTY[tmp][w], sty[tmp]);
				total_cost += params.funOTD[tmp][realSTY[tmp][w]];
			}
			for (int k = 1; k <= params.OTK[tmp].Pre[0]; k++) {
				preid = params.OTK[tmp].Pre[k];// 出站卡车kth的入站卡车id
				if (solx[preid] >= 0) {//被安排
					transtime = params.Time[preid][tmp] * params.Dis[solx[preid]][l];
					delta_trans = 0;
					for (int w = 0; w < _SmpNum; w++) {
						if (realSTY[tmp][w] + params.OTK[tmp].GT <= params.OTK[tmp].LF) {
							delta_trans += params.funTrans[preid][tmp][ESTX[preid][w]][realSTY[tmp][w] - transtime];//delta_Transcost}
						}

					}
					total_cost += -delta_trans * params.FP[preid][tmp];
				}
			}
		}
	}
}

/************* Baseline Policy Comparison ***********/

double SAA::CalculateCost(int M, int N,
	const std::vector<ScheduleResult>& inRes,
	const std::vector<ScheduleResult>& outRes) {

	double totalCost = 0.0;

	// 1. 入库拖期成本
	for (const auto& r : inRes) {
		if (r.doorID != -1) {
			totalCost += params.funOTO[r.truckID][r.startTime];
		}
	}

	// 2. 出库拖期成本
	for (const auto& r : outRes) {
		if (r.doorID != -1) {
			totalCost += params.funOTD[r.truckID][r.startTime];
		}
	}

	// 3. 货物转移成本计算 (计算挽回的惩罚)
	double savedPenalty = 0.0;

	for (const auto& outTruck : outRes) {
		// 如果出库卡车调度失败，无法运送任何货物
		if (outTruck.doorID == -1) continue;

		int outID = outTruck.truckID;
		int count = params.OTK[outID].Pre[0];
		int gt = params.OTK[outID].GT; // 作业时长

		for (int k = 1; k <= count; ++k) {
			int inID = params.OTK[outID].Pre[k]; // 入库车ID
			const ScheduleResult& inTruck = inRes[inID];

			// 如果入库卡车调度失败，无法供货
			if (inTruck.doorID == -1) continue;

			// 计算转移参数
			int transUnitTime = params.Time[inID][outID];
			int distance = params.Dis[inTruck.doorID][outTruck.doorID];
			int totalTransTime = transUnitTime * distance;

			// --- 核心判定逻辑 ---
			// 只要货物在 (离开时间 - 作业时间) 之前到达，就算成功
			// 这里的 finishTime 是经过动态延长的
			int effectiveDeadline = outTruck.finishTime - gt;

			// 倒推参数：有效截止时间 - 转移耗时
			int checkTimeParam = effectiveDeadline - totalTransTime;
			if (checkTimeParam < 0) checkTimeParam = 0;

			// 检查 params.funTrans (1表示成功)
			if (params.funTrans[inID][outID][inTruck.startTime][checkTimeParam] == 1) {
				savedPenalty += params.FP[inID][outID]; // 成功转移
			}
		}
	}

	// 总成本 = 总潜在惩罚 + 拖期成本 - 成功挽回的惩罚
	return params.TotalPenality + totalCost - savedPenalty;
}


double SAA::SolveOnline(int* ScenarioX, int* ScenarioY,
	int I, int J,
	int M, int N,
	PolicyType policy) {

	

	std::vector<int> inboundTrucks(M);
	for (int i = 0; i < M; ++i) inboundTrucks[i] = i;

	std::sort(inboundTrucks.begin(), inboundTrucks.end(), [&](int a, int b) {
		if (ScenarioX[a] != ScenarioX[b]) return ScenarioX[a] < ScenarioX[b];
		return params.ITK[a].EF < params.ITK[b].EF;
	});

	std::vector<ScheduleResult> inLookup(M);
	std::vector<ScheduleResult> inResVector; inResVector.reserve(M); // 用于 CalculateCost

	std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> inDoors;
	for (int k = 0; k < I; ++k) inDoors.push({ 0, k });

	for (int truckID : inboundTrucks) {
		std::pair<int, int> door = inDoors.top();
		inDoors.pop();

		int doorFreeTime = door.first;
		int doorID = door.second;
		int arrivalTime = ScenarioX[truckID];

		int startTime = std::max(arrivalTime, doorFreeTime);
		int processTime = params.ITK[truckID].GT;

		if (startTime + processTime > params.ITK[truckID].LF) {
			ScheduleResult res = { truckID, -1, -1, -1 };
			inLookup[truckID] = res;
			inResVector.push_back(res);
			inDoors.push({ doorFreeTime, doorID }); // 门退回，时间不变
		}
		else {
			// 成功
			int finishTime = startTime + processTime;
			ScheduleResult res = { truckID, doorID, startTime, finishTime };
			inLookup[truckID] = res;
			inResVector.push_back(res);
			inDoors.push({ finishTime, doorID });
		}
	}


	std::vector<ScheduleResult> outResults;
	outResults.reserve(N);

	std::vector<int> pendingOutbound;
	for (int i = 0; i < N; ++i) pendingOutbound.push_back(i);

	std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> outDoors;
	for (int k = 0; k < J; ++k) outDoors.push({ 0, k });

	while (!pendingOutbound.empty()) {
		std::pair<int, int> door = outDoors.top();
		outDoors.pop();

		int currentTime = door.first;
		int doorID = door.second;

	
		std::vector<int> readyTrucks;
		int nextArrivalTime = std::numeric_limits<int>::max();
		std::vector<int> expiredTrucks; // 存放超时的

		for (int tid : pendingOutbound) {
			int arrival = ScenarioY[tid];
			int gt = params.OTK[tid].GT;
			int lf = params.OTK[tid].LF;

			int predictedStart = std::max(arrival, currentTime);

			if (predictedStart + gt > lf) {
				expiredTrucks.push_back(tid);
				continue;
			}

			if (arrival <= currentTime) {
				readyTrucks.push_back(tid);
			}
			else {
				if (arrival < nextArrivalTime) nextArrivalTime = arrival;
			}
		}

		if (!expiredTrucks.empty()) {
			for (int expiredID : expiredTrucks) {
				ScheduleResult res = { expiredID, -1, -1, -1 };
				outResults.push_back(res);
				auto it = std::find(pendingOutbound.begin(), pendingOutbound.end(), expiredID);
				if (it != pendingOutbound.end()) pendingOutbound.erase(it);
			}
		}

		if (readyTrucks.empty()) {
			if (pendingOutbound.empty()) break;

			if (nextArrivalTime != std::numeric_limits<int>::max()) {
				outDoors.push({ nextArrivalTime, doorID });
			}
			else {
				outDoors.push({ currentTime + 1, doorID });
			}
			continue;
		}

		int selectedTruck = -1;

		if (policy == POLICY_EDD) {
			// === EDD: 选 EF 最早的 ===
			int bestDue = std::numeric_limits<int>::max();
			int bestArrival = std::numeric_limits<int>::max();

			for (int tid : readyTrucks) {
				int ef = params.OTK[tid].EF;
				int arr = ScenarioY[tid];

				if (ef < bestDue) {
					bestDue = ef;
					bestArrival = arr;
					selectedTruck = tid;
				}
				else if (ef == bestDue) {
					if (arr < bestArrival) {
						bestArrival = arr;
						selectedTruck = tid;
					}
				}
			}
		}
		else if (policy == POLICY_MAG) {
			// === MAG: 选潜在货物量最大的 ===
			// 这里的评分需要基于"如果我选了这辆车，它的动态离开时间能带走多少货"
			double maxScore = -1.0;

			for (int tid : readyTrucks) {
				int arrival = ScenarioY[tid];
				int start = std::max(arrival, currentTime);
				int gt = params.OTK[tid].GT;
				int lf = params.OTK[tid].LF;

				int minFinish = start + gt;
				int maxGoodsArrival = minFinish;

				int count = params.OTK[tid].Pre[0];
				for (int k = 1; k <= count; ++k) {
					int inID = params.OTK[tid].Pre[k];
					const ScheduleResult& inInfo = inLookup[inID];
					if (inInfo.doorID != -1) {
						int dist = params.Dis[inInfo.doorID][doorID]; // 假设分配给当前 doorID
						int transTime = params.Time[inID][tid] * dist;
						int arrivalAtDoor = inInfo.finishTime + transTime;
						if (arrivalAtDoor > maxGoodsArrival) maxGoodsArrival = arrivalAtDoor;
					}
				}
				int simulatedFinish = std::min(lf, std::max(minFinish, maxGoodsArrival));
				int effectiveDeadline = simulatedFinish - gt;

				double currentScore = 0.0;
				for (int k = 1; k <= count; ++k) {
					int inID = params.OTK[tid].Pre[k];
					const ScheduleResult& inInfo = inLookup[inID];
					if (inInfo.doorID != -1) {
						int dist = params.Dis[inInfo.doorID][doorID];
						int transTime = params.Time[inID][tid] * dist;
						int checkTimeParam = effectiveDeadline - (transTime);
						if (checkTimeParam < 0) checkTimeParam = 0;

						if (params.funTrans[inID][tid][inInfo.startTime][checkTimeParam] == 1) {
							currentScore += params.FP[inID][tid];
						}
					}
				}

				if (currentScore > maxScore) {
					maxScore = currentScore;
					selectedTruck = tid;
				}
				else if (std::abs(currentScore - maxScore) < 1e-6) {
					// Tie-breaker: EDD
					if (params.OTK[tid].EF < params.OTK[selectedTruck].EF) {
						selectedTruck = tid;
					}
				}
			}
		}

		if (selectedTruck != -1) {
			auto it = std::find(pendingOutbound.begin(), pendingOutbound.end(), selectedTruck);
			if (it != pendingOutbound.end()) pendingOutbound.erase(it);

			int arrival = ScenarioY[selectedTruck];
			int startTime = std::max(arrival, currentTime);

			int gt = params.OTK[selectedTruck].GT;
			int lf = params.OTK[selectedTruck].LF;

			int minFinish = startTime + gt;
			int maxGoodsArrival = minFinish;

			int count = params.OTK[selectedTruck].Pre[0];
			for (int k = 1; k <= count; ++k) {
				int inID = params.OTK[selectedTruck].Pre[k];
				const ScheduleResult& inInfo = inLookup[inID];
				if (inInfo.doorID != -1) {
					int dist = params.Dis[inInfo.doorID][doorID];
					int transTime = params.Time[inID][selectedTruck] * dist;
					int arrivalAtDoor = inInfo.finishTime + transTime;
					if (arrivalAtDoor > maxGoodsArrival) {
						maxGoodsArrival = arrivalAtDoor;
					}
				}
			}

			int finishTime = std::min(lf, std::max(minFinish, maxGoodsArrival));
			ScheduleResult res = { selectedTruck, doorID, startTime, finishTime };
			outResults.push_back(res);
			outDoors.push({ finishTime, doorID });
		}
	}

	return CalculateCost(M, N, inResVector, outResults);
}
void SAA::BaselinePolicy_profiles(double & _Total_Cost, int _SmpNum, PolicyType policy, std::string logfilename)
{
	_Total_Cost = 0;
	int  total_cost = 0;
	std::ofstream myfile(logfilename, std::ios::app);
	for (int i = 0; i < _SmpNum; ++i) {
		int * scenarioX = new int[params.nbInTrucks];
		for (int k = 0; k < params.nbInTrucks; k++) {
			scenarioX[k] = sampleX[k][i];
		}
		int * scenarioY = new int[params.nbOutTrucks];
		for (int k = 0; k < params.nbOutTrucks; k++) {
			scenarioY[k] = sampleY[k][i];
		}
		if (policy == POLICY_MAG|| policy == POLICY_EDD) {
			total_cost = (int)(SolveOnline(scenarioX, scenarioY, params.nbInDoors, params.nbOutDoors, params.nbInTrucks, params.nbOutTrucks, policy));
		}
		myfile << total_cost << endl;
		_Total_Cost += total_cost;
		delete[] scenarioX;
		delete[] scenarioY;
	}
	myfile.close();
	_Total_Cost = _Total_Cost / _SmpNum;
}

void SAA::LoadDeterministicPolicy(std::string filename)
{
	std::string content, content2, content3;
	cout << "DeterministicPolicy = "<<filename << endl;
	// Read INPUT dataset
	std::ifstream inputFile(filename);
	std::string line;
	double minCost = 999999999;
	if (inputFile.is_open())
	{
		std::vector<int> tempInSequence[200], tempOutSequence[200];
		int InStart[1000] = { 0 };
		int OutStart[1000] = { 0 };
		double currentCost = 0;
		bool isMinCostSolution = false;

		while (std::getline(inputFile, line)) {
			if (line.find("Cost") != std::string::npos) {
				std::stringstream ss(line);
				std::string temp;
				ss >> temp >> currentCost;

				if (currentCost < minCost) {
					minCost = currentCost;
					isMinCostSolution = true;
				}
				else {
					isMinCostSolution = false;
				}
			}
			else if (line.find("InDoor") != std::string::npos || line.find("OuDoor") != std::string::npos) {
				int doorIndex;
				std::string doorTypeStr;
				std::stringstream ss(line);

				ss >> doorTypeStr;  // 读取 "InDoor" 或 "OuDoor"
				std::string indexStr;
				ss >> indexStr;     // 读取带有 "#" 的编号字符串

				if (indexStr[0] == '#') {
					doorIndex = std::stoi(indexStr.substr(1)) - 1;
				}
				else {
					doorIndex = -1;
				}


				char doorType = doorTypeStr[0];
				std::vector<int> &sequence = (doorType == 'I') ? tempInSequence[doorIndex] : tempOutSequence[doorIndex];
				sequence.clear();

				std::string truckData;
				while (ss >> truckData) {
					size_t pos = truckData.find('(');
					int truckNum = std::stoi(truckData.substr(0, pos)); 
					int startTime = std::stoi(truckData.substr(pos + 1, truckData.find(')') - pos - 1));

					sequence.push_back(truckNum);
					if (doorType == 'I') {
						InStart[truckNum] = startTime;
					}
					else {
						OutStart[truckNum] = startTime;
					}
				}
			}

			if (isMinCostSolution) {
				int inId = 0;
				for (int i = 0; i < params.nbInDoors; ++i) {
					int cnt = 0;
					//std::cout << "InDoor #" << i + 1 << " trucks count: " << tempInSequence[i].size() << std::endl;
					for (size_t j = 0; j < tempInSequence[i].size(); ++j) {
						inId = tempInSequence[i][j];
						if (InStart[inId] >= 0) {
							cnt++;
							SequenceX[i][cnt] = inId;
							solx[inId] = i;
							stx[inId] = InStart[inId];
						}
						else {
							solx[inId] = -1;
							stx[inId] = -1;
						}
					}
					SequenceX[i][0] = cnt;
				}

				int outId = -1;
				for (int i = 0; i < params.nbOutDoors; ++i) {
					int cnt = 0;
					for (size_t j = 0; j < tempOutSequence[i].size(); ++j) {
						outId = tempOutSequence[i][j];
						if (OutStart[outId] >= 0) {
							cnt++;
							SequenceY[i][cnt] = outId;
							soly[outId] = i;
							sty[outId] = OutStart[outId];
							L[outId] = cnt;
						}
						else {
							soly[outId] = -1;
							sty[outId] = -1;
							L[outId] = -1;
						}
					}
					SequenceY[i][0] = cnt;
				}
				isMinCostSolution = false;
			}
		}

		for (int i = 0; i < params.nbInDoors; ++i) {
			std::cout << "InDoor #" << i << ": ";
			for (int j = 1; j <= SequenceX[i][0]; ++j) {
				std::cout << SequenceX[i][j] << "(" << stx[SequenceX[i][j]] << ") ";
			}
			std::cout << "\n";
		}
		std::cout << "OutDoor sequences:\n";
		for (int i = 0; i < params.nbOutDoors; ++i) {
			std::cout << "OutDoor #" << i << ": ";
			for (int j = 1; j <= SequenceY[i][0]; ++j) {
				std::cout << SequenceY[i][j] << "(" << sty[SequenceY[i][j]] << ") ";
			}
			std::cout << "\n";
		}
		inputFile.close();
	}
	else
		throw std::string("Impossible to open instance file: " + filename);
}

//debug purpose
void SAA::SetSequence()
{
	int a[3] = { 3,4,3 };
	int b[3] = { 4,4,2 };

	int ssolx[10] = { 1,2,2,2,1,1,1,0,0,0 };
	int ssoly[10] = { 2,1,2,1,0,1,0,0,0,1 };

	int stxx[10] = { 46,140,125,169,64,114,92,142,160,33 };
	int styy[10] = { 161,109,151,218,208,98,114,237,176,132 };
	int a1[3] = { 9,7,8 };
	int a2[4] = { 0,4,6,5 };
	int a3[3] = { 2,1,3 };
	int b1[4] = { 6,8,4,7 };
	int b2[4] = { 5,1,9,3 };
	int b3[2] = { 2,0 };
	for (int i = 0; i < 10; i++) {
		solx[i] = ssolx[i];
		soly[i] = ssoly[i];
		stx[i] = stxx[i];
		sty[i] = styy[i];
	}
	for (int i = 0; i < 3; i++) {
		SequenceX[i][0] = a[i];
		SequenceY[i][0] = b[i];
	}
	for (int i = 1; i <= 3; i++) {
		SequenceX[0][i] = a1[i - 1];
	}
	for (int i = 1; i <= 4; i++) {
		SequenceX[1][i] = a2[i - 1];
	}
	for (int i = 1; i <= 3; i++) {
		SequenceX[2][i] = a3[i - 1];
	}
	for (int i = 1; i <= 4; i++) {
		SequenceY[0][i] = b1[i - 1];
	}
	for (int i = 1; i <= 4; i++) {
		SequenceY[1][i] = b2[i - 1];
	}
	for (int i = 1; i <= 2; i++) {
		SequenceY[2][i] = b3[i - 1];
	}

	int lb = 0;
	long long int simlb = 0;
	generate_EST();
	sortSenario();
	SGS2(simlb, sty);
	cout << "simlb=" << simlb << endl;
	simlb = simlb / SmpNum + params.TotalPenality;
	cout << "simlb=" << simlb << " smpnum=" << SmpNum << " params.TotalPenality;" << params.TotalPenality << endl;
	SGS(simlb, sty);
	simlb = simlb / SmpNum + params.TotalPenality;
	cout << "simlb=" << simlb << endl;

	for (int i = 0; i < params.nbInDoors; i++) {
		for (int j = 1; j <= SequenceX[i][0]; j++) {
			cout << "(" << SequenceX[i][j] << "," << stx[SequenceX[i][j]] << ") ";
		}
		cout << endl;
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		for (int j = 1; j <= SequenceY[i][0]; j++) {
			cout << "(" << SequenceY[i][j] << "," << sty[SequenceY[i][j]] << ") ";
		}
		cout << endl;
	}

	system("pause");
}

//only once
SAA::SAA(Params & params) : params(params) {
	TotalIter = 0;
	TotalFalseIter = 0;
	TotalRegret = 0;
	SimTIME = 0;
	RateAverageAffectedScenarios = 0.;
	TotalUpdate = 0;
	solx = new int[params.nbInTrucks];
	soly = new int[params.nbOutTrucks];
	stx = new int[params.nbInTrucks];
	sty = new int[params.nbOutTrucks];
	SequenceX = new int*[params.nbInDoors];
	SequenceY = new int*[params.nbOutDoors];
	sampleX = new int*[params.nbInTrucks];
	sampleY = new int*[params.nbOutTrucks];
	ESTX = new int*[params.nbInTrucks];
	ESTY = new int*[params.nbOutTrucks];
	H = new int*[params.nbOutTrucks];
	ll = new int*[params.nbOutTrucks];
	L = new int[params.nbOutTrucks];
	SenarioY = new senario*[params.nbOutTrucks];
	delta_vector = new long long int*[2];
	realSTX = new int*[params.nbInTrucks];
	realSTY = new int*[params.nbOutTrucks];
	for (int i = 0; i < params.nbInTrucks; i++) {
		solx[i] = -1;
		stx[i] = 0;
		
		sampleX[i] = new int[SmpNum];
		realSTX[i] = new int[SmpNum];
		ESTX[i] = new int[SmpNum];
		for (int j = 0; j < SmpNum; j++) {
			sampleX[i][j] = 0;
			realSTX[i][j] = 0;
			ESTX[i][j] = 0;
		}
	}
	for (int i = 0; i < params.nbOutTrucks; i++) {
		soly[i] = -1;
		sty[i] = 0;
		L[i] = -1;
		H[i] = new int[params.MAXDLINE + 1];
		ll[i] = new int[SmpNum];
		ESTY[i] = new int[SmpNum];
		realSTY[i] = new int[SmpNum];
		sampleY[i] = new int[SmpNum];
		SenarioY[i] = new senario[SmpNum];
		for (int j = 0; j < SmpNum; j++) {
			ll[i][j] = 0;
			ESTY[i][j] = 0;
			realSTY[i][j] = 0;
			sampleY[i][j] = 0;
		}
		
	}
	for (int i = 0; i < params.nbInDoors; i++) {
		SequenceX[i] = new int[params.nbInTrucks + 1];
		for (int j = 0; j < params.nbInTrucks + 1; j++) {
			SequenceX[i][j] = 0;
		}
	}
	for (int i = 0; i < params.nbOutDoors; i++) {
		SequenceY[i] = new int[params.nbOutTrucks + 1];
		for (int j = 0; j < params.nbOutTrucks + 1; j++) {
			SequenceY[i][j] = 0;
		}
	}
	for (int i = 0; i < 2; i++) {
		delta_vector[i] = new long long  int[params.nbOutTrucks];
	}

	generate_sample2(1, 0);
	//reset SmpNum to train size
	SmpNum = TrainSmpNum;
	//SetSequence();
}
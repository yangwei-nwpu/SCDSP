#pragma once
/*
 * Copyright (c) 2026 Wei Yang and contributors
 * Licensed under the MIT License.
 */

#ifndef SAA_H
#define SAA_H

#include "Params.h"
#include <array>
// Scenario identifier and earliest start time, sorted in decreasing EST order.
struct senario {
	int id;
	int EST;
	bool operator< (const senario& tmp) const
	{
		return EST > tmp.EST;
	}
};

/******* For Baseline Policy comparison *******/
// Dispatching policies evaluated in the baseline comparison experiment.
enum PolicyType {
	POLICY_EDD, //  Earliest Due Date
	POLICY_MAG,  //  Maximum Available Goods
	POLICY_CMAG, // Conditional MAG
	POLICY_CEDD
};

// Start/finish record of one truck under a baseline dispatching policy.
// 辅助结构：记录卡车调度结果
struct ScheduleResult {
	int truckID;
	int doorID;
	int startTime;
	int finishTime;
};
/******* End for Baseline Policy comparison *******/

// Main local learch structure
// Implements scenario generation, sample-average approximation, simulation, and policy evaluation.
class SAA
{

public:

	Params & params;							// Problem parameters
	// Best simulated objective and its value before stochastic local improvement.
	double local_best;// simulation local best
	double ini_sim_local;
	int TrainSmpNum = 1000;
	int TestSmpNum = 1000;
	int  SmpNum = TestSmpNum;
	int SmallSmpNum = 100;
	// Scenario-specific earliest start times and sampled processing times.
	int ** ESTX, **ESTY;
	int **sampleX, **sampleY;
	// Inbound/outbound service sequences indexed by door.
	int **SequenceX, **SequenceY;
	senario **SenarioY;
	int **H, **ll;//store the sequence and the index of senarios
	// Door assignments and planned start times for inbound and outbound trucks.
	int *solx, *soly, *stx, *sty;
	int *L;
	int ** realSTX, **realSTY;//M*SmpNum, N*SmpNum
	long long int **delta_vector;//N
	// Indicates whether the most recent stochastic refinement improved the solution.
	bool fitimp = false;
	int seed = 1;

	int TotalIter, TotalFalseIter;// Sample size analysis
	// Aggregated regret, runtime, tardiness, and penalty statistics.
	double TotalRegret, AveRateRegret;
	double SimTIME;
	std::vector<double>DetailRegret, DetailAveRate;
	double AvgTotalTardiness, AvgTotalPenaltyCost;

	double RateAverageAffectedScenarios;
	int TotalUpdate, NumAffectedScenarios;
	
	// Generates training/test scenarios and prepares earliest-start-time orderings.
	void generate_sample2(int seed, int mode);
	void generate_sample(int seed);
	void generate_EST();
	void generate_ESTP(int _SmpNum);
	int fast_compute_delta(int kth, int buf);
	void sortSenario();
	void sortSenarioP(int _SmpNum);
	// Computes and applies incremental stochastic start-time moves.
	void move_and_fast_update(int kth, int buf);
	void move_and_fast_update2(int kth, int buf);
	void move_and_update(int kth, int buf,long long int _local);
	void initial_DeltaVector();
	void initial_DeltaVector_No_FastEval(long long int local);
	// Runs the SAA local-search variants and sample-size experiment.
	void SAA_LS();
	void SAA_LS_SmpNumTest();
	void SAA_LSF();
	// Runs scenario-based decomposition with standard or fast incremental updates.
	void SBD(bool bestor);
	void SBD_Fast_Update();
	// Evaluates a schedule on the active, requested, profiled, or test sample set.
	void SGS2(long long int &total_cost, int *sty);
	void SGS2P(long long int & total_cost, int * sty, int _SmpNum);
	void SGS2_profiles(long long int &total_cost, int *sty, std::string filename);//Output the cost of each scenario to a log file
	void SGS2Small(int _smpNum, int *sty);//small replications simulation// totoal_cost is the average cost
	void SGS(long long int & total_cost, int *sty);
	void SGST(long long int & total_cost, int * sty);
	void SGSTSmpNum(long long int & total_cost,int _SmpNum, int * sty);

	/************* Baseline Policy Comparison ***********/
	// Computes and profiles the EDD/MAG-family baseline policies.
	double CalculateCost(int M, int N, const std::vector<ScheduleResult>& inRes, const std::vector<ScheduleResult>& outRes);
	double SolveOnline(int * ScenarioX, int * ScenarioY, int I, int J, int M, int N,  PolicyType policy);
	void BaselinePolicy_profiles(double & _Total_Cost, int _SmpNum, PolicyType policy, std::string logfilename);
	void LoadDeterministicPolicy(std::string filename);


	/************* End Baseline Policy Comparison ***********/

	// Copies the current door assignments into the sequence representation.
	void SetSequence();
	
	// Constructor
	// Allocates scenario data structures for the supplied problem instance.
	SAA(Params & params);
};

#endif

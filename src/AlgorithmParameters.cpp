
#include "AlgorithmParameters.h"
#include <iostream>

extern "C"
struct AlgorithmParameters default_algorithm_parameters() {
	struct AlgorithmParameters ap {};

	ap.nbGranular =50;
	ap.mu = 5;//5, 25 40
	ap.lambda = 20;//10
	ap.nbElite = 5;//4
	ap.nbClose = 4;//5
	ap.tollerance = 0.9;
	ap.useBest = false;//false: best; true: first;
	ap.useSAA = true;
	ap.seed = 1;
	ap.nbIter = 20000;//20000
	ap.nbIterTraces = 500;
	ap.timeLimit =30;
	ap.useSwapStar = 1;
	ap.maxrestart = 1;
	ap.rato = 1;
	return ap;
}

void print_algorithm_parameters(const AlgorithmParameters & ap)
{
	std::cout << "=========== Algorithm Parameters =================" << std::endl;
	std::cout << "---- nbGranular              is set to " << ap.nbGranular << std::endl;
	std::cout << "---- mu                      is set to " << ap.mu << std::endl;
	std::cout << "---- lambda                  is set to " << ap.lambda << std::endl;
	std::cout << "---- nbElite                 is set to " << ap.nbElite << std::endl;
	std::cout << "---- nbClose                 is set to " << ap.nbClose << std::endl;
	std::cout << "---- seed                    is set to " << ap.seed << std::endl;
	std::cout << "---- nbIter                  is set to " << ap.nbIter << std::endl;
	std::cout << "---- nbIterTraces            is set to " << ap.nbIterTraces << std::endl;
	std::cout << "---- timeLimit               is set to " << ap.timeLimit << std::endl;
	std::cout << "---- useSwapStar             is set to " << ap.useSwapStar << std::endl;
	std::cout << "==================================================" << std::endl;
}

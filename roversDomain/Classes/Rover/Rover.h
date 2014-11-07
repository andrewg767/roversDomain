#pragma once
#include "../EnvironmentBounds/EnvironmentBounds.h"
#include <cstdlib>
//#include "NeuralNet.h"
#include "../NeuroEvo/NeuroEvo.h"
#include <set>

#define SENSORFAILLIMIT 5.0 // amount that a 'sensorfail' type rover can deviate from desired course
enum FailType{
	NOMINAL, // no error
	SENSORFAIL, // takes inaccurate steps
	PROPULSIONLOSS, // stays in place
	PROPULSIONGAIN, // takes an extra step in whichever desired direction
	FAILTYPECOUNT // number of elements in enum
};

class Rover{
public:
	Rover();
	~Rover();
	int ID;
	bool caught;
	int max_ind(std::vector<double> myvector);
	void boundPosition();
	void walk(double dx, double dy, double percentFail);
	double coin();
	double x,y;
	EnvironmentBounds bounds;
	void randWalk(double percentFail);
	double orientation; // NOTE: ORIENTATION IS ABSOLUTE, SCALED BETWEEN 0-1 (NOT 0-2PI)
};
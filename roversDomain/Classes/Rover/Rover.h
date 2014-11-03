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

	int max_ind(std::vector<double> myvector){
		return distance(myvector.begin(),std::max_element(myvector.begin(),myvector.end()));
	}
	
	void boundPosition(){
		// bound x/y ... note, should this be done on gridworld level?
 		if (x<0) x=0;
		if (x>bounds.size('x')) x= bounds.size('x')-1;
		if (y<0) y=0;
		if (y>bounds.size('y')) y= bounds.size('y')-1;
	}

	
	void walk(double dx, double dy, double percentFail){
		// Normal walking behavior
		x+=dx;
		y+=dy;
		boundPosition();
	}

	double coin(){
		return double(rand())/double(RAND_MAX);
	}

	int x,y;
	EnvironmentBounds bounds;
	
	void randWalk(double percentFail){
		/*int direction = rand()%9+1; // select between directions (numpad map)
		walk(direction, percentFail);*/
		// randomly select dx/dy
		walk(rand()%2,rand()%2,percentFail); // take 0-1 steps in any direction
	}

	// for predPrey
	double orientation; // NOTE: ORIENTATION IS ABSOLUTE, SCALED BETWEEN 0-1 (NOT 0-2PI)
};
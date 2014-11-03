#pragma once
#include <vector>
//#include "GridWorld.h"

//#define STATEELEMENTS 5 // This defines the number of state elements

/* STATE DEFINITION:

[0-3]: rovers in q1-q4
[4-7]: pois in q1-q4


#define STATEELEMENTS 8

*/

typedef int Action;

class State{
public:
	State(){}; // DEFAULT CONSTRUCTOR
	~State();
	std::vector<double> sensorInputs;
	State(std::vector<double> inputs){
		sensorInputs=inputs;
	}

	struct cmpState{ // for map this should be operator< function
		bool operator()(const State& s1, const State& s2) const{ // direct mapping of sensor inputs to state number...
			for (int i=0; i<s1.sensorInputs.size(); i++){
				if (s1.sensorInputs[i]<s2.sensorInputs[i]){ // less than
					return true;
				} else if (s1.sensorInputs[i]>s2.sensorInputs[i]){ // greater than
					return false;
				} else {
					continue;
				}
			}
			return false; // Equal
		}
	};
};
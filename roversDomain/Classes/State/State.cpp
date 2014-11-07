#include "State.h"


State::~State(void)
{
}

State::State(std::vector<double> inputs){
	sensorInputs=inputs;
}

bool State::cmpState::operator()(const State& s1, const State& s2) const{ // direct mapping of sensor inputs to state number...
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
#pragma once
#include <vector>

typedef int Action;

class State{
public:
	State(){}; // DEFAULT CONSTRUCTOR
	~State();
	std::vector<double> sensorInputs;
	State(std::vector<double> inputs);

	struct cmpState{ // for map this should be operator< function
		bool operator()(const State& s1, const State& s2) const;
	};
};
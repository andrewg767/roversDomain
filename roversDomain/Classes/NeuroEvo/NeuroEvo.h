#pragma once

#include "../NeuralNet/NeuralNet.h"
#include <set>
#include <utility>
#include <algorithm>
#include <list>


class NeuroEvoParameters{
public:
	NeuroEvoParameters(int inputSet, int outputSet):
		nInput(inputSet),
		nHidden(15),
		nOutput(outputSet),
		popSize(50){}

	int nInput;
	int nHidden;
	int nOutput;
	int popSize; // suriving population size
	double epsilon;
};

class NeuroEvo
{
public:
	NeuroEvo(){};
	NeuroEvo(NeuroEvoParameters* neuroEvoParamsSet); // Create a neuro-evo 'agent' with defined neural network parameters and a population size
	~NeuroEvo(void);

	NeuroEvoParameters* params;
	void generateNewMembers(); // Generate k members to evaluate (also, select member 0 to begin evaluation)
	
	std::vector<double> getOutput(std::vector<double> inputs); // during simulation, use current NN to get actions (NOTE: could scale this at the end)

	bool selectNewMember(); // Select the next member to test; if cannot be selected, end epoch
	double getBestMemberVal(); // get the highest evaluation in the group

	void setNNToBestMember();


	void selectSurvivors();

	//void selectAndMutate(void);
	//void replaceWorstWithCurrent(double currentNNScore);
	std::list<NeuralNet*> population;

	static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {return (x->evaluation>y->evaluation);}
};


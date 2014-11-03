#include "NeuroEvo.h"

using namespace std;

NeuroEvo::NeuroEvo(NeuroEvoParameters* neuroEvoParamsSet)
{
	params = neuroEvoParamsSet;
	for (int i=0; i<params->popSize; i++){
		NeuralNet* nn = new NeuralNet(params->nInput,params->nHidden,params->nOutput);
		population.push_back(nn);
	}
}


NeuroEvo::~NeuroEvo(void)
{
	while (!population.empty()){
		delete population.back();
		population.pop_back();
	}
}

void NeuroEvo::generateNewMembers(){
	// Mutate existing members to generate more
	list<NeuralNet*>::iterator popMember=population.begin();
	for (int i=0; i<params->popSize; i++){ // add k new members
		(*popMember)->evaluation = 0.0;
		NeuralNet* m = new NeuralNet(**popMember); // dereference pointer AND iterator
		m->mutate();
		population.push_back(m);
		popMember++;
	}
}

double NeuroEvo::getBestMemberVal(){
	// Find the HIGHEST FITNESS value of any neural network
	double highest = population.front()->evaluation;
	for (list<NeuralNet*>::iterator popMember=population.begin(); popMember!=population.end(); popMember++){
		if (highest<(*popMember)->evaluation) highest=(*popMember)->evaluation;
	}
	return highest;
}


void NeuroEvo::selectSurvivors(){
	// Select neural networks with the HIGHEST FITNESS
	population.sort(NNCompare); // Sort by the highest fitness
	int nExtraNN = population.size()-params->popSize;
	for (int i=0; i<nExtraNN; i++){ // Remove the extra
		delete population.back();
		population.pop_back();
	}
}
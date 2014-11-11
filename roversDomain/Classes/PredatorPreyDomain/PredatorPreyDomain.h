#pragma once
#include "../GridWorld/GridWorld.h"
#include "omp.h"
#include <time.h>

class Predator:
	public Rover
{
public:
	Predator(bool usingTypes);

	// Types definition
	const enum PredatorTypes{Normal,CW, CCW, Fast, numTypes};
	const enum stateElementNames {orientationMe,dxNearestPrey,dyNearestPrey,dxNearestNeighbor,dyNearestNeighbor,numNonTypeElements};

	double moveDistCap;
	PredatorTypes type;

	bool involvedInCapture;
	//int stepsToCapture;
	std::vector<double> stateInputs;
	std::vector<double> actionToTake; // output as a dx dy command


	void selectAction(NeuralNet* NN);
};


class PredatorPreyDomainParameters{
public:
	PredatorPreyDomainParameters();
	bool usingTypes;
	int nPredators;
	int nPrey;
	const enum RewardTypes{Global};
	RewardTypes rewardType;
	std::vector<Predator::PredatorTypes> fixedTypes;
};


class PredatorPreyDomain :
	public GridWorld
{
public:
	void detectZero();
	PredatorPreyDomainParameters* params;
	PredatorPreyDomain(PredatorPreyDomainParameters* paramsSet);
	~PredatorPreyDomain(void);
	bool allCaptured;
	std::vector<bool> captured; // as large as the prey; defines capture or not
	std::vector<int> stepsToCapture; // this is also as large as the prey
	std::vector<int> predStepsToCapture;
	void outputSteps();

	//Predators functions/elements
	std::vector<std::vector<double> > deltaPredPrey;
	enum PredatorTypes{Normal,CW, CCW, Fast, numTypes};
	void getPredatorState (Predator &p);

	std::vector<std::vector<std::vector<double> > > stepLog;
	std::vector<Predator> predators;
	State getState(Predator &p, bool returnBlankState=false); // deprecated
	void randomizePredatorPositions();
	void randomizePreyPositions();
	void setPredatorActions(std::vector<NeuralNet*> NNSet);
	void setPreyActions(std::vector<std::vector<double> > &preyActions);
	void movePredators();
	void movePrey(std::vector<std::vector<double> > &preyActions);
	bool checkSystemCapture(int t);

	// Prey functions/elements
	bool checkCapture(Rover &p);
	bool checkKill(Rover &p);
	std::vector<Rover> prey;
	std::vector<double> getLocalPreyReward(void);
	std::vector<double> getLocalPredReward(void);
	State getPreyState (int me, bool returnBlankState=false);

	// General domain functions
	void initializePredPreyRun();
	void initializePredPreyEpisode(int steps);
	void setUniqueRandomPredPreyPositions();

	void simulatePredPreyEpisode(std::vector<NeuralNet*> NNSet, std::vector<double> &predFitnesses, std::vector<double> &preyFitnesses);
	void move(Rover &me, std::vector<double> &action);
	void movePred(Predator &me, std::vector<double> &action);
	GridWorld::PairQueueAscending sortedPredatorDists(double xref, double yref);
	GridWorld::PairQueueAscending sortedPreyDists(double xref, double yref);

	void showTerrain();

	// parent functions
	double getLocalReward(int me); // TODO
	double getGlobalReward(); // TODO
	std::vector<double> getDifferenceReward(); // TODO

};

// uncategorized functions
void simulatePredPreyRun(PredatorPreyDomainParameters* PPparams, int nEpochs, int nTrials, std::vector<double> &GLog);
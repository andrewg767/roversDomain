#pragma once
#include "../GridWorld/GridWorld.h"
#include "omp.h"
#include <time.h>

class Predator:
	public Rover
{
public:
	Predator(bool usingTypes):Rover(){
		// fill with other things
		static int IDset = 0;
		ID = IDset++;

		moveDistCap=2.0;
		type = PredatorTypes(rand()%int(numTypes));

		stateInputs = std::vector<double>(numNonTypeElements + int(numTypes)*(usingTypes?1:0),0.0);
	};

	// Types definition
	const enum PredatorTypes{Normal,CW, CCW, Fast, numTypes};
	const enum stateElementNames {orientationMe,dxNearestPrey,dyNearestPrey,dxNearestNeighbor,dyNearestNeighbor,numNonTypeElements};

	double moveDistCap;
	PredatorTypes type;

	bool involvedInCapture;
	//int stepsToCapture;
	std::vector<double> stateInputs;
	std::vector<double> actionToTake; // output as a dx dy command


	void selectAction(NeuralNet* NN){ // given a certain stateInputs (defined prior) get an action
		// Sets the values of actionToTake
		actionToTake = NN->predictContinuous(stateInputs); // gives [rotation dtheta][movement d]
		actionToTake[0] = 2.0*actionToTake[0]-1.0; // scale between [-1,1]


		//printf("NN out = %f,%f\n",actionToTake[0],actionToTake[1]);
		
		// Type implementation scales the output
		if (type==CCW){
			// scale the neural network orientation command between 0-pi
			actionToTake[0]/=4.0; // HACK: can only turn 90deg max

			// Hack #2: immobile agent
			//actionToTake[1] = 0.0;
		} else if (type==CW){	
			// unreliable agent
			if (rand()%100<30){ // 30% of the time random
				actionToTake[0] = double(rand())/double(RAND_MAX)*2.0-1.0;
				actionToTake[1] = double(rand())/double(RAND_MAX);
			}

		} else if (type== Fast){
			actionToTake[1] *= 2.0; // multiply output action by 2
		}
		
		actionToTake[1]*=moveDistCap;
	}
};


class PredatorPreyDomainParameters{
public:
	PredatorPreyDomainParameters():
		usingTypes(true),
		nPredators(20),
		nPrey(8),
		rewardType(Global)
	{
		fixedTypes= std::vector<Predator::PredatorTypes>(nPredators);
		for (int i=0; i<fixedTypes.size(); i++){
			// Creates an even distribution of predator types
			fixedTypes[i] = Predator::PredatorTypes(i%int(Predator::PredatorTypes::numTypes));
		}
	};
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
	
	void randomizePredatorPositions(){
		for (int i=0; i<predators.size(); i++){
			predators[i].x = rand()%bounds.size('x');
			predators[i].y = rand()%bounds.size('y');
		}

		bool blocked = true;
		while (blocked){
			blocked = false;	
			for (int i=0; i<predators.size()-1; i++){
				for (int j=i+1; j<predators.size(); j++){
					if (!gridDistance(predators[i],predators[j])){
						predators[j].x = rand()%bounds.size('x');
						predators[j].y = rand()%bounds.size('y');
						blocked = true;
						break;
					}
				}
				if (blocked) break;
			}
		}
	}
	void randomizePreyPositions(){
		for (int i=0; i<prey.size(); i++){
			prey[i].x = rand()%bounds.size('x');
			prey[i].y = rand()%bounds.size('y');
		}

		bool blocked = true;
		while (blocked){
			blocked = false;	
			for (int i=0; i<prey.size()-1; i++){
				for (int j=i+1; j<prey.size(); j++){
					if (!gridDistance(prey[i],prey[j])){
						prey[j].x = rand()%bounds.size('x');
						prey[j].y = rand()%bounds.size('y');
						blocked = true;
						break;
					}
				}
				if (blocked) break;
			}
		}
	}

	void setPredatorActions(std::vector<NeuralNet*> NNSet){
		for (int i=0; i<predators.size(); i++){
			if (!predators[i].caught){
				getPredatorState(predators[i]);
				predators[i].selectAction(NNSet[i]); // uses stateInputs previously set
			}
		}
	}

	std::vector<double> getParticleVector(int j){
		GridWorld::PairQueueAscending pq = sortedPreyDists(prey[j].x,prey[j].y);
		double sumcharges;
		for (int i=0; i<4; i++){
			//force:
			double r = pq.top().first;
			double fx = predators[pq.top().second].x - prey[j].x;
			double fy = predators[pq.top().second].y - prey[j].y;
			
		}
	}

	void setPreyActions(std::vector<std::vector<double> > &preyActions){
		for (int j=0; j<prey.size(); j++){
			preyActions[j] = std::vector<double>(2);
			if (!prey[j].caught){
				preyActions[j][0] = double(rand())/double(RAND_MAX);// random movements
				// escaping prey

				double fx = 0.0;
				double fy = 0.0;
				for (int i=0; i<predators.size(); i++){
					double dx = predators[i].x - prey[j].x;
					double dy = predators[i].y - prey[j].y;
					double dxy = sqrt(dx*dx + dy*dy);

					double Fpred = 1/(dxy*dxy); // note: could multiply by charge force; omitting because assuming constant
					fx += Fpred*(-dx/dxy); // F*xhat, also changes direction (opposite of predators)
					fy += Fpred*(-dy/dxy); // F*yhat
				}
				double fDir = (atan2(fy,fx)/3.1416); //[-1,1]
				fDir *= 0.5; // rescale to bounds [0,1]
				if (fDir<0) fDir = 1.0-fDir; // rescale to bounds [0,1]

				// ABSOLUTELY SET ORIENTATION
				prey[j].orientation = fDir;
				preyActions[j][0]=0; // already set by fdir
				preyActions[j][1]=2.0;
			}
		}
	}

	void movePredators(){
		
		for (int j=0; j<predators.size(); j++){
			if (!predators[j].caught) movePred(predators[j],predators[j].actionToTake); // previously set by selectAction; ONLY the predators not eating move
		}
	}

	void movePrey(std::vector<std::vector<double> > &preyActions){
		for (int j=0; j<prey.size(); j++){
			if (!prey[j].caught) move(prey[j],preyActions[j]);
		}
	}

	bool checkSystemCapture(int t){
		// Goal check, mixed with reward updating
		int nCaptured = 0;
		for (int j=0; j<prey.size(); j++){
			if (!captured[j]){
				captured[j] = checkCapture(prey[j]);
				if (!captured[j]){
					stepsToCapture[j]++;
				}
			} else {
				nCaptured++;
			}
		}
		for (int j=0; j<predators.size(); j++){
			if(!predators[j].caught) predStepsToCapture[j]++;
		}

		if (nCaptured==prey.size()) allCaptured = true;
		
		for (int j=0; j<predators.size(); j++){
			GridWorld::PairQueueAscending pq = sortedPreyDists(predators[j].x,predators[j].y);
			while(pq.size() && prey[pq.top().second].caught){
				pq.pop();
			}
			
			if (pq.size()==0){
				deltaPredPrey[j][t] = 0.0; // no penalty, all caught!
			} else {
				deltaPredPrey[j][t] = pq.top().first;
			}
		}
		
		return allCaptured;
	}

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
	void setUniqueRandomPredPreyPositions(){
		std::set<std::pair<int,int> > positionSet;
		while (positionSet.size()<(predators.size()+prey.size())){
			positionSet.insert(std::make_pair(rand()%bounds.size('x'),rand()%bounds.size('y')));
		}
		for (int i=0; i<predators.size(); i++){
			int randEntry = (rand()%positionSet.size());
			std::set<std::pair<int,int> >::iterator it = positionSet.begin();
			std::advance(it,randEntry);
			predators[i].x = it->first;
			predators[i].y = it->second;
			positionSet.erase(it);
		}

		for (int i=0; i<prey.size(); i++){
			int randEntry = (rand()%positionSet.size());
			std::set<std::pair<int,int> >::iterator it = positionSet.begin();
			std::advance(it,randEntry);
			prey[i].x = it->first;
			prey[i].y = it->second;
			positionSet.erase(it);
		}
	}

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
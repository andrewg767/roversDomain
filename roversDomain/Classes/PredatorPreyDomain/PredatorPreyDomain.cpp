#include "PredatorPreyDomain.h"

void generateNewMembers(std::vector<NeuroEvo*> &NESet){
	// create new members
	for (int i=0; i<NESet.size(); i++){
		NESet[i]->generateNewMembers();
	}
}


std::vector<double> mean1(std::vector<std::vector<double> > myVector){
	std::vector<double> myMean(myVector.size(),0.0);

	for (int i=0; i<myVector.size(); i++){
		for (int j=0; j<myVector[i].size(); j++){
			myMean[i]+=myVector[i][j];
		}
		myMean[i]/=double(myVector[i].size());
	}
	return myMean;
}

std::vector<double> mean2(std::vector<std::vector<double> > myVector){
	std::vector<double> myMean(myVector[0].size(),0.0);

	for (int i=0; i<myVector.size(); i++){
		for (int j=0; j<myVector[i].size(); j++){
			myMean[j]+=myVector[i][j]/double(myVector.size());
		}
	}
	return myMean;
}

void simulatePredPreyEpisodeFull(std::vector<NeuralNet*> NNSet, std::vector<PredatorPreyDomain*> trials){
	// Parallelized SimulatePredPreyEpisode, with external elements of neuroevo
	// NNSet is the set of neural networks being tested, 1/predator

	std::vector<std::vector<double> > predFitnesses(trials.size());
	std::vector<std::vector<double> > preyFitnesses(trials.size());

//#pragma omp parallel for // TODO: verify parallelizability
	for (int t=0; t<trials.size(); t++){
		//printf("t=%i, ",t);
		trials[t]->simulatePredPreyEpisode(NNSet,predFitnesses[t],preyFitnesses[t]); // pred/prey fitnesses set inside this
	}

	std::vector<double> predFit = mean2(predFitnesses);
	std::vector<double> preyFit = mean2(preyFitnesses);


	for (int i=0; i<NNSet.size(); i++){
		NNSet[i]->evaluation = predFit[i];
	}

}

void simulatePredPreyEpoch(std::vector<NeuroEvo*> &NESet, std::vector<std::vector<PredatorPreyDomain*> > &domains){
	// Domains: vector of domains that is population.size() x trials large, each an individual simulation
	// Prey movement: Random

	generateNewMembers(NESet); // NESet represents the predators; random prey movement
	// Preprocessing: access a set of neural networks given by each neuroEvo element
	int numNN = NESet[0]->population.size();
	std::vector<std::vector<NeuralNet*> > NNSets(numNN); // Set to the number of population members
	
	std::vector<std::list<NeuralNet*>::iterator> popMembersInNNSet(NESet.size()); // list of iterators to set members across NE objects
	for (int i=0; i<NESet.size(); i++){ // initialization of iterator list
		popMembersInNNSet[i]=NESet[i]->population.begin();
	}
	for (int i=0; i<numNN; i++){
		NNSets[i] = std::vector<NeuralNet*>(popMembersInNNSet.size());
		for (int j=0; j<popMembersInNNSet.size(); j++){
			NNSets[i][j] = *popMembersInNNSet[j];
			popMembersInNNSet[j]++; // iterate up when done with assignment
		}
	}

//#pragma omp parallel for // MEMDEBUG	
	for (int nthNN=0; nthNN<NESet[0]->population.size(); nthNN++){ // go through every neural network ...
		simulatePredPreyEpisodeFull(NNSets[nthNN],domains[nthNN]);
	}

	for (int i=0; i<NESet.size(); i++){
		NESet[i]->selectSurvivors();
	}
}


PredatorPreyDomain::PredatorPreyDomain(PredatorPreyDomainParameters* paramsSet){
	params = paramsSet;
}


PredatorPreyDomain::~PredatorPreyDomain(void)
{
}

bool PredatorPreyDomain::checkKill(Rover &p){
	for (int i=0; i<predators.size(); i++){
		if (!gridDistance(predators[i],p)) return true;
	}
	return false;
}

double getAverageBestNNScore(std::vector<NeuroEvo*> &NESet){
	double NNBestSum = 0.0;
		for (int i=0; i<NESet.size(); i++){
			NNBestSum += NESet[i]->getBestMemberVal();
		}
		return NNBestSum/double(NESet.size());
	}

void PredatorPreyDomain::detectZero(){
	for (int j=0; j<predators.size(); j++){
			for (int k=0; k<prey.size(); k++){
				if (!gridDistance(predators[j], prey[k])){
					printf("Zero detected.\n");
				}
			}
		}
}

void PredatorPreyDomain::move(Rover &me, std::vector<double> &action){
		// action is: [rotation modification][distance to move] 

		double preyMoveDistCap = 2.0;
		me.orientation+= action[0];
	//	printf("Action 0 was %f\n",action[0]);
		if (me.orientation>1.0) me.orientation -=1.0; // adjust for turning
		else if (me.orientation<0.0) me.orientation +=1.0; // adjust for turning
		

		// The xy vals of proposed movement
		double xset = me.x;
		double yset = me.y;
	
		double dx = floor(preyMoveDistCap*action[1]*cos(me.orientation*2.0*3.14));
		double dy = floor(preyMoveDistCap*action[1]*sin(me.orientation*2.0*3.14));
		xset+= dx;
		yset+= dy;

		//cap
		bounds.cap(xset,'x');
		bounds.cap(yset,'y');

		// restore original position if another exists in that spot
		bool blocked = false;
		for (int i=0; i<predators.size(); i++){ // check if any predators in that spot
			if (predators[i].x==xset && predators[i].y==yset){
				blocked=true;
				break;
			}
		}
		for (int i=0; i<prey.size(); i++){
			if (prey[i].x==xset && prey[i].y==yset){ // check if any prey in that spot
				blocked=true;
			}
		}

		if (!blocked){
			me.x=xset;
			me.y=yset;
		}
		
		//detectZero();
	}

void PredatorPreyDomain::movePred(Predator &me, std::vector<double> &action){
		// action is: [rotation modification][distance to move]

		me.orientation+= action[0]/2.0; // only 180deg turns OK
		if (me.orientation>=1.0){
			me.orientation -=1.0; // adjust for turning
		} else if (me.orientation<0.0){
			me.orientation +=1.0; // adjust for turning
		}
		

		// The xy vals of proposed movement
		double xset = me.x;
		double yset = me.y;

		double xAdd = floor(action[1]*cos(me.orientation*2.0*3.14));
		double yAdd = floor(action[1]*sin(me.orientation*2.0*3.14));

		xset+=xAdd;
		yset+=yAdd;

		//cap
		bounds.cap(xset,'x');
		bounds.cap(yset,'y');

		// restore original position if another exists in that spot
		bool blocked = false;
		for (int i=0; i<predators.size(); i++){ // check if any predators in that spot
			if (predators[i].x==xset && predators[i].y==yset){
				blocked=true;
				break;
			}
		}
		for (int i=0; i<prey.size(); i++){
			if (prey[i].x==xset && prey[i].y==yset){ // check if any prey in that spot
				blocked=true;
			}
		}

		if (!blocked){
			me.x=xset;
			me.y=yset;
		}
		
		//detectZero(); // for debugging
	}

GridWorld::PairQueueAscending PredatorPreyDomain::sortedPredatorDists(double xref, double yref){
		typedef std::pair<double,int> P;
		std::vector<P> dists(predators.size(),std::make_pair<double,int>(0.0,0));
		for (int j=0; j<predators.size(); j++){
			dists[j]=P(gridDistance(xref,yref,predators[j].x,predators[j].y),j);
		}

		PairQueueAscending q(dists.begin(),dists.end());
		return q;
	}

GridWorld::PairQueueAscending PredatorPreyDomain::sortedPreyDists(double xref, double yref){
	// distance of predator away from each prey
		typedef std::pair<double,int> P;
		std::vector<P> dists(prey.size(),std::make_pair<double,int>(0.0,0));
		for (int j=0; j<prey.size(); j++){
			dists[j]=P(gridDistance(xref,yref,prey[j].x,prey[j].y),j);
		}

		PairQueueAscending q(dists.begin(),dists.end());
		return q;
	}

void PredatorPreyDomain::initializePredPreyRun(){
	/*
	Creates settings. Also places the predators and prey in the world and initializes them.
	*/


	deltaPredPrey.clear();
	deltaPredPrey=std::vector<std::vector<double> >(params->nPredators);

	GLog.clear();

	for (int i=0; i<params->nPredators; i++){
		predators.push_back(Predator(params->usingTypes));
		predators[i].type = params->fixedTypes[i];
	}
	for (int i=0; i<params->nPrey; i++){
		prey.push_back(Rover());
	}
}

void PredatorPreyDomain::outputSteps(){
	std::ofstream fs;
	fs.open("steplog.txt");
	for (int i=0; i<stepLog.size(); i++){
		for (int j=0; j<stepLog[i].size(); j++){
			fs << stepLog[i][j][0] << "," << stepLog[i][j][1] << ",";
		}
		fs << "\n";
	}
	fs.close();
	system("pause");
}

void simulatePredPreyRun(PredatorPreyDomainParameters* PPparams, int nEpochs, int nTrials, std::vector<double> &GLog){ //PREDPREY
	// PPparams should be initialized outside of this function to reflect the settings (types, reward, etc) that is being varied

	// Run settings and logging
	GLog = std::vector<double>(nEpochs,0.0); // resets and allocates global reward logging

	NeuroEvoParameters* NEParams = new NeuroEvoParameters(Predator::numNonTypeElements + int(Predator::numTypes)*(PPparams->usingTypes?1:0),2);
//	NeuroEvoParameters* NEParams = new NeuroEvoParameters(Predator::numNonTypeElements,2); // both start the same for now...

	std::vector<std::vector<PredatorPreyDomain*> > allTrialDomainsForEpoch(NEParams->popSize*2); // hardcoded: 2k=100 neural networks... edit this later

	//int typeDelay = nEpochs; // backwards hack: this will make delay UNTIL step 1/2 the run; 'typedelay' is amount of time types has to learn

	/*
	bool trueUsingTypes = false;
	if (PPparams->usingTypes){
		PPparams->usingTypes=false;
		trueUsingTypes = true;
	}
	*/

	for (int i=0; i<allTrialDomainsForEpoch.size(); i++){
		allTrialDomainsForEpoch[i] = std::vector<PredatorPreyDomain*>(nTrials);
		for (int j=0; j<allTrialDomainsForEpoch[i].size(); j++){
			allTrialDomainsForEpoch[i][j] = new PredatorPreyDomain(PPparams);
			allTrialDomainsForEpoch[i][j]->initializePredPreyRun();
		}
	}

	std::vector<NeuroEvo*> NESet(PPparams->nPredators);
	for (int i=0; i<PPparams->nPredators; i++){
		NESet[i] = new NeuroEvo(NEParams);
	}

	
	//if (trueUsingTypes) nEpochs = nEpochs-typeDelay; // prepares for break later, to insert stuff for typing

	for (int i=0; i<nEpochs; i++){
		//printf("Epoch %i\n",i);
		if (i%50==0) printf("(%i/%i)",i,nEpochs);
		else printf(".");
		simulatePredPreyEpoch(NESet,allTrialDomainsForEpoch);		
		//outputSteps();
		GLog[i] = getAverageBestNNScore(NESet);
	}

	// using types again
/*	if (trueUsingTypes){
		PPparams->usingTypes=true;
		nEpochs = nEpochs+typeDelay;
		for (int i=0; i<NESet.size(); i++){
			for (std::list<NeuralNet*>::iterator popMember=NESet[i]->population.begin(); popMember!=NESet[i]->population.end(); popMember++){
				(*popMember)->addInputs(Predator::numTypes);
			}
		}
		NEParams->nInput += Predator::numTypes;
		
		for (int i=0; i<allTrialDomainsForEpoch.size(); i++){
			for (int j=0; j<allTrialDomainsForEpoch[i].size(); j++){
				delete allTrialDomainsForEpoch[i][j]; // object cleanup
				allTrialDomainsForEpoch[i][j] = new PredatorPreyDomain(PPparams);
				allTrialDomainsForEpoch[i][j]->initializePredPreyRun();
			}
		}

		for (int i=nEpochs-typeDelay; i<nEpochs; i++){
			printf("Epoch %i\n",i);
			simulatePredPreyEpoch(NESet,allTrialDomainsForEpoch);		
			//outputSteps();
			GLog[i] = getAverageBestNNScore(NESet);
		}
	}
	*/

	// OBJECT CLEANUP
	for (int i=0; i<allTrialDomainsForEpoch.size(); i++){ // remove domains not being used
		for (int j=0; j<allTrialDomainsForEpoch[i].size(); j++){
			delete allTrialDomainsForEpoch[i][j];
		}
	}
	for (int i=0; i<NESet.size(); i++){
		delete NESet[i];
	}
	delete NEParams; // remove parameters not being used
}


double sum(std::vector<double> myVector){
	double mySum = 0.0;
	for (int i=0; i<myVector.size(); i++){
		mySum+=myVector[i];
	}
	return mySum;
}

std::vector<double> sum(std::vector<std::vector<double> > myVector){
	std::vector<double> mySum(myVector.size(),0.0);

	for (int i=0; i<myVector.size(); i++){
		for (int j=0; j<myVector[i].size(); j++){
			mySum[i]+=myVector[i][j];
		}
	}
	return mySum;
}

double mean(std::vector<double> myVector){
	return sum(myVector)/double(myVector.size());
}





std::vector<double> PredatorPreyDomain::getLocalPredReward(void){
	// NOTE: THIS IS A LOCAL REWARD, no coordination required...
	// may later encourage coordination if other predators do not catch prey
	
	/* PREVIOUS (NOT WORKING):*/
	std::vector<double> rwdvect(predators.size(),0.0);
	for (int i=0; i<predators.size(); i++){
		if (predators[i].caught){
			rwdvect[i] = -predStepsToCapture[i];
		} else {
			double deltasum = 0.0;
			for (int t=0; t<deltaPredPrey[i].size(); t++){
				rwdvect[i] -= deltaPredPrey[i][t];
			}
		}
	}
	return rwdvect;
	//*/


	/* ALSO DEPRECATED: MORE RECENT THOUGH
	std::vector<double> rwdvect(predators.size(),0.0);
	double rwdsum = 0.0;
	if (allCaptured){
		for (int i=0; i<predStepsToCapture.size(); i++){
			rwdsum -= predStepsToCapture[i]/predStepsToCapture.size();
		}
	} else {
		for (int i=0; i<deltaPredPrey.size(); i++){
			for (int t=0; t<deltaPredPrey[i].size(); t++){
				rwdsum -= deltaPredPrey[i][t];
			}
		}
	}

	for (int i=0; i<predators.size(); i++){
		rwdvect[i] = rwdsum;
	}
	
	return rwdvect;*/

	/*std::vector<std::vector<double> > preyManhattanDists(predators.size());

	std::vector<std::vector<GridWorld::PairQueueAscending> > pq(prey.size()); // pq[prey#][slot#]
	for (int i=0; i<prey.size(); i++){
		std::vector<std::vector<double> > cs = captureSlots(prey);
		pq[i] = std::vector<GridWorld::PairQueueAscending>(captureSlots.size());
		for (int j=0; j<captureSlots.size(); j++){ // go through cardinal capture directions
			pq[i][j] = sortedPredatorDists(captureSlots[j][0],captureSlots[j][1]);
		}
	}

	// Go through and ID which ones have conflict
	for (int i=0; i<pq.size()-1; i++){
		for (int j=i+1; j<pq.size(); j++){

		}
	}*/

}

bool PredatorPreyDomain::checkCapture(Rover &p){
	std::set<std::pair<int,int> > capturePositions; // spaces a predator needs to occupy to capture the prey
	if (p.x-1>=0) capturePositions.insert(std::pair<int,int>(p.x-1,p.y));
	if (p.x+1<bounds.size('x')) capturePositions.insert(std::pair<int,int>(p.x+1,p.y));
	if (p.y-1>=0) capturePositions.insert(std::pair<int,int>(p.x,p.y-1));
	if (p.y+1<bounds.size('y')) capturePositions.insert(std::pair<int,int>(p.x,p.y+1));

	std::vector<int> catchList;
	for (int i=0; i<predators.size(); i++){
		if (capturePositions.count(std::pair<int,int>(predators[i].x,predators[i].y))){
			capturePositions.erase(std::pair<int,int>(predators[i].x,predators[i].y));
			catchList.push_back(i);
		}
	}

	if (!capturePositions.size()){ // all captured!
		p.caught=true;
		for (int i=0; i<catchList.size(); i++){
			predators[catchList[i]].caught=true; // participated in a kill
		}
		return true;
	} else return false;
}

void PredatorPreyDomain::showTerrain(){
	const int boundSize = 10;
	std::string grid[boundSize][boundSize];
	for (int y=0; y<boundSize; y++){
		for (int x=0; x<boundSize; x++){
				grid[y][x]=".";
			}
		}
		for (int p=0; p<predators.size(); p++){
			grid[predators[p].y][predators[p].x]="d";
		}
		for (int y=0; y<prey.size(); y++){
			grid[prey[y].y][prey[y].x]="y";
		}
		for (int y=0; y<10; y++){
			for (int x=0; x<10; x++){
				printf("%s",grid[y][x].c_str());
			}
			printf("\n");
		}
		printf("\n\n\n");
		system("pause");
}

void PredatorPreyDomain::initializePredPreyEpisode(int steps){
	
	setUniqueRandomPredPreyPositions();

	deltaPredPrey.clear();
	deltaPredPrey = std::vector<std::vector<double> >(predators.size());

	
	for (int i=0; i<predators.size(); i++){
		deltaPredPrey[i] = std::vector<double>(steps,0.0);
		predators[i].caught=false;
	}
	for (int i=0; i<prey.size(); i++){
		prey[i].caught=false;
	}

	stepsToCapture=std::vector<int>(prey.size(),0);
	predStepsToCapture=std::vector<int>(predators.size(),0);
	captured=std::vector<bool>(prey.size(),false);
	allCaptured = false;
}



void PredatorPreyDomain::simulatePredPreyEpisode(std::vector<NeuralNet*> NNSet, std::vector<double> &predFitnesses, std::vector<double> &preyFitnesses){
//	int steps = 100;
	int steps = 100;
	stepLog.clear();
	stepLog = std::vector<std::vector<std::vector<double> > >(steps);

	initializePredPreyEpisode(steps);

	for (int i=0; i<steps; i++){
		//printf("Step %i\n",i);
		//showTerrain();

		for (int j=0; j<predators.size(); j++){
			if (!predators[j].caught){
				getPredatorState(predators[j]); // sets stateInputs
				predators[j].selectAction(NNSet[j]); // uses stateInputs previously set
			}
		}

		std::vector<std::vector<double> > preyActions(prey.size()); // deprecated
		setPreyActions(preyActions);
		movePredators();
		movePrey(preyActions);
		allCaptured = checkSystemCapture(i);
		if (allCaptured){
			//printf("All Captured!!\n\n\n");
			break;
		}
		
		stepLog[i] = std::vector<std::vector<double> >(predators.size());
		for (int j=0; j<predators.size(); j++){
			stepLog[i][j] = std::vector<double>(2);
			stepLog[i][j][0] = predators[j].x;
			stepLog[i][j][1] = predators[j].y;
		}
	}
	predFitnesses = getLocalPredReward();
	preyFitnesses = getLocalPreyReward();

}

std::vector<double> PredatorPreyDomain::getDifferenceReward(){
	printf("difference reward placeholder for predator prey domain");
	return std::vector<double>();
}

std::vector<double> PredatorPreyDomain::getLocalPreyReward(void){
	//printf("nonlearning prey");
	return std::vector<double>(prey.size(),0.0); // RANDOM MOTION ON PREY FOR NOW
}

double PredatorPreyDomain::getLocalReward(int me){
	// placeholder
	return 0.0;
}

double PredatorPreyDomain::getGlobalReward(){
	// Get distance between me and prey
	/*if (allcaptured){
		return -stepsToCapture;
	} else {
		double deltasum = 0.0;
		for (int i=0; i<deltaPredPrey.size(); i++){
			for (int j=0; j<deltaPredPrey[i].size(); j++){
				deltasum+=deltaPredPrey[i][j];
			}
		}
		return -deltasum;
	}*/
	//placeholder
	printf("placeholder only");
	return 0.0;
}


void PredatorPreyDomain::getPredatorState(Predator &p){
	double sightRange = 20.0;

	// collect domain information for Predator state
	// STATE ELEMENTS:
	// {oMe, dNp, oNp, dNN, oNN, [tNN]} = 5 + nTypes
	
	p.stateInputs[(int)Predator::stateElementNames::orientationMe] = p.orientation;
	
	// Nearest prey
	GridWorld::PairQueueAscending pq = sortedPreyDists(p.x,p.y);
	if (pq.top().first<sightRange){
		p.stateInputs[(int)Predator::stateElementNames::dxNearestPrey] = ((prey[pq.top().second].x-p.x)+sightRange)/(2.0*sightRange);
		p.stateInputs[(int)Predator::stateElementNames::dyNearestPrey] = ((prey[pq.top().second].y-p.y)+sightRange)/(2.0*sightRange);
	} else {
		p.stateInputs[(int)Predator::stateElementNames::dxNearestPrey] = 1.0; // defaults
		p.stateInputs[(int)Predator::stateElementNames::dyNearestPrey] = 1.0;
	}


	// Nearest neighbor
	// find the nearest neighbor (this is same for both predator and prey)
	pq = sortedPredatorDists(p.x,p.y);
	if (predators[pq.top().second].ID==p.ID){
		pq.pop();
	}
	// nearest neighbor now at top
	if (pq.top().first<sightRange){
		p.stateInputs[(int)Predator::stateElementNames::dxNearestNeighbor] = ((predators[pq.top().second].x-p.x)+sightRange)/(2.0*sightRange);
		p.stateInputs[(int)Predator::stateElementNames::dyNearestNeighbor] = ((predators[pq.top().second].y-p.y)+sightRange)/(2.0*sightRange);
	} else {
		p.stateInputs[(int)Predator::stateElementNames::dyNearestNeighbor] = 1.0; // defaults
		p.stateInputs[(int)Predator::stateElementNames::dyNearestNeighbor] = 1.0;
	}

	// Now activate the type on its own node
	if (params->usingTypes){
		p.stateInputs[(int)Predator::stateElementNames::numNonTypeElements+predators[pq.top().second].type]=1.0;
	}
}

State PredatorPreyDomain::getPreyState(int me, bool returnBlankState){
	double sightRange = 20.0;
	// collect domain information for state
	// STATE ELEMENTS:
	// {oMe, dNp, oNp} = 3
	enum stateElementNames {orientationMe,distanceNearestPredator,orientationNearestPredator,numNonTypeElements}; // we assume that the prey sees no types
	std::vector<double> stateElements(numNonTypeElements,0.0);
	
	if (!returnBlankState){
		stateElements[orientationMe] = predators[me].orientation;
	
	
		// Nearest prey
		GridWorld::PairQueueAscending pq = sortedPredatorDists(predators[me].x,predators[me].y);
		if (pq.top().first<sightRange){
			stateElements[distanceNearestPredator] = pq.top().first/sightRange;
			stateElements[orientationNearestPredator] = predators[pq.top().second].orientation; // orientation (scaled)
		} else {
			stateElements[distanceNearestPredator] = sightRange; // defaults
			stateElements[orientationNearestPredator] = 0.0; // defaults
		}
	}

	return State(stateElements);
}
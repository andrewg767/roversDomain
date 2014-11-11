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

	#pragma omp parallel for // TODO: verify parallelizability
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

	double dx = preyMoveDistCap*action[1]*cos(me.orientation*2.0*3.14);
	double dy = preyMoveDistCap*action[1]*sin(me.orientation*2.0*3.14);
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

	double xAdd = action[1]*cos(me.orientation*2.0*3.14);
	double yAdd = action[1]*sin(me.orientation*2.0*3.14);

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
	//	NeuroEvoParameters* NEParams = new NeuroEvoParameters(Predator::numNonTypeElements,2); // both start the same for now... FROM TESTING WITH DELAYED TYPE INTRODUCTION

	std::vector<std::vector<PredatorPreyDomain*> > allTrialDomainsForEpoch(NEParams->popSize*2); // hardcoded: 2k=100 neural networks... edit this later



	for (int i=0; i<allTrialDomainsForEpoch.size(); i++){
		allTrialDomainsForEpoch[i] = std::vector<PredatorPreyDomain*>(nTrials); // for every trial
		for (int j=0; j<allTrialDomainsForEpoch[i].size(); j++){
			allTrialDomainsForEpoch[i][j] = new PredatorPreyDomain(PPparams);
			allTrialDomainsForEpoch[i][j]->initializePredPreyRun();
		}
		// Make positions at beginning consistent...
		for (int j=0; j<nTrials; j++){
			for (int k=0; k<allTrialDomainsForEpoch[i][j]->predators.size(); k++){
				allTrialDomainsForEpoch[i][j]->predators[k].x = allTrialDomainsForEpoch[i][0]->predators[k].x;
				allTrialDomainsForEpoch[i][j]->predators[k].y = allTrialDomainsForEpoch[i][0]->predators[k].y;
			}
			for (int k=0; k<allTrialDomainsForEpoch[i][j]->prey.size(); k++){
				allTrialDomainsForEpoch[i][j]->prey[k].x = allTrialDomainsForEpoch[i][0]->prey[k].x;
				allTrialDomainsForEpoch[i][j]->prey[k].y = allTrialDomainsForEpoch[i][0]->prey[k].y;
			}
		}
	}

	std::vector<NeuroEvo*> NESet(PPparams->nPredators);
	for (int i=0; i<PPparams->nPredators; i++){
		NESet[i] = new NeuroEvo(NEParams);
	}


	for (int i=0; i<nEpochs; i++){
		//printf("Epoch %i\n",i);
		if (i%50==0) printf("(%i/%i)",i,nEpochs);
		else printf(".");
		simulatePredPreyEpoch(NESet,allTrialDomainsForEpoch);		
		//outputSteps();
		GLog[i] = getAverageBestNNScore(NESet);
	}

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
}

bool PredatorPreyDomain::checkCapture(Rover &p){
	std::set<std::pair<int,int> > capturePositions; // spaces a predator needs to occupy to capture the prey
	if (p.x-1>=0) capturePositions.insert(std::pair<int,int>(p.x-1,p.y));
	if (p.x+1<bounds.size('x')) capturePositions.insert(std::pair<int,int>(p.x+1,p.y));
	if (p.y-1>=0) capturePositions.insert(std::pair<int,int>(p.x,p.y-1));
	if (p.y+1<bounds.size('y')) capturePositions.insert(std::pair<int,int>(p.x,p.y+1));

	std::vector<int> catchList;
	for (int i=0; i<predators.size(); i++){
		if (capturePositions.count(std::pair<int,int>(floor(predators[i].x),floor(predators[i].y)))){
			capturePositions.erase(std::pair<int,int>(floor(predators[i].x),floor(predators[i].y)));
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
		grid[(int)floor(predators[p].y)][(int)floor(predators[p].x)]="d";
	}
	for (int y=0; y<prey.size(); y++){
		grid[(int)floor(prey[y].y)][(int)floor(prey[y].x)]="y";
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

Predator::Predator(bool usingTypes):Rover(){
	// fill with other things
	static int IDset = 0;
	ID = IDset++;

	moveDistCap=2.0;
	type = PredatorTypes(rand()%int(numTypes));

	stateInputs = std::vector<double>(numNonTypeElements + int(numTypes)*(usingTypes?1:0),0.0);
};

void Predator::selectAction(NeuralNet* NN){ // given a certain stateInputs (defined prior) get an action
	// Sets the values of actionToTake
	actionToTake = NN->predictContinuous(stateInputs); // gives [rotation dtheta][movement d]
	actionToTake[0] = 2.0*actionToTake[0]-1.0; // scale between [-1,1]


	//printf("NN out = %f,%f\n",actionToTake[0],actionToTake[1]);

	// Type implementation scales the output
	if (type==CCW){
		// scale the neural network orientation command between 0-pi
		//actionToTake[0]/=4.0; // HACK: can only turn 90deg max
		actionToTake[0]/=8.0;

		// Hack #2: immobile agent
		//actionToTake[1] = 0.0;
	} else if (type==CW){	
		// unreliable agent
		/*if (rand()%100<30){ // 30% of the time random
			actionToTake[0] = double(rand())/double(RAND_MAX)*2.0-1.0;
			actionToTake[1] = double(rand())/double(RAND_MAX);
		}*/
		if (rand()%100<80){ // 30% of the time random
			actionToTake[0] = double(rand())/double(RAND_MAX)*2.0-1.0;
			actionToTake[1] = double(rand())/double(RAND_MAX);
		}
	} else if (type== Fast){
		//actionToTake[1] *= 2.0; // multiply output action by 2
		actionToTake[1] *= 10.0;
	}

	actionToTake[1]*=moveDistCap;
}


PredatorPreyDomainParameters::PredatorPreyDomainParameters():
	usingTypes(true),
	nPredators(20),
	nPrey(10),
	rewardType(Global)
{
	fixedTypes= std::vector<Predator::PredatorTypes>(nPredators);
	for (int i=0; i<fixedTypes.size(); i++){
		// Creates an even distribution of predator types
		fixedTypes[i] = Predator::PredatorTypes(i%int(Predator::PredatorTypes::numTypes));
	}
};

void PredatorPreyDomain::randomizePredatorPositions(){
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

void PredatorPreyDomain::randomizePreyPositions(){
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


void PredatorPreyDomain::setPredatorActions(std::vector<NeuralNet*> NNSet){
	for (int i=0; i<predators.size(); i++){
		if (!predators[i].caught){
			getPredatorState(predators[i]);
			predators[i].selectAction(NNSet[i]); // uses stateInputs previously set
		}
	}
}


void PredatorPreyDomain::setPreyActions(std::vector<std::vector<double> > &preyActions){
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

void PredatorPreyDomain::movePredators(){
	for (int j=0; j<predators.size(); j++){
		if (!predators[j].caught) movePred(predators[j],predators[j].actionToTake); // previously set by selectAction; ONLY the predators not eating move
	}
}

void PredatorPreyDomain::movePrey(std::vector<std::vector<double> > &preyActions){
	for (int j=0; j<prey.size(); j++){
		if (!prey[j].caught) move(prey[j],preyActions[j]);
	}
}

bool PredatorPreyDomain::checkSystemCapture(int t){
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

void PredatorPreyDomain::setUniqueRandomPredPreyPositions(){
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
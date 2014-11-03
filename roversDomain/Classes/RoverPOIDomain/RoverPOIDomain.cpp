#include "RoverPOIDomain.h"


/*
RoverPOIDomain::RoverPOIDomain(void):nPOIs(10),teleportation(false)
{
}


RoverPOIDomain::~RoverPOIDomain(void)
{
}

void RoverPOIDomain::initializeRoverDomain(bool usingTypesSet, bool teleportationSet, std::string rewardSet, bool firstInitialization){
	//printf("Deprecated functions removed, needs rewrite.");
	// Initialize run constants
	usingTypes = usingTypesSet;
	teleportation = teleportationSet;
	rewardType = rewardSet;

	nStateElements = NSECTORS*NDISTANCES*(2+FAILTYPECOUNT*int(usingTypes)); // first two are COUNTS of ALL POIs/rovers, failtypecount/sector for rest
	
	// Place the POIs in the world
	if (firstInitialization){
		generatePOIs(); // generate POIs
		generateStaticPOIPositions(); // Place POIs uniquely+save positions
	}

	// Place the rovers in the world
	generateRovers(); // generate Rovers
	generateStaticRoverPositions(); // Place rovers in graph, store original places


//	initializeNeuralNetPopulation(); // initialize Neural Net (NOTE, HARDCODING IN THIS FUNCTION) // deprecated
}

std::vector<double> RoverPOIDomain::getDifferenceReward(){
	//If more than two robots visit a POI, only the observations of the closest two are considered,
	// and their visit distances are averaged in the computation of the system evaluation

	std::vector<double> D(rovers.size(),0.0); // D for all of the rovers
	typedef std::pair<double,int> P;

	for (int i=0; i<POIs.size(); i++){
		POI p = POIs[i];

		// Get all distances to POI
		PairQueueAscending q = sortedRoverDists(p.x,p.y);

		// Get top 3 closest (dij, dik, dil) in order closest to farthest
		double dij = q.top().first;
		double j = q.top().second;
		q.pop();
		double dik = q.top().first;
		double k = q.top().second;
		q.pop();
		double dil = q.top().first;
		double l = q.top().second;

		// BEGIN MODIFICATION
		double gatheredValjk, gatheredValjl,gatheredValkl; // modification
		double multiplej, multiplek,multiplel; // modification
		// average the multiples based on type...
		if (rovers[j].type==0){
			multiplej=0.25;
		} else if (rovers[j].type==2){
			multiplej=0.5;
		} else if (rovers[j].type==3){
			multiplej=0.75;
		}
		if (rovers[k].type==0){
			multiplek=0.25;
		} else if (rovers[k].type==2){
			multiplek=0.5;
		} else if (rovers[k].type==3){
			multiplek=0.75;
		}
		if (rovers[l].type==0){
			multiplel=0.25;
		} else if (rovers[l].type==2){
			multiplel=0.5;
		} else if (rovers[l].type==3){
			multiplel=0.75;
		}
		gatheredValjk = p.val*(multiplej+multiplek);
		gatheredValjl = p.val*(multiplej+multiplel);
		gatheredValkl = p.val*(multiplek+multiplel);
		// END MODIFICATION


		if (dil<deltaO){
			//D[unsigned(j)]+=2.0*p.val*(1.0/(dik+dij+2.0)-1.0/(dik+dil+2.0)); // original
			//D[unsigned(k)]+=2.0*p.val*(1.0/(dik+dij+2.0)-1.0/(dij+dil+2.0)); // original
			D[unsigned(j)]+=gatheredValjk/(dik+dij+2.0)-gatheredValkl/(dik+dil+2.0); // modified
		} else if ((dij<deltaO) && (dik<deltaO)){
			D[unsigned(j)]+=2.0*gatheredValjk/(dik+dij+2.0); // reduces to original
			D[unsigned(k)]+=2.0*gatheredValjk/(dik+dij+2.0);
		}
	}
	return D;
}


double RoverPOIDomain::getLocalReward(int me){
	// Returns Pj(z)
	double L = 0.0;
	for (int i=0; i<POIs.size(); i++){
		double deltaMe = gridDistance(POIs[i].x,POIs[i].y,rovers[me].x,rovers[me].y);
		if (deltaMe<deltaO){
			double gatheredVal = POIs[i].val/deltaMe;
			
			// BEGIN VALUE MODIFICATION*********
			if (rovers[me].type==0){
				gatheredVal*=0.25;
			} else if (rovers[me].type==2){
				gatheredVal*=0.5;
			} else if (rovers[me].type==3){
				gatheredVal*=0.75;
			}
			// END VALUE MODIFICATION************

			L+=gatheredVal;
		}
	}

	return L;
}

void RoverPOIDomain::simulateRunRoverDomain(){
	// Performs |maxSteps| number of epochs, consisting of some number of steps, for the rover domain.
	// Randomizes starting positions between the rovers. These are reset to be the same during a single epoch though. 
	int maxEpochs = 1000;
	for (int T=0; T<maxEpochs; T++){
		//printf("T=%i\n",T);
 		simulateEpochRoverDomain();
		generateStaticRoverPositions();
	}
}

double RoverPOIDomain::getGlobalReward(){
	//If more than two robots visit a POI, only the observations of the closest two are considered,
	// and their visit distances are averaged in the computation of the system evaluation
	rovers;

	double G=0.0;
	for (int i=0; i<POIs.size(); i++){
		POI p = POIs[i];

		// Get all distances to POI
		PairQueueAscending q = sortedRoverDists(p.x,p.y);

		double dij = q.top().first; // closest, DISTANCE
		int jID = q.top().second; // closest, ID
		q.pop();
		double dik = q.top().first; // second closest, DISTANCE
		int kID = q.top().second; // second closest, ID

		double Nij = 0.0;
		double Nik = 0.0;
		if (dij<deltaO){
			Nij=1.0;
		}
		if (dik<deltaO){
			Nik=1.0;
		}

		// BEGIN MODIFICATION
		double gatheredVal; // modification
		double multiplej, multiplek; // modification
		// average the multiples based on type...
		if (rovers[jID].type==0){
			multiplej=0.25;
		} else if (rovers[jID].type==2){
			multiplej=0.5;
		} else if (rovers[jID].type==3){
			multiplej=0.75;
		} else {
			multiplej=1.0;
		}
		if (rovers[kID].type==0){
			multiplek=0.25;
		} else if (rovers[kID].type==2){
			multiplek=0.5;
		} else if (rovers[kID].type==3){
			multiplek=0.75;
		} else {
			multiplek = 1.0;
		}
		// END MODIFICATION

		gatheredVal = POIs[i].val*(multiplej+multiplek);

		G += 2.0*(gatheredVal*Nij*Nik)/(dij+dik+2.0);
	}
	return G;
}

void RoverPOIDomain::simulateEpochRoverDomain(){
	if (!rovers.size()){
		printf("No rovers! Aborting.");
		exit(1);
	}

	double gAvg = 0;
	int gCount = 0;

	int steps;
	if(teleportation) steps=1;
	else steps = 10;

	double travelScale;
	if (teleportation) travelScale = TRAVELBOUND; // MOVE TO INITIALIZATION
	else travelScale = 1.0;  // MOVE TO INITIALIZATION

	logRoverPositions(); // logs the starting position
	
	for (int i=0; i<rovers.size(); i++){
		rovers[i].evo.generateNewMembers();
	}

	while(true){
		for (int i=0; i<steps; i++){
			std::vector<std::vector<double> > xyActions(rovers.size()); // list of xy coordinates to move to
			for (int j=0; j<rovers.size(); j++){
				// select xy-coordinates here... scale selection as percent of POSSIBLE distance
				xyActions[j] = rovers[j].selectNNActionMultiple(getState(j));
				// scale to domain
				xyActions[j][0]*=travelScale; // this is the DX ACTION
				xyActions[j][1]*=travelScale; // this is the DY ACTION
		
			}
			for (int j=0; j<rovers.size(); j++){
				rovers[j].walk(xyActions[j][0],xyActions[j][1],percentFail);
			}
			logRoverPositions();
		}

		
		// Calculate reward and update NOTE: THIS IS ONLY GLOBAL REWARD
		double epG = getGlobalReward();
		gAvg += epG;
		gCount++;

		std::vector<double> rewards(rovers.size(),0);
		if (!strcmp(rewardType.c_str(),"global")){
			rewards = std::vector<double>(rovers.size(),epG);
		} else if (!strcmp(rewardType.c_str(),"difference")){
			rewards = getDifferenceReward();
		} else if (!strcmp(rewardType.c_str(),"local")){
			for (int i=0; i<rovers.size(); i++){
				rewards[i] = getLocalReward(i);
			}
		}

		bool epochDone = false;
		for (int i=0; i<rovers.size(); i++){
			rovers[i].evo.updateMember(epG);
			if (!rovers[i].evo.selectNewMember()) epochDone = true;
		}
		if (epochDone){
			break;
			gAvg /= double(gCount);
		}

		resetForEpoch(); // reset the domain for the next epoch, as it was before
	}

	for (int i=0; i<rovers.size(); i++){
		rovers[i].evo.selectSurvivors();
	}
	performanceVals.push_back(gAvg);
}

void RoverPOIDomain::resetForEpoch(){
	// Replace rovers
	resetStaticRovers(); // later; make sure static positions are changed between epochs...
}

void RoverPOIDomain::generateStaticPOIPositions(){
	for (int i=0; i<POIs.size(); i++){
		staticPOIPositions.push_back(std::vector<double>(2,0.0));
		double randX = bounds.size('x')*double(rand())/double(RAND_MAX);
		double randY = bounds.size('y')*double(rand())/double(RAND_MAX);
		POIs[i].x = randX;
		POIs[i].y = randY;
		staticPOIPositions[i][0] = randX;
		staticPOIPositions[i][1] = randY;
	}
}

GridWorld::PairQueueAscending RoverPOIDomain::sortedPOIDists(double xref, double yref){
	typedef std::pair<double,int> P;
	std::vector<P> dists(POIs.size(),std::make_pair<double,int>(0.0,0));
	for (int j=0; j<POIs.size(); j++){
		dists[j]=P(gridDistance(xref,yref,POIs[j].x,POIs[j].y),j);
	}

	PairQueueAscending q(dists.begin(),dists.end());
	return q;
}

State RoverPOIDomain::getState(int me)
{
	// NEW sonar state... go by quadrant AND distance...
	// note here only quadrant is implemented

	// State elements/init
	std::vector<std::vector<double> > poisByQuadrantAndDistance(NSECTORS); // count POIs for each sector
	std::vector<std::vector<double> > roversByQuadrantAndDistance(NSECTORS); // count POIs for each sector
	std::vector<std::vector<std::vector<double> > > roverTypeCountByQuadrantAndDistance; // used if using types [QUADRANT][DIST][TYPE]
	std::vector<double> stateInputs;

	// Reserve space
	for (int i=0; i<NSECTORS;i++){
		poisByQuadrantAndDistance[i] = std::vector<double>(NDISTANCES,0.0);
		roversByQuadrantAndDistance[i] = std::vector<double>(NDISTANCES,0.0);
	}

	// Counting rovers by quadrant
	for (int i=0; i<POIs.size(); i++){
		std::pair<Direction,DistanceDivision> quadAndDist = relativePosition(rovers[me].x,rovers[me].y,POIs[i].x,POIs[i].y);
		int quadrant = int(quadAndDist.first);
		int dist = int(quadAndDist.second);
		poisByQuadrantAndDistance[quadrant][dist]++;
	}

	// Normalization
	for (int i=0; i<poisByQuadrantAndDistance.size(); i++){
		for (int j=0; j<poisByQuadrantAndDistance[i].size(); j++){
			poisByQuadrantAndDistance[i][j] /= POIs.size();
		}
	}
	for (int i=0; i<roversByQuadrantAndDistance.size(); i++){
		for (int j=0; j<roversByQuadrantAndDistance[i].size(); j++){
			roversByQuadrantAndDistance[i][j] /= rovers.size();
		}
	}

	// Stitch this together to form state...
	for (int i=0; i<poisByQuadrantAndDistance.size(); i++){
		stateInputs.insert(stateInputs.end(),poisByQuadrantAndDistance[i].begin(),poisByQuadrantAndDistance[i].end());
	}
	for (int i=0; i<roversByQuadrantAndDistance.size(); i++){
		stateInputs.insert(stateInputs.end(),roversByQuadrantAndDistance[i].begin(),roversByQuadrantAndDistance[i].end());
	}
	
	if (usingTypes){
		// Collect and add type information if necessary
		roverTypeCountByQuadrantAndDistance = std::vector<std::vector<std::vector<double> > >(NSECTORS);
		for (int i=0; i<NSECTORS; i++){
			roverTypeCountByQuadrantAndDistance[i] = std::vector<std::vector<double> >(NDISTANCES);
			for (int j=0; j<NDISTANCES; j++){
				roverTypeCountByQuadrantAndDistance[i][j] = std::vector<double>(FAILTYPECOUNT);
			}
		}
		// Type count
		for (int i=0; i<rovers.size(); i++){
			std::pair<Direction,DistanceDivision> quadAndDist = relativePosition(rovers[me].x,rovers[me].y,rovers[i].x,rovers[i].y);
			int quadrant = int(quadAndDist.first);
			int dist = int(quadAndDist.second);
			roversByQuadrantAndDistance[quadrant][dist]++;
			roverTypeCountByQuadrantAndDistance[quadrant][dist][rovers[i].type]++;
		}
		// Normalization
		for (int i=0; i<roverTypeCountByQuadrantAndDistance.size(); i++){
			for (int j=0; j<roverTypeCountByQuadrantAndDistance[i].size(); j++){
				if (roversByQuadrantAndDistance[i][j]==0) continue;
				else {
					for (int k=0; k<roverTypeCountByQuadrantAndDistance[i][j].size(); k++){
						roverTypeCountByQuadrantAndDistance[i][j][k] /= roversByQuadrantAndDistance[i][j];
					}
				}
			}
		}

		// Stitching
		for (int i=0; i<roverTypeCountByQuadrantAndDistance.size(); i++){
			for (int j=0; j<roverTypeCountByQuadrantAndDistance[i].size(); j++){
				stateInputs.insert(stateInputs.end(), roverTypeCountByQuadrantAndDistance[i][j].begin(), roverTypeCountByQuadrantAndDistance[i][j].end());
			}
		}
	}

	// Generate state from stitched inputs
	return State(stateInputs);
}

void RoverPOIDomain::generatePOIs(){
	// randomly generates POIs
	POIs.clear(); // clears out previous POIs
	for (int i=0; i<nPOIs; i++){
		POIs.push_back(POI());
	}
}
*/
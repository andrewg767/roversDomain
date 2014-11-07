#include "GridWorld.h"


GridWorld::GridWorld(void): nRovers(20),deltaO(10.0),percentFail(0.80){
	bounds = EnvironmentBounds();
}

void GridWorld::logRoverPositions(){
	for (int i=0; i<rovers.size(); i++){
		pathLog.push_back(std::vector<double>(2,0.0));
		pathLog.back().at(0) = rovers[i].x;
		pathLog.back().at(1) = rovers[i].y;
	}
}

void GridWorld::generateStaticRoverPositions(){
	staticRoverPositions.clear();
	for (int i=0; i<rovers.size(); i++){
		staticRoverPositions.push_back(std::vector<double>(2,0.0));
		double randX = bounds.size('x')*double(rand())/double(RAND_MAX);
		double randY = bounds.size('y')*double(rand())/double(RAND_MAX);
		rovers[i].x = randX;
		rovers[i].y = randY;
		staticRoverPositions[i][0] = randX;
		staticRoverPositions[i][1] = randY;

	}
}

std::pair<GridWorld::Direction,GridWorld::DistanceDivision> GridWorld::relativePosition(double x1, double y1, double x2, double y2){
	// Get the relative position of the second object to the first object

	double dist = gridDistance(x1,y1,x2,y2);
	DistanceDivision div;
	if (dist<CLOSEBOUND) div = CLOSE;
	else if (dist<MEDIUMBOUND) div = MEDIUM;
	else div = FAR;

	if (y1<y2){ // rover above me, quadrant 1-2
		if (x1<x2){ // rover right, quadrant 1
			return std::make_pair(Q1,div);
		} else {
			return std::make_pair(Q2,div); // quadrant 2
		}
	} else { // rover below me, quadrant 3-4
		if (x1<x2){ // rover right, quadrant 4
			return std::make_pair(Q4,div);
		} else {
			return std::make_pair(Q3,div);
		}
	}
}

void GridWorld::resetStaticRovers(){
	for (int i=0; i<staticRoverPositions.size(); i++){
		rovers[i].x = staticRoverPositions[i][0];
		rovers[i].y = staticRoverPositions[i][1];
	}
}

GridWorld::~GridWorld(void)
{
}

double GridWorld::gridDistance(double x1,double y1,double x2,double y2){
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

GridWorld::PairQueueAscending GridWorld::sortedRoverDists(double xref, double yref){
	typedef std::pair<double,int> P;
	std::vector<P> dists(rovers.size(),std::make_pair<double,int>(0.0,0));
	for (int j=0; j<rovers.size(); j++){
		dists[j]=P(gridDistance(xref,yref,rovers[j].x,rovers[j].y),j);
	}

	PairQueueAscending q(dists.begin(),dists.end());
	return q;
}




int GridWorld::nearestNeighbor(int roverID){
	// Get all distances to rovers
	PairQueueAscending q = sortedRoverDists(rovers[roverID].x,rovers[roverID].y);
	if (q.top().second==roverID) q.pop(); // remove top if it's me
	return q.top().second;
}


void GridWorld::roverRandomWalk(){
	for (int i=0; i<rovers.size(); i++){
		rovers[i].randWalk(percentFail);
	}
}

void GridWorld::generateRovers(){ // 'fill' template function might be nice to have
	rovers.clear(); // clears out any Rovers that might have been lrking
	for (int i=0; i<nRovers; i++){
		rovers.push_back(Rover());
	}
}

void GridWorld::randomizePositions(std::vector<Rover> &r){
	for (int i=0; i<r.size(); i++){
		r[i].x = rand()%bounds.size('x');
		r[i].y = rand()%bounds.size('y');
	}

	bool blocked = true;
	while (blocked){
		//printf("X");
		blocked = false;	
		for (int i=0; i<r.size()-1; i++){
			for (int j=i+1; j<r.size(); j++){
				if (!gridDistance(r[i],r[j])){
					r[j].x = rand()%bounds.size('x');
					r[j].y = rand()%bounds.size('y');
					blocked = true;
					break;
				}
			}
			if (blocked) break;
		}
	}
}


void GridWorld::exportGLog(std::string fileName){
	//  export a saved G value to a file
	std::ofstream myfile;
	myfile.open(fileName.c_str());
	for (int i=0; i<GLog.size(); i++){
		myfile << GLog[i] << "\n";
	}
	myfile.close();
}

void GridWorld::exportCSV2(std::vector<std::vector<double> > myvec, std::string fileName){
	// Export a 2-d csv file
	std::ofstream myfile;
	myfile.open(fileName.c_str());
	for (int i=0; i<myvec.size(); i++){
		for (int j=0; j<myvec[i].size(); j++){
			myfile << myvec[i][j] << ",";
		}
		myfile << "\n";
	}
	myfile.close();
}

double GridWorld::gridDistance(Rover &r1, Rover &r2){
	return gridDistance(r1.x,r1.y,r2.x,r2.y);
}

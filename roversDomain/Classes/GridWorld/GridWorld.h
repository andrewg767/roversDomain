#pragma once
#include "../EnvironmentBounds/EnvironmentBounds.h"
#include "../State/State.h"
#include "../Rover/Rover.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include "../NeuralNet/NeuralNet.h"
#include <queue>
#include <functional>
#include <utility>

#define TRAVELBOUND 10.0 // amount you can travel in any direction (scaling parameter for NN)
#define CLOSEBOUND 5.0 // distance away and still close (past this medium or far)
#define MEDIUMBOUND 10.0 // distance away and still medium (past this far)

class GridWorld{

public:
	int nRovers;
	double percentFail; // failure/type enactment percentage
	
	std::vector<double> performanceVals;
	
	// State definition
	enum Direction{Q1,Q2,Q3,Q4,NSECTORS};
	enum DistanceDivision{CLOSE,MEDIUM,FAR,NDISTANCES};

	// Sensor functions
	std::pair<Direction,DistanceDivision> relativePosition(double x1, double y1, double x2, double y2);

	void logRoverPositions();

	typedef std::pair<double,int> P;
	typedef std::priority_queue<P,std::vector<P>,std::greater<P> > PairQueueAscending;
	PairQueueAscending sortedRoverDists(double xref, double yref); // Rovers, sorted by distance away from reference point
	int nearestNeighbor(int roverID); // just get the nearest



	void generateStaticRoverPositions();

	std::vector<std::vector<double> > staticRoverPositions;

	void resetStaticRovers();


	double deltaO;
	double gridDistance(double x1, double y1, double x2, double y2);
	double gridDistance(Rover &r1, Rover &r2){
		return gridDistance(r1.x,r1.y,r2.x,r2.y);
	}
	
	EnvironmentBounds bounds;
	std::vector<Rover> rovers;
	std::vector<std::vector<char> > positionData; // deprecated?

	GridWorld();
	~GridWorld();
	
	// REWARD STRUCTURES
	virtual double getLocalReward(int me)=0;
	virtual double getGlobalReward()=0;
	virtual std::vector<double> getDifferenceReward()=0;

	
	// Walking functions
	void roverRandomWalk();

	// Generation functions
	void generateRovers();

	// IO
	void exportGLog(std::string fileName);
	void exportPositionLog(std::string fileName, int rover);
	void exportCSV2(std::vector<std::vector<double> > myvec, std::string fileName);

	// Terrain changes
	void resetPositionData();
	void reducePositionValues();

	std::vector<std::vector<double> > pathLog;
	std::vector<double> GLog;

	// Logging
	void logPosition(Rover r);

	// Reset
	void randomizePositions(std::vector<Rover> &r);
//	void randomizePositions(std::vector<POI> &p);
	void randomizeErrTypes(std::vector<Rover> &rovers);

	std::vector<std::vector<int> > startingPositions;
};


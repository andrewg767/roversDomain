#pragma once
#include "../GridWorld/GridWorld.h"
class RoverPOIDomain :
	public GridWorld
{
public:
	RoverPOIDomain(void);
	~RoverPOIDomain(void);

	int nPOIs;
	std::vector<POI> POIs;
	bool teleportation;
	void initializeRoverDomain(bool usingTypesSet, bool teleportationSet, std::string rewardSet, bool firstInitialization=true);
	void simulateRunRoverDomain();
	//void simulateEpisodeRoverDomain();
	void simulateEpochRoverDomain();

	// Rewards
	double getLocalReward(int me);
	double getGlobalReward();
	std::vector<double> getDifferenceReward();

	void resetForEpoch();

	
	//void generateStaticPOIPositions();
	//PairQueueAscending sortedPOIDists(double xref, double yref);
	State getState(int me);
	
	//void generatePOIs();
};
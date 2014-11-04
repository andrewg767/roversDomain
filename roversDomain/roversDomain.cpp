// roversDomain.cpp : Defines the entry point for the console application.
//
// Authors: Carrie Rebhuhn, Dr. Sam Devlin
//
// future: can add a risk map in order to extend to gliders

#include "stdafx.h"
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "Classes\GridWorld\GridWorld.h"
#include "Classes\NeuralNet\NeuralNet.h"
#include "..\..\master_libraries\easyio\easyio.h"
#include <time.h>
#include "Classes\PredatorPreyDomain\PredatorPreyDomain.h"

// for memory leak detection
/*
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
*/

using namespace std;

vector<vector<double> > importCSV2(string filename){
    ifstream file(filename.c_str());
    if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
    string value;
    vector<vector<double> > filematrix;

    while (file.good()){
        getline(file,value);
        istringstream iss(value);
        string word;
        vector<double> line;
        while (getline(iss,word,',')){
            line.push_back(atof(word.c_str()));
        }
        if (line.size()) filematrix.push_back(line);
    }
    file.close();
    return filematrix;
}

vector<double> importCSV1(string filename){
    ifstream file(filename.c_str());
    if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
    string value;
    vector<double> filematrix;

    while (file.good()){
        getline(file,value);
		filematrix.push_back(atof(value.c_str()));
    }
	filematrix.pop_back();
    file.close();
    return filematrix;
}

void printVector(vector<double> myvector, string fileName){
	ofstream myfile;
	myfile.open(fileName);
	for (int i=0; i<myvector.size(); i++){
		myfile<<myvector[i] <<"\n";
	}
	myfile.close();
}
/*
void MLExperiments(){
	GridWorld grid;

	int nRovers = 10;
	int nPOI = 5;
	int nSteps = 10;
	int nEpisodesLearn = 1000;
	int nEpisodesAct = 500;

	grid.roverLearn(nSteps,nEpisodesLearn); // learn a policy with 10 rovers on 5 poi taking 10 steps for 1000 episodes
	grid.exportGLog("roverG.txt");
		for (double pf=0.0; pf<1.0; pf=pf+0.1){
			grid.percentFail = pf;
		grid.roverAct(nSteps,nEpisodesAct); // Fixed policy,
	}
	
	grid.exportCSV2(grid.pathLog,"pathLog.txt");
}
*/
/*void mattCoevolution(){
	// Coevolution from Matt's paper

	GridWorld grid;
	grid.initialize(false,true,"global");
	
	grid.simulateRun();
	PrintOut::toFile(grid.performanceVals,"roverGJuly.txt");
	grid.performanceVals.clear();

	grid.initialize(true,true,"global",false); // sets rovers so they are in original game

	grid.simulateRun();
	PrintOut::toFile(grid.performanceVals,"roverGJulyTypes.txt");
}*/
/*
void mattCoevolutionWithTeleportation(){
	// Coevolution from matt's paper assuming you can teleport to POI
	//srand(time(NULL));
	int MAXRUNS = 10;
	int MAXEPOCHS = 100;
	bool teleportation = false;
	std::string rewardType = "global";
	std::vector<std::vector<double> > typePerformanceVals;
	std::vector<std::vector<double> > blindPerformanceVals;

	for (int run=0; run<MAXRUNS; run++){
		
		printf("run %i/%i\n",run+1,MAXRUNS);
		RoverPOIDomain domain;
		domain.initializeRoverDomain(false,teleportation,rewardType);


		domain.simulateRunRoverDomain();

		//PrintOut::toFile(grid.performanceVals,"blind.txt");
		//PrintOut::toFile(grid.pathLog,"blindpaths.csv");
		blindPerformanceVals.push_back(domain.performanceVals);
		domain.performanceVals.clear();

		domain.initializeRoverDomain(true,teleportation,rewardType,false);

		domain.simulateRunRoverDomain();
		//PrintOut::toFile(grid.performanceVals,"types.txt");
		//PrintOut::toFile(grid.pathLog,"typepaths.csv");

		typePerformanceVals.push_back(domain.performanceVals);
	}

	PrintOut::toFile(blindPerformanceVals,"blind.txt");
	PrintOut::toFile(typePerformanceVals,"types.txt");
}*/

void runSim(int nRuns, int nEpochs, int nTrials, PredatorPreyDomainParameters* domainParams){
	std::vector<std::vector<double> > runGLogs(nRuns);
	
	for (int run=0; run<nRuns; run++){
		clock_t tref = clock();
		printf("Run %i: ",run);
		simulatePredPreyRun(domainParams,nEpochs,nTrials, runGLogs[run]);
		double timediff = double(clock()-tref);
		printf(" runtime = %f mins, time/epoch= %f seconds\n\n",timediff/60.0, timediff/double(nEpochs));
	}

	if (domainParams->usingTypes){
		PrintOut::toFile(runGLogs,"predPreytypes.csv");
	} else {
		PrintOut::toFile(runGLogs,"predPreyNoTypes.csv");
	}
}


void predatorPrey(){
	// Evo settings
	int nRuns = 1;
	int nEpochs = 300;
	int nTrials = 20;

	//******** No types setting
	// Domain settings: uses default settings in PredatorPreyDomainParameters constructor
	PredatorPreyDomainParameters *domainParams = new PredatorPreyDomainParameters();
	domainParams->usingTypes = true;
	runSim(nRuns, nEpochs, nTrials, domainParams);
	
	//******* Types setting
	domainParams->usingTypes = false;
	runSim(nRuns, nEpochs, nTrials, domainParams);

	delete domainParams;
}

/*
void coevolutionWithTeleportation(){
	// Coevolution from matt's paper assuming you can teleport to POI
	//srand(time(NULL));
	int MAXRUNS = 10;
	int MAXEPOCHS = 100;
	bool teleportation = false;
	bool usingTypes = true;
	std::string rewardType = "global";
	std::string fileName;

	if (teleportation) fileName += "teleport-";
	else fileName+="noteleport-";
	fileName+=rewardType+"-";
	if (usingTypes) fileName += "types.csv";
	else fileName += "blind.csv";

	std::vector<std::vector<double> > pVals;

#pragma omp parallel for
	for (int run=0; run<MAXRUNS; run++){
		// INITIALIZATION/RESET
		printf("run %i/%i\n",run+1,MAXRUNS);
		RoverPOIDomain domain;
		domain.initializeRoverDomain(usingTypes,teleportation,rewardType);
		
		domain.simulateRunRoverDomain();
		pVals.push_back(domain.performanceVals);
	}

	PrintOut::toFile(pVals,fileName);
}
*/

/*
void nntest(){
	
	Predator P = Predator(50,false);
	std::vector<double> fitLog;
	for (int i=0; i<10000; i++){
		printf("Epoch %i\n",i);
		P.evo.generateNewMembers();

		bool epochNotEnded = true;
		int nTrials = 20;
		int nthNN=0;
		while(epochNotEnded){
			//printf("NN=%i\n",nthNN);
			std::vector<double> pFitness(nTrials);

			for (int t=0; t<nTrials; t++){
				double x = double(rand())/double(RAND_MAX);
				P.stateInputs = std::vector<double>(Predator::numNonTypeElements,0.0);
				P.stateInputs[0] = x;
//				printf("(input=%f,target=%f)\n",x,x*x);
				vector<double> actionToTake = P.evo.getOutput(P.stateInputs);
				pFitness[t] = -abs(actionToTake[0]-x*x);
				//system("pause");
			}

			double pFit=0.0;
			for (int i=0; i<nTrials; i++){
				pFit += pFitness[i]/double(nTrials);
			}

			P.evo.updateMember(pFit);
			epochNotEnded = P.evo.selectNewMember();
			nthNN++;
		}
		fitLog.push_back(P.evo.getBestMemberVal());

		P.evo.selectSurvivors();
	}
	PrintOut::toFile(fitLog,"fitTest.txt");
	
	std::vector<double> predicted;
	P.evo.setNNToBestMember();
	for (double x=0.0; x<1.0; x+=0.01){
		P.stateInputs = std::vector<double>(Predator::numNonTypeElements,0.0);
		P.stateInputs[0] = x;
		vector<double> actionToTake = P.evo.getOutput(P.stateInputs);
		predicted.push_back(actionToTake[0]);
	}
	PrintOut::toFile(predicted,"predicted.txt");

}
*/
int _tmain(int argc, _TCHAR* argv[])
{
	printf("Blah");
	//coevolutionWithTeleportation();
	predatorPrey();

	//_CrtDumpMemoryLeaks();
	system("pause");
	return 0;
}
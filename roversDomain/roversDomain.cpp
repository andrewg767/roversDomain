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

void runSim(int nRuns, int nEpochs, int nTrials, PredatorPreyDomainParameters* domainParams){
	std::vector<std::vector<double> > runGLogs(nRuns);
	
	for (int run=0; run<nRuns; run++){
		clock_t tref = clock();
		printf("Run %i: ",run);
		simulatePredPreyRun(domainParams,nEpochs,nTrials, runGLogs[run]);
		double timediff = double(clock()-tref);
		printf(" runtime = %f mins, time/epoch= %f seconds\n\n",timediff/(CLOCKS_PER_SEC*60.0), timediff/double(nEpochs*CLOCKS_PER_SEC));
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
	int nEpochs = 500;
	int nTrials = 5;

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

int _tmain(int argc, _TCHAR* argv[])
{
	predatorPrey();

	//_CrtDumpMemoryLeaks();
	system("pause");
	return 0;
}
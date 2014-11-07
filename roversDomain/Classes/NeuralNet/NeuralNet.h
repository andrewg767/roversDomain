#pragma once
#include <vector>
#include <iostream>
#include <chrono>
#include <random>
//#include "../../master_libraries/easyio/easyio.h" // Include for use of "NeuralNet::tester"

/*class NeuralNetParams{
	
};*/

class NeuralNet{
public:
	std::vector<std::vector<double> > O; // container for all outputs on way through neural network (may want to move this to memory later...)
	void  matrixMultiply(std::vector<double> &A, std::vector<std::vector<double> > &B, std::vector<double> &C);
	NeuralNet(){};
	~NeuralNet(){};
	double evaluation;
	void mutate(){
		double mutStd = 0.5;
		double mutationRate = 0.5;
		for (int i=0; i<Wbar.size(); i++){
			for (int j=0; j<Wbar[i].size(); j++){
		#pragma parallel omp for
				for (int k=0; k<Wbar[i][j].size(); k++){
					if (double(rand())/double(RAND_MAX)<mutationRate){
						// reset one of the weights
						double fan_in = double(Wbar[i].size());
						std::default_random_engine generator;
						generator.seed(time(NULL));
						std::normal_distribution<double> distribution(0.0,mutStd);
						Wbar[i][j][k] += distribution(generator);
					}
				}
			}
		}
	}

	void addInputs(int nToAdd);

	NeuralNet(int nInput, int nHidden, int nOutput, double gamma=0.9);
	NeuralNet(std::vector<int> &nodes, double gamma=0.9);
	void train(std::vector<std::vector<double> > &O, std::vector<std::vector<double> > &T, double epsilon=0.0, int iterations=0);
	std::vector<double> predictBinary(std::vector<double> o);
	std::vector<double> predictContinuous(std::vector<double> o);
	std::vector<std::vector<double> > batchPredictBinary(std::vector<std::vector<double> > &O);
	std::vector<std::vector<double> > batchPredictContinuous(std::vector<std::vector<double> > &O);
	//void tester(std::string trainfile="train.csv", std::string testfile="test.csv", std::string predictfile="predictions.csv"); // Include for testing net
private:
	double gamma_;
	std::vector<int> nodes_; // number of nodes at each layer of the network
	std::vector<std::vector<std::vector<double> > > W; // weights, W[interface][input][hidden(next unit)]. Without bias
	std::vector<std::vector<std::vector<double> > > Wbar; // weights with bias;

	double sum(std::vector<double> &myVector);
	double SSE(std::vector<double> &myVector);
	int connections();
	double backProp(std::vector<double> &o, std::vector<double> &t);
	void feedForward(std::vector<double> &o, std::vector<std::vector<double> > &Ohat, std::vector<std::vector<std::vector<double> > > &D);
	std::vector<std::vector<double> > matrixMultiply(std::vector<std::vector<double> > &A, std::vector<std::vector<double> > &B);
	std::vector<std::vector<double> > matrixMultiply(std::vector<double> &A, std::vector<double> &B);
	std::vector<double> matrixMultiply(std::vector<std::vector<double> > &A, std::vector<double> &B);
	std::vector<double> matrixMultiply(std::vector<double> &A, std::vector<std::vector<double> > &B);
	void sigmoid(std::vector<double> &myVector);
	void cmp_int_fatal(int a, int b);
};
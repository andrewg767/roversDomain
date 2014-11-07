#pragma once
#include <vector>
#include <iostream>
#include <chrono>
#include <random>

class NeuralNet{
public:
	std::vector<std::vector<double> > O; // container for all outputs on way through neural network (may want to move this to memory later...)
	void  matrixMultiply(std::vector<double> &A, std::vector<std::vector<double> > &B, std::vector<double> &C);
	NeuralNet(){};
	~NeuralNet(){};
	double evaluation;
	void mutate();

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
#pragma once
#include "NeuralNet.h"

using namespace std;

NeuralNet::NeuralNet(int nInputs, int nHidden, int nOutputs, double gamma){
	std::vector<int> nodes(3);
	nodes[0] = nInputs;
	nodes[1] = nHidden;
	nodes[2] = nOutputs;

	nodes_ = nodes; // note, this should NOT include the bias node
	gamma_ = gamma;

	Wbar = vector<vector<vector<double> > >(connections());
	W = vector<vector<vector<double> > >(connections());
	for (int connection=0; connection<connections(); connection++){ // number of layers
		int above = connection;
		int below = connection+1;

		// Populate Wbar with small random weights, including bias
		Wbar[connection] = (vector<vector<double> >(nodes_[above]+1));
		for (int i=0; i<nodes_[above]+1; i++){ // above+1 include bias;
			Wbar[connection][i] = vector<double>(nodes_[below]); // reserve memory for the connections below
			for (int j=0; j<nodes_[below]; j++){
				double fan_in = nodes_[above]+1.0;
				double rand_neg1to1 = ((double(rand())/double(RAND_MAX))*2.0-1.0)*0.1;
				Wbar[connection][i][j] = rand_neg1to1/sqrt(fan_in);
			}
		}

		W[connection] = Wbar[connection];
		W[connection].pop_back(); // remove extra bias weights
	};

	O = std::vector<std::vector<double> >(connections());
	for (int connection=0; connection<connections(); connection++){
		O[connection] = std::vector<double>(Wbar[connection][0].size(),0.0);
		if (connection+1!=connections()){ // if not the output layer
 			O[connection].push_back(1.0);
		}
	}
	evaluation = 0.0;
}


void NeuralNet::addInputs(int nToAdd){
	nodes_[0] += nToAdd;

	for (int i=0; i<nToAdd; i++){ // add new connections leading to each of the lower nodes
		Wbar[0].push_back(std::vector<double>(nodes_[1])); // adding another connection, each new one leads to nodes below
		for (int j=0; j<nodes_[1]; j++){
			double fan_in = nodes_[0]+1.0;
			double rand_neg1to1 = (double(rand())/double(RAND_MAX))*2.0-1.0;
			Wbar[0].back()[j]=rand_neg1to1/sqrt(fan_in);
		}
	}
	W[0] = Wbar[0];
	W[0].pop_back();

	O = std::vector<std::vector<double> >(connections());
	for (int connection=0; connection<connections(); connection++){
		O[connection] = std::vector<double>(Wbar[connection][0].size(),0.0);
		if (connection+1!=connections()){ // if not the output layer
 			O[connection].push_back(1.0);
		}
	}
}

NeuralNet::NeuralNet(vector<int> &nodes, double gamma){
	evaluation = 0.0;
	nodes_ = nodes; // note, this should NOT include the bias node
	gamma_ = gamma;

	Wbar = vector<vector<vector<double> > >(connections());
	W = vector<vector<vector<double> > >(connections());
	for (int connection=0; connection<connections(); connection++){ // number of layers
		int above = connection;
		int below = connection+1;

		// Populate Wbar with small random weights, including bias
		Wbar[connection] = (vector<vector<double> >(nodes_[above]+1));
		for (int i=0; i<nodes_[above]+1; i++){ // above+1 include bias;
			Wbar[connection][i] = vector<double>(nodes_[below]); // reserve memory for the connections below
			for (int j=0; j<nodes_[below]; j++){
				double fan_in = nodes_[above]+1.0;
				double rand_neg1to1 = (double(rand())/double(RAND_MAX))*2.0-1.0;
				Wbar[connection][i][j] = rand_neg1to1/sqrt(fan_in);
			}
		}

		W[connection] = Wbar[connection];
		W[connection].pop_back(); // remove extra bias weights
	};

	O = std::vector<std::vector<double> >(connections());
	for (int connection=0; connection<connections(); connection++){
		O[connection] = std::vector<double>(Wbar[connection][0].size());
	}
}

void NeuralNet::train(vector<vector<double> > &O, vector<vector<double> > &T, double epsilon, int iterations){
	double err = 2*epsilon+1.0; // just ensure it's bigger always to begin...
	int step = 0;
	if (iterations==0){
		while(err>=epsilon){
			vector<double> errs;
			for (int i=0; i<O.size();  i++){
				errs.push_back(backProp(O[i],T[i]));
			}
			err = sum(errs);
			printf("Err=%f\n",err);
		}
	} else {
		while(err>=epsilon && iterations>=step){
			vector<double> errs;
			for (int i=0; i<O.size();  i++){
				errs.push_back(backProp(O[i],T[i]));
			}
			err = sum(errs);
			printf("Err=%f\n",err);
			step++;
		}
	}
}

vector<double> NeuralNet::predictBinary(vector<double> o){
	for (int connection=0; connection<connections(); connection++){
		//printf("size(Wbar[connection])=%i",Wbar[connection].size());
		o.push_back(1.0); // add 1 for bias
		//printf("PREDICT: Asize=%i,Bsize=%i",o.size(),Wbar[connection].size());
		o = matrixMultiply(o,Wbar[connection]);
		sigmoid(o); // Compute outputs
	}
	return o;
}

vector<double> NeuralNet::predictContinuous(vector<double> o){
	
	o.push_back(1.0);
	matrixMultiply(o,Wbar[0],O[0]);
	sigmoid(O[0]);

	for (int connection=1; connection<connections(); connection++){
		O[connection-1].back() = 1.0; // static size allocation... last element is set to 1.0, bias (may not need to?)
		matrixMultiply(O[connection-1],Wbar[connection],O[connection]);
		sigmoid(O[connection]);
	}

	return O.back();

}

vector<vector<double> > NeuralNet::batchPredictBinary(vector<vector<double> > &O){
	vector<vector<double> > out;
	for (int i=0; i<O.size(); i++){
		out.push_back(predictBinary(O[i]));
	}
	return out;
}

vector<vector<double> > NeuralNet::batchPredictContinuous(vector<vector<double> > &O){
	vector<vector<double> > out;
	for (int i=0; i<O.size(); i++){
		out.push_back(predictContinuous(O[i]));
	}
	return out;
}

double NeuralNet::sum(vector<double> &myVector){
	double mySum = 0.0;
	for (int i=0; i<myVector.size(); i++){
		mySum+=myVector[i];
	}
	return mySum;
}

double NeuralNet::SSE(vector<double> &myVector){
	double err = 0.0;
	for (int i=0; i<myVector.size(); i++){
		err += myVector[i]*myVector[i];
	}
	return err;
}

int NeuralNet::connections(){
	return nodes_.size()-1;
}

double NeuralNet::backProp(vector<double> &o, vector<double> &t){
	// 'o' is the input vector, 't' is the 'target vector'
	// returns the SSE for the output vector

	vector<vector<double> > Ohat; // outputs with bias
	vector<vector<vector<double> > > D; // stored derivatives
	// Go through network "feed forward" computation

	feedForward(o,Ohat,D);

	vector<double> e(Ohat.back().size()-1,0.0); // "stored derivatives of the quadratic deviations"
	for (int i=0; i<Ohat.back().size()-1; i++){
		e[i] = (Ohat.back()[i]-t[i]);
	}

	// Hidden/output layer delta calcs
	vector<vector<double> > delta;
	for (int i=0; i<connections(); i++){
		delta.push_back(vector<double>());
	}
	delta.back() = matrixMultiply(D.back(),e); // output layer delta

	for (int connection=connections()-2; connection>=0; connection--){ // back propagation
		delta[connection]=matrixMultiply(matrixMultiply(D[connection],W[connection+1]),delta[connection+1]);
	}

	// Corrections to weights
	for (int connection=0; connection<connections(); connection++){
		vector<vector<double> > DeltaWbarT = matrixMultiply(delta[connection],Ohat[connection]);
		for (int i=0; i<Wbar[connection].size(); i++){
			for (int j=0; j<Wbar[connection][i].size(); j++){
				Wbar[connection][i][j] -= gamma_*DeltaWbarT[j][i]; // ji because it's transpose :)
			}
		}

		W[connection] = Wbar[connection];
		W[connection].pop_back();
	}

	// Calculate SS
	return SSE(e);
}

void NeuralNet::feedForward(vector<double> &o, vector<vector<double> > &Ohat, vector<vector<vector<double> > > &D){
	Ohat.push_back(o);
	Ohat.back().push_back(1.0); // add 1 for bias

	for (int connection=0; connection<connections(); connection++){
		Ohat.push_back(matrixMultiply(Ohat[connection],Wbar[connection])); // Compute outputs
		sigmoid(Ohat.back());

		// D stuff
		D.push_back(vector<vector<double> >());
		int k = Ohat.back().size(); // number of hidden/output units, excluding bias
		for (int i=0; i<k; i++){ // add for last entry
			// For last output given, calculate the derivatives
			double Oi = Ohat.back().at(i);
			D[connection].push_back(vector<double>(k,0.0));
			D[connection][i][i] = Oi*(1-Oi); // create a diagonal matrix
		}

		Ohat.back().push_back(1.0);
	}
};

vector<vector<double> > NeuralNet::matrixMultiply(vector<vector<double> > &A, vector<vector<double> > &B){
	// returns a size(A,1)xsize(B,2) matrix
	//printf("mm");
	cmp_int_fatal(A[0].size(), B.size());

	vector<vector<double> > C(A.size());
	for (int row=0;	row<A.size(); row++){
		C[row] = vector<double>(B[0].size(),0.0);
		for (int col=0; col<B[0].size(); col++){
			for (int inner=0; inner<B.size(); inner++){
				C[row][col] += A[row][inner]*B[inner][col];
			}
		}
	}

	return C;
};

vector<vector<double> > NeuralNet::matrixMultiply(vector<double> &A, vector<double> &B){
	// returns a A.size()xB.size() matrix

	vector<vector<double> > C(A.size());
	for (int row=0;	row<A.size(); row++){
		C[row] = vector<double>(B.size(),0.0);
		for (int col=0; col<B.size(); col++){
			C[row][col] += A[row]*B[col];
		}
	}

	return C;
};

vector<double> NeuralNet::matrixMultiply(vector<vector<double> > &A, vector<double> &B){
	// returns a size(A,1)x1 matrix
	// assumes B is a COLUMN vector
	//printf("mm1");
	cmp_int_fatal(A[0].size(),B.size());

	vector<double> C(A.size(),0.0);
	for (int row=0;	row<A.size(); row++){
		for (int inner=0; inner<B.size(); inner++){
			C[row] += A[row][inner]*B[inner];
		}
	}

	return C;
};

vector<double> NeuralNet::matrixMultiply(vector<double> &A, vector<vector<double> > &B){
	// Use this if expecting to get a vector back;
	// assumes A is a ROW vector (1xcols)
	// returns a 1xsize(B,2) matrix

	//printf("Asize=%i,Bsize=%i",A.size(),B.size());
	//fprintf("mm2");
	cmp_int_fatal(A.size(),B.size());

	// MODIFY TO MATCH1
	vector<double> C(B[0].size(),0.0);
	for (int col=0; col<B[0].size(); col++){
		for (int inner=0; inner<B.size(); inner++){
			C[col] += A[inner]*B[inner][col];
		}
	}

	return C;
};

void  NeuralNet::matrixMultiply(vector<double> &A, vector<vector<double> > &B, vector<double> &C){
	// This fills C up to the size of B[0].size(). C is allowed to be larger by 1, to accommodate bias
	// Use this if expecting to get a vector back;
	// assumes A is a ROW vector (1xcols)
	// returns a 1xsize(B,2) matrix

	cmp_int_fatal(A.size(),B.size());
	if (B[0].size()!=C.size() && B[0].size()!=C.size()-1){
		printf("B and C sizes don't match. pausing");
		system("pause");
	}

	// MODIFY TO MATCH1
	for (int col=0; col<B[0].size(); col++){
		C[col] = 0.0;
		for (int inner=0; inner<B.size(); inner++){
			C[col] += A[inner]*B[inner][col];
		}
	}
};

void NeuralNet::sigmoid(vector<double> &myVector){
	for (int i=0; i<myVector.size(); i++){
		myVector[i] = 1/(1+exp(-myVector[i]));
	}
}

void NeuralNet::cmp_int_fatal(int a, int b){
	if (a!=b){
		printf("Ints do not match! Pausing to debug then exiting.");
		system("pause");
		exit(1);
	}
}
#pragma once
#include <vector>
#include <map>
#include "State.h"
#include <algorithm>

#define NACTIONS 9

typedef double Value;

class QTable{
public:
	QTable(){};
	double alpha, gamma, init;
	QTable(double initSet, double alphaSet, double gammaSet){
		init = initSet;
		alpha = alphaSet;
		gamma = gammaSet;
	}

	typedef std::map<Action,Value> AVmap; // Internal "action value" map in Q-table
	typedef std::map<State,AVmap, State::cmpState> Qmap; // State to action-value map, with defined state comparator
	Qmap Qtable;
	Action softMax(State s, double tau){
		// NOTE, THIS COULD BE FASTER

		double sum=0.0;
		for (AVmap::iterator av=Qtable[s].begin(); av!=Qtable[s].end(); av++){
			sum+=exp(av->second/tau);
		}
		double compsum=0.0;
		double c = coin();
		for (AVmap::iterator av=Qtable[s].begin(); av!=Qtable[s].end(); av++){
			compsum+=exp(av->second/tau);
			if (compsum>=c) return av->first; 
		}
	}

	double coin(){
		return double(rand())/double(RAND_MAX);
	}

	void update(State s, Action a, Value R, State sNew){
		double Qmax;
		if (Qtable.empty()) Qmax = init;
		else Qmax =maxQ(sNew);

		Qtable[s][a] = Qtable[s][a]+alpha*(R+gamma*Qmax-Qtable[s][a]);
	}

	struct lessAVval {
		template <typename Lhs, typename Rhs>
		bool operator()(const Lhs& lhs, const Rhs& rhs) const{
			return lhs.second < rhs.second;
		}
	};
	
	AVmap::iterator maxAV(State s){
		AVmap::iterator maxElement = std::max_element(Qtable[s].begin(),Qtable[s].end(), lessAVval());
		return maxElement;
	}

	bool stateExists(State s){
		if (Qtable.empty()) return false;
		else return Qtable.find(s) != Qtable.end();
	}

	double maxQ(State s){
		AVmap::iterator maxElement;
		if (stateExists(s)){
			maxElement = maxAV(s);
			if (maxElement->second>init){
				return maxElement->second;
			}
		}else return init; // NOTE MAYBE WRONG IF STATE FULLY EXPLORED, EDGE CASE
	}

	Action maxA(State s){
		return maxAV(s)->first;
	}
};

class QAgent{
public:
	double epsilon;
	~QAgent();
	QAgent(){};
	QAgent(double initSet, double alphaSet, double gammaSet){
		epsilon = 0.1;
		Q = QTable(initSet, alphaSet, gammaSet);
	}
	void update(State s, Action a, Value R, State sNew){
		Q.update(s,a,R,sNew);
	}

	Action eGreedy(State s){
		double coin = double(rand())/double(RAND_MAX);
		if (coin<epsilon || Q.stateExists(s)==false) return rand()%NACTIONS;
		else return Q.maxA(s);
	}

	Action softMax(State s){

	}

private:
	QTable Q;
};
#include "Rover.h"


Rover::Rover(){
	static int IDset = 0;
	ID = IDset;
	IDset++;
	bounds = EnvironmentBounds(); // Use a default
	x = rand()%bounds.size('x'); // Random world placement
	y = rand()%bounds.size('y');

	orientation = double(rand())/double(RAND_MAX);

}


Rover::~Rover(void)
{
}


int Rover::max_ind(std::vector<double> myvector){
	return distance(myvector.begin(),std::max_element(myvector.begin(),myvector.end()));
}

void Rover::boundPosition(){
	// bound x/y ... note, should this be done on gridworld level?
	if (x<0) x=0;
	if (x>bounds.size('x')) x= bounds.size('x')-1;
	if (y<0) y=0;
	if (y>bounds.size('y')) y= bounds.size('y')-1;
}


void Rover::walk(double dx, double dy, double percentFail){
	// Normal walking behavior
	x+=dx;
	y+=dy;
	boundPosition();
}

double Rover::coin(){
	return double(rand())/double(RAND_MAX);
}

void Rover::randWalk(double percentFail){
	walk(rand()%2,rand()%2,percentFail); // take 0-1 steps in any direction
}
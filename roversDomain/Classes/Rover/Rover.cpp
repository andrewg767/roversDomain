#include "Rover.h"


Rover::Rover(){
//	A = QAgent(0.0,0.1,0.8); // not using Qagent for now...
	static int IDset = 0;
	ID = IDset;
	IDset++;
	bounds = EnvironmentBounds(); // Use a default
	x = rand()%bounds.size('x'); // Random world placement
	y = rand()%bounds.size('y');

//	type = NOMINAL; // STATIC FOR NOW
	orientation = double(rand())/double(RAND_MAX);
	
}


Rover::~Rover(void)
{
}

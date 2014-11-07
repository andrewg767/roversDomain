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

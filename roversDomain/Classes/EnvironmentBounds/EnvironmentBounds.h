#pragma once
#include <cstdio>
#include <cstdlib>

class EnvironmentBounds{
	/* This class contains the default values for the environment.
	* It should be created using the default constructor and accessed as
	* necessary by world classes.
	*/
public:
	EnvironmentBounds();
	~EnvironmentBounds();
	int size(char dim){
		if (dim=='x') return xBound;
		else if (dim=='y') return yBound;
		else {
			printf("Unknown dimension found in call EnvironmentBounds.size('%c'). Aborting.",dim);
			exit(-1);
			return -1;
		}
	}

	void cap(double &val, char dim){
		if (dim=='x'){
			if (val>=xBound) val = double(xBound-1);
			else if (val<0) val=0.0;
		} else if (dim=='y'){
			if (val>=yBound) val = double(yBound-1);
			else if (val<0) val=0.0;
		} else {
			printf("Unknown dimension found in call EnvironmentBounds.cap(%f,'%c'). Aborting.",val, dim);
			exit(-1);
			return;
		}

	}
private:
	int xBound, yBound;
};


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
	int size(char dim);
	void cap(double &val, char dim);

private:
	int xBound, yBound;
};


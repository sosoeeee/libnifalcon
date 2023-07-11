//////////////////////////////////////////////////////////
// Novint Falcon Kinematics/Dynamics based on R.E.Stamper's PhD(1997)
// with some modifications
//
// Using LibniFalcon Beta 4
//
// Alastair Barrow 26/08/09


#include <iostream>
#include <string>
#include <cmath>
#include <csignal>

#include "controller.h"

// #include "ros/ros.h"

using namespace std;

//////////////////////////////////////////////////////////
/// Ask libnifalcon to get the Falcon ready for action
/// nothing clever here, straight from the examples

void sigproc(int i)
{
	std::cout << "closing falcon and quitting" << std::endl;
	exit(0);
}


int main(int argc, char* argv[])
{
	signal(SIGINT, sigproc);

     FalconController falcon;

	if(!falcon.initialise())
		return 0;

	while (!falcon.calibrateDevice()){}

     falcon.run();

	return 0;
}


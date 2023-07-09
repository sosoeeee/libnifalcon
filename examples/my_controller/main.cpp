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

#include "controller.h"

using namespace std;

//////////////////////////////////////////////////////////
/// Ask libnifalcon to get the Falcon ready for action
/// nothing clever here, straight from the examples

int main(int argc, char* argv[])
{
     FalconController falcon;

	if(!falcon.initialise())
		return 0;

	while (!falcon.calibrateDevice()){}

     while (true)
     {
          falcon.run();
     }

	return 0;
}


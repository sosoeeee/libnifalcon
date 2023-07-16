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
#include <pthread.h>

#include "controller.h"

// #include "ros/ros.h"

using namespace std;

// global variables
pthread_mutex_t FalconController::mutex_ = PTHREAD_MUTEX_INITIALIZER;

void sigproc(int i)
{
	std::cout << "closing falcon and quitting" << std::endl;
	exit(0); // exit process, which means all threads are cleaned up
}


int main(int argc, char* argv[])
{
	signal(SIGINT, sigproc);

     FalconController falcon;

	if(!falcon.initialise())
		return 0;

	while (!falcon.calibrateDevice()){}
	
	pthread_t runThread;
	if (pthread_create(&runThread, NULL, FalconController::runThread, &falcon)!=0)
	{
		std::cout << "Failed to create runThread" << std::endl;
		return 0;
	}else
	{
		std::cout << "Created runThread" << std::endl;
	}

	pthread_t updateThread;
	if (pthread_create(&updateThread, NULL, FalconController::updateThread, &falcon)!=0)
	{
		std::cout << "Failed to create updateThread" << std::endl;
		return 0;
	}else
	{
		std::cout << "Created updateThread" << std::endl;
	}

	pthread_join(runThread, NULL);
	pthread_join(updateThread, NULL);
}


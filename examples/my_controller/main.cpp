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

#include "ros/ros.h"
#include "std_msgs/String.h"

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

	ros::init(argc, argv, "falcon_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::Publisher controllerPub = n.advertise<std_msgs::String>("controllerSignal", 1); // 1 is the buffer size, means only the latest message is kept

	while (ros::ok())
	{
		std_msgs::String msg;
		std::array<double, 3> pos = falcon.getPosition();
		std::stringstream ss;
		
		if(falcon.getGripState(1))
		{
			ss << pos[0] << "," << pos[1] << "," << pos[2];
		}
		else
		{
			ss << "0,0,0";
		}
		msg.data = ss.str();		
		controllerPub.publish(msg);

		loop_rate.sleep();
	}

	// if(falcon->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::CENTER_BUTTON)

	pthread_join(runThread, NULL);
	pthread_join(updateThread, NULL);
}


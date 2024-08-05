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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;

// global variables
pthread_mutex_t FalconController::mutex_ = PTHREAD_MUTEX_INITIALIZER;

void sigproc(int i)
{
	std::cout << "closing falcon and quitting" << std::endl;
	exit(0); // exit process, which means all threads are cleaned up
}

class FalconControllerNode : public rclcpp::Node
{
public:
    FalconControllerNode(std::shared_ptr<FalconController> falcon)	
        : Node("falcon_controller"), loop_rate_(100)
    {
        controller_pub_ = this->create_publisher<std_msgs::msg::String>("stickSignal", 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / loop_rate_), std::bind(&FalconControllerNode::timer_callback, this));
		falcon_ = falcon;
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String msg;
        std::array<double, 3> pos = falcon_->getPosition();
        std::array<double, 3> force = falcon_->getForce();
        std::stringstream ss;

        if (falcon_->getGripState(1))
        {
            ss << pos[0] << "," << pos[1] << "," << pos[2] << "," << force[0] << "," << force[1] << "," << force[2];
        }
        else
        {
            ss << "0,0,0,0,0,0";
        }
        msg.data = ss.str();
        controller_pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Rate loop_rate_;
	std::shared_ptr<FalconController> falcon_;
};

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

	rclcpp::init(argc, argv);
	auto node = std::make_shared<FalconControllerNode>(std::make_shared<FalconController>(falcon));
	rclcpp::spin(node);
	rclcpp::shutdown();

	// if(falcon->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::CENTER_BUTTON)

	pthread_join(runThread, NULL);
	pthread_join(updateThread, NULL);
}


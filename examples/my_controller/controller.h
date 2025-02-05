#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "falcon/core/FalconDevice.h"

class FalconController
{
private:
     std::unique_ptr<libnifalcon::FalconDevice> m_falconDevice;
     bool m_displayCalibrationMessage;

     bool ctlReady;
     bool sampleReady;

     std::array<double, 3> currentPos;
     std::array<double, 3> lastPos;
     std::array<double, 3> currentVel;
     std::array<double, 3> currentForce;

     std::array<double, 3> centerPos;
     std::array<double, 3> errorToCenter;
     std::array<double, 3> errorSum;

     struct timespec startT, endT;

     float stiffness;    // P
     float damping;      // D
     float integral;     // I

     void updateState();
     void computeForce();

     static pthread_mutex_t mutex_; // mutex for the thread
     
public:
     FalconController();
     FalconController(float stiffness, float damping);
     ~FalconController();
     bool initialise();
     bool calibrateDevice();
     std::array<double, 3> getPosition();
     std::array<double, 3> getForce();
     unsigned int getGripState(int index); // 1: center button, 2: forward button, 3: minus button, 4: plus button
     void update(); // update the state of the controller, and compute the force
     void run(); // set the force to the falcon

     // thread
     static void* updateThread(void* arg);
     static void* runThread(void* arg);
};

#endif
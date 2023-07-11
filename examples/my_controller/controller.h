#ifndef CONTROLLER_H
#define CONTROLLER_H_H

#include "falcon/core/FalconDevice.h"
#include "falcon/gmtl/gmtl.h"

class FalconController
{
private:
     std::unique_ptr<libnifalcon::FalconDevice> m_falconDevice;
     bool m_displayCalibrationMessage;

     bool ctlReady;

     std::array<double, 3> currentPos;
     std::array<double, 3> lastPos;
     std::array<double, 3> currentVel;
     std::array<double, 3> currentForce;

     std::array<double, 3> centerPos;
     std::array<double, 3> errorToCenter;

     double currectTime;
     double lastTime;

     float stiffness;
     float damping;

     void updateState();
     void computeForce();
     
public:
     FalconController();
     FalconController(float stiffness, float damping);
     ~FalconController();
     bool initialise();
     bool calibrateDevice();
     void run();
};

#endif
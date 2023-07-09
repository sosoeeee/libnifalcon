#ifndef CONTROLLER_H
#define CONTROLLER_H_H

#include "falcon/core/FalconDevice.h"
#include "falcon/gmtl/gmtl.h"

class FalconController
{
private:
     std::unique_ptr<libnifalcon::FalconDevice> m_falconDevice;
     bool m_displayCalibrationMessage;
     gmtl::Vec3d currentPosVec;
     std::array<double, 3> currentPos;
     gmtl::Vec3d currentForceVec;
     std::array<double, 3> currentForce;
     
public:
     FalconController();
     ~FalconController();
     bool initialise();
     bool calibrateDevice();
     void run();
};

#endif
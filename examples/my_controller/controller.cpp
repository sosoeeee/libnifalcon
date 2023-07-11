#include "controller.h"

#include <time.h>
#include <unistd.h>

#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/core/FalconGeometry.h"

FalconController::FalconController():
	m_falconDevice(std::unique_ptr<libnifalcon::FalconDevice>(new libnifalcon::FalconDevice)),
	m_displayCalibrationMessage(true),
	ctlReady(false),
	// sampleReady(false),
	currentPos({0,0,0}),
	lastPos({0,0,0}),
	currentVel({0,0,0}),
	centerPos({0,0,0}),
	errorToCenter({0,0,0}),
	currentForce({0,0,0}),
	currectTime(0),
	lastTime(0)
{
	this->stiffness = 1.0f;
	this->damping = 0.1f;
}

FalconController::FalconController(float stiffness, float damping):
	m_falconDevice(std::unique_ptr<libnifalcon::FalconDevice>(new libnifalcon::FalconDevice)),
	m_displayCalibrationMessage(true),
	ctlReady(false),
	// sampleReady(false),
	currentPos({0,0,0}),
	lastPos({0,0,0}),
	currentVel({0,0,0}),
	centerPos({0,0,0}),
	errorToCenter({0,0,0}),
	currentForce({0,0,0}),
	currectTime(0),
	lastTime(0)
{
	this->stiffness = stiffness;
	this->damping = damping;
}

FalconController::~FalconController()
{
}

bool FalconController::initialise()
{
     m_falconDevice->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();

	std::cout << "Setting up comm interface for Falcon comms" << std::endl;

	unsigned int count;
	m_falconDevice->getDeviceCount(count);
	std::cout << "Connected Device Count: " << count << std::endl;

	//Open the device number:
	int deviceNum = 0;
	std::cout << "Attempting to open Falcon device:  " << deviceNum << std::endl;
	if(!m_falconDevice->open(deviceNum))
	{
		std::cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << m_falconDevice->getErrorCode() << " Device Error Code: " << m_falconDevice->getFalconComm()->getDeviceErrorCode() << std::endl;
		return false;
	}
	else
	{
		std::cout << "Connected to Falcon device " << deviceNum << std::endl ;
	}

	//Load the device firmware:
	//There's only one kind of firmware right now, so automatically set that.
	m_falconDevice->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();
	//Next load the firmware to the device
	
	bool skip_checksum = false;
	//See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = m_falconDevice->isFirmwareLoaded();
	if(!firmware_loaded)
	{
		std::cout << "Loading firmware" << std::endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 10; ++i)
			{
				if(!m_falconDevice->getFalconFirmware()->loadFirmware(skip_checksum, libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					std::cout << "Firmware loading try failed";
					//Completely close and reopen
					//m_falconDevice->close();
					//if(!m_falconDevice->open(m_varMap["device_index"].as<int>()))
					//{
					//	std::cout << "Cannot open falcon device index " << m_varMap["device_index"].as<int>() << " - Lib Error Code: " << m_falconDevice->getErrorCode() << " Device Error Code: " << m_falconDevice->getFalconComm()->getDeviceErrorCode() << std::endl;
					//	return false;
					//}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else if(!firmware_loaded)
	{
		std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
		//return false;
	}
	else
	{
		//return true;
	}
	if(!firmware_loaded || !m_falconDevice->isFirmwareLoaded())
	{
		std::cout << "No firmware loaded to device, cannot continue" << std::endl;
		//return false;
	}
	std::cout << "Firmware loaded" << std::endl;

	//Seems to be important to run the io loop once to be sure of sensible values next time:
	m_falconDevice->runIOLoop();

	m_falconDevice->getFalconFirmware()->setHomingMode(true);

	m_falconDevice->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
     std::cout << "Falcon kinematic set" << std::endl;

     m_falconDevice->setFalconGrip<libnifalcon::FalconGripFourButton>();
     std::cout << "Falcon grip set" << std::endl;

	return true;
}

bool FalconController::calibrateDevice()
{
     m_falconDevice->getFalconFirmware()->setHomingMode(true);
     m_falconDevice->runIOLoop();
     if(!m_falconDevice->getFalconFirmware()->isHomed())
     {
          m_falconDevice->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
          if(m_displayCalibrationMessage)
          {
               std::cout << "Falcon not currently calibrated. Move control all the way out then push straight all the way in." << std::endl;
               m_displayCalibrationMessage = false;
          }
          return false;
     }
     std::cout << "Falcon calibrated successfully." << std::endl;
     m_falconDevice->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
     return true;
}

void FalconController::updateState()
{
	if (!this->ctlReady)
	{
		this->currectTime = clock();
		// run the IO loop
		m_falconDevice->runIOLoop();
		this->currentPos = m_falconDevice->getPosition();
		this->ctlReady = true;
	}
	else
	{
		this->lastPos = this->currentPos;

		// run the IO loop
		m_falconDevice->runIOLoop();
		this->currentPos = m_falconDevice->getPosition();

		this->lastTime = this->currectTime;
		this->currectTime = clock();

		double temp = ((double) (this->currectTime - this->lastTime)) / CLOCKS_PER_SEC;

		// compute the velocity
		this->currentVel[0] = (this->currentPos[0] - this->lastPos[0]) / temp;
		this->currentVel[1] = (this->currentPos[1] - this->lastPos[1]) / temp;
		this->currentVel[2] = (this->currentPos[2] - this->lastPos[2]) / temp;

		// std::cout << "Pos error" <<(this->currentPos[0] - this->lastPos[0]) << std::endl;
		// std::cout << "time error" << temp << std::endl;
		// std::cout << "Vel error" << this->currentVel[0] << std::endl;
	}
}

void FalconController::computeForce()
{
	// if (this->ctlReady)
	// {
	// 	this->errorToCenter = this->centerPos - this->currentPos;

	// 	// compute the force
	// 	this->currentForce = this->errorToCenter * this->stiffness - this->currentVel * this->damping;
	// }
}

void FalconController::run()
{
	double maxX = 0.0;
	double maxY = 0.0;
	double maxZ = 0.0;
	while(true)
	{
		// update the current position

		updateState();

		if (this->ctlReady)
		{
			// compute the force
			computeForce();
			// send the force to the device
			m_falconDevice->setForce(this->currentForce);
		}

		if (abs(this->currentPos[0]) > maxX)
		{
			maxX = abs(this->currentPos[0]);
		}
		if ((this->currentPos[1]) > maxY)
		{
			maxY = abs(this->currentPos[1]);
		}
		if (abs(this->currentPos[2]) > maxZ)
		{
			maxZ = abs(this->currentPos[2]);
		}

		// std::cout << "Strange"<<std::endl;

		// std::cout << "Max position: " << maxX << ", " << maxY << ", " << maxZ << std::endl;

		sleep(0.6);

		std::cout << "Current position: " << this->currentPos[0] << ", " << this->currentPos[1] << ", " << this->currentPos[2] << std::endl;
		// std::cout << "Current velocity: " << this->currentVel[0] << ", " << this->currentVel[1] << ", " << this->currentVel[2] << std::endl;
		// std::cout << "Current force: " << this->currentForce[0] << ", " << this->currentForce[1] << ", " << this->currentForce[2] << std::endl;
	}
}
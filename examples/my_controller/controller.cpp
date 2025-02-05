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
	sampleReady(false),
	currentPos({0,0,0}),
	lastPos({0,0,0}),
	currentVel({0,0,0}),
	centerPos({0,0,0.11}),
	errorToCenter({0,0,0}),
	errorSum({0,0,0}),
	currentForce({0,0,0})
{
	this->stiffness = 60;
	this->damping = 10;
	this->integral = 1;
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
		clock_gettime(CLOCK_MONOTONIC, &this->startT);
		
		// pthread_mutex_lock(&mutex_);
		this->currentPos = m_falconDevice->getPosition();
		// pthread_mutex_unlock(&mutex_);

		this->ctlReady = true;
	}
	else
	{
		this->lastPos[0] = this->currentPos[0];
		this->lastPos[1] = this->currentPos[1];
		this->lastPos[2] = this->currentPos[2];

		// pthread_mutex_lock(&mutex_);
		this->currentPos = m_falconDevice->getPosition();
		// pthread_mutex_unlock(&mutex_);

		// compute the time
		clock_gettime(CLOCK_MONOTONIC, &this->endT);
		double temp = (this->endT.tv_sec - this->startT.tv_sec) + (this->endT.tv_nsec - this->startT.tv_nsec) / 1000000000.0;
		clock_gettime(CLOCK_MONOTONIC, &this->startT);

		// compute the velocity
		this->currentVel[0] = (this->currentPos[0] - this->lastPos[0]) / temp;
		this->currentVel[1] = (this->currentPos[1] - this->lastPos[1]) / temp;
		this->currentVel[2] = (this->currentPos[2] - this->lastPos[2]) / temp;

		// std::cout << "Pos error" <<(this->currentPos[0] - this->lastPos[0]) << std::endl;
		// std::cout << "time error" << temp << std::endl;
		// std::cout << "Vel error" << this->currentVel[0] << std::endl;
	}
}

// PID controller
void FalconController::computeForce()
{
	if (this->ctlReady)
	{
		this->errorToCenter[0] = this->centerPos[0] - this->currentPos[0];
		this->errorToCenter[1] = this->centerPos[1] - this->currentPos[1];
		this->errorToCenter[2] = this->centerPos[2] - this->currentPos[2];

		this->errorSum[0] += this->errorToCenter[0];
		this->errorSum[1] += this->errorToCenter[1];
		this->errorSum[2] += this->errorToCenter[2];

		// limit the error sum
		if (abs(this->errorSum[0]) > 1)
		{
			this->errorSum[0] = 1 * (this->errorSum[0] / abs(this->errorSum[0]));
		}
		if (abs(this->errorSum[1]) > 1)
		{
			this->errorSum[1] = 1 * (this->errorSum[1] / abs(this->errorSum[1]));
		}
		if (abs(this->errorSum[2]) > 1)
		{
			this->errorSum[2] = 1 * (this->errorSum[2] / abs(this->errorSum[2]));
		}

		// printf("errorSum: %f, %f, %f\n", this->errorSum[0], this->errorSum[1], this->errorSum[2]);

		// compute the force
		this->currentForce[0] = this->errorToCenter[0] * this->stiffness - this->currentVel[0] * this->damping + this->errorSum[0] * this->integral;
		this->currentForce[1] = this->errorToCenter[1] * this->stiffness - this->currentVel[1] * this->damping + this->errorSum[1] * this->integral;
		this->currentForce[2] = this->errorToCenter[2] * this->stiffness - this->currentVel[2] * this->damping + this->errorSum[2] * this->integral;
	}
}

std::array<double, 3> FalconController::getPosition()
{
	return this->currentPos;
}

std::array<double, 3> FalconController::getForce()
{
	return this->currentForce;
}

unsigned int FalconController::getGripState(int index)
{
	switch (index)
	{
		case 1:
			return m_falconDevice->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::CENTER_BUTTON;
			break;
		
		case 2:
			return m_falconDevice->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::FORWARD_BUTTON;
			break;
		
		case 3:
			return m_falconDevice->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::MINUS_BUTTON;
			break;

		case 4:
			return m_falconDevice->getFalconGrip()->getDigitalInputs() & libnifalcon::FalconGripFourButton::PLUS_BUTTON;
			break;
	}
}

void FalconController::update()
{
	while(true)
	{
		updateState();

		if (this->ctlReady)
		{
			// compute the force
			computeForce();
			// send the force to the device

			// pthread_mutex_lock(&mutex_);
			m_falconDevice->setForce(this->currentForce);
			// pthread_mutex_unlock(&mutex_);
		}

		usleep(5 * 1000); // 5 ms

		// std::cout << "Current position: " << this->currentPos[0] << ", " << this->currentPos[1] << ", " << this->currentPos[2] << std::endl;
		// std::cout << "Current velocity: " << this->currentVel[0] << ", " << this->currentVel[1] << ", " << this->currentVel[2] << std::endl;
		// std::cout << "Current force: " << this->currentForce[0] << ", " << this->currentForce[1] << ", " << this->currentForce[2] << std::endl;
	}
}

void FalconController::run()
{
	while (true)
	{	
		// pthread_mutex_lock(&mutex_);
		m_falconDevice->runIOLoop();
		// pthread_mutex_unlock(&mutex_);
	}
}

void* FalconController::updateThread(void* arg)
{
	FalconController* controller = static_cast<FalconController*>(arg);
	controller->update();
	return NULL;
}

void* FalconController::runThread(void* arg)
{
	FalconController* controller = static_cast<FalconController*>(arg);
	controller->run();
	return NULL;
}
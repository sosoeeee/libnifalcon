#include "controller.h"

#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/core/FalconGeometry.h"

FalconController::FalconController():
     m_falconDevice(std::unique_ptr<libnifalcon::FalconDevice>(new libnifalcon::FalconDevice)),
     m_displayCalibrationMessage(true),
     currentPosVec(0,0,0),
     currentForceVec(0,0,0),
	currentPos({0,0,0}),
	currentForce({0,0,0})
{
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

void FalconController::run()
{
     m_falconDevice->runIOLoop();
     currentPos = m_falconDevice->getPosition();
	currentPosVec[0] = currentPos[0];
	currentPosVec[1] = currentPos[1];
	currentPosVec[2] = currentPos[2];
     std::cout << "Current position: " << currentPosVec << std::endl;
}
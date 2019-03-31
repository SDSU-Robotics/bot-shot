#include "Pickup.h"

#include <thread>
#include <chrono>

#include "PixyController.h"
#include "Display.h"
#include "DriveBase.h"


TalonSRX Pickup::_motorL = {DeviceIDs::pickupL};
TalonSRX Pickup::_motorR = {DeviceIDs::pickupR};

PIDController Pickup::_centeringPID = PIDController();

void Pickup::init()
{
    // configure PID controller for automatic centering
    _centeringPID.setKP(0.006);
    _centeringPID.setKI(0.0001);
    _centeringPID.setKD(0.01);
    _centeringPID.setILimit(1000.0);
    _centeringPID.setMaxOut(0.2);

    // set center of image as setpoint
    _centeringPID.setSetpoint(PixyController::getImageW() / 2.0);
}

void Pickup::active(bool active)
{
    if (active)
    {
        _motorL.Set(ControlMode::PercentOutput, -1 * INTAKE_POWER);
        _motorR.Set(ControlMode::PercentOutput, INTAKE_POWER);
    }
    else
    {
        _motorL.Set(ControlMode::PercentOutput, 0.0);
        _motorR.Set(ControlMode::PercentOutput, 0.0);
    }
}

void Pickup::center()
{
    // give the servo time to move into place
	if (pixy_rcs_get_position(0) != 0)
	{
		pixy_rcs_set_position(0, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// set brightness
	pixy_cam_set_brightness(PICKUP_PIXY_BRIGHTNESS);

	// update PID controller
	float output = _centeringPID.calcOutput(PixyController::getLatestBlock().x);

	DriveBase::setLeftPercent(output);
	DriveBase::setRightPercent(output * -1);
}
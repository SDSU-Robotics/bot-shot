#include "Pickup.h"

#include <thread>
#include <chrono>

#include "PixyController.h"
#include "Display.h"
#include "DriveBase.h"


TalonSRX Pickup::_motorL = {DeviceIDs::pickupL};
TalonSRX Pickup::_motorR = {DeviceIDs::pickupR};

PIDController Pickup::_centeringPID = PIDController();

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
		Display::debug("[Pickup, center] Centering on target...");
		pixy_rcs_set_position(0, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// set brightness
	pixy_cam_set_brightness(PICKUP_PIXY_BRIGHTNESS);

	struct Block block;
	int count = 0;

	do
	{
		block = PixyController::getLatestBlock();
		++count;
	} while (block.signature != ORANGE_BALL_SIG && count < 30);

	// update PID controller
	if (block.signature == ORANGE_BALL_SIG)
	{
		float output = _centeringPID.calcOutput(block.x);

		DriveBase::setLeftPercent(output);
		DriveBase::setRightPercent(output * -1);
	}
	
}
#include "DriveBase.h"

TalonSRX DriveBase::_motorL = {DeviceIDs::driveL};
TalonSRX DriveBase::_motorR = {DeviceIDs::driveR};

void DriveBase::setLeftPercent(float percentOutput)
{
	#ifndef DRIVE
		return;
	#endif // DRIVE

	// limit values
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motorL.Set(ControlMode::PercentOutput, percentOutput);
}

void DriveBase::setRightPercent(float percentOutput)
{
	#ifndef DRIVE
		return;
	#endif // DRIVE
	
	// limit values
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motorR.Set(ControlMode::PercentOutput, -1 * percentOutput);
}

void DriveBase::stop()
{
	_motorL.Set(ControlMode::PercentOutput, 0.0);
	_motorR.Set(ControlMode::PercentOutput, 0.0);
}
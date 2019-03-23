#include "DriveBase.h"

void DriveBase::setLeftPercent(float percentOutput)
{
	// limit values
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motorL.Set(ControlMode::PercentOutput, percentOutput);
}

void DriveBase::setRightPercent(float percentOutput)
{
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
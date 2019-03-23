#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class DriveBase
{
private:
	TalonSRX _motorL = {DeviceIDs::driveL};
	TalonSRX _motorR = {DeviceIDs::driveR};

public:
	void setLeftPercent(float);
	void setRightPercent(float);
	void stop();
};

#endif
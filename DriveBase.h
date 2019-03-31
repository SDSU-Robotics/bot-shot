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
	static TalonSRX _motorL;
	static TalonSRX _motorR;

public:
	static void setLeftPercent(float);
	static void setRightPercent(float);
	static void stop();
};

#endif
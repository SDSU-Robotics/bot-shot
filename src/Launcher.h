#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "Conversions.h"

const int MAX_RPM = 2000;

class Launcher
{
private:
	static TalonSRX _topWheel;
	static TalonSRX _bottomWheel;
	static TalonSRX _comArm;
	static TalonSRX _angleMotor;

	static float _rpmSetpoint;

public:
	static void init();

	static void setRPM(float rpm);
	static float getRPM() { return _rpmSetpoint; }
	static float getRPMsetpoint() { return _rpmSetpoint; }
	static float getBottomEncoderRPM()
		{ return Conversions::toRpm( _bottomWheel.GetSelectedSensorVelocity()); }
	static float getTopEncoderRPM()
		{ return Conversions::toRpm( _topWheel.GetSelectedSensorVelocity()); }

	static void setAngleSpeed(float percentOutput) 
		{ _angleMotor.Set(ControlMode::PercentOutput, percentOutput); }
	static void setComSpeed(float percentOutput) 
		{ _comArm.Set(ControlMode::PercentOutput, percentOutput); }

	static void stop();
};

#endif
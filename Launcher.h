#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "Conversions.h"
#include "PIDController.h"

const float MIN_LAUNCH_ANGLE = 35.0;
const float MAX_LAUNCH_ANGLE = 85.0;

const uint8_t LAUNCH_PIXY_BRIGHTNESS = 80;

const int HOOP_SIG = 1;

class Launcher
{
private:
	static TalonSRX _topWheel;
	static TalonSRX _bottomWheel;
	static TalonSRX _comArm;
	static TalonSRX _angleMotor;


	static PIDController _launchAnglePID;
	static PIDController _comArmPID;
	static PIDController _horizontalPixyPID;
	static PIDController _verticalPixyPID;

	static ControlMode _launchAngleControlMode;
	static ControlMode _comAngleControlMode;

	static float _angleMotorOutput;
	static float _lastLaunchAngle;
	static float _rpmSetpoint;

public:
	static void init();

	static void setLaunchAngleControlMode(ControlMode controlMode);
	static ControlMode getLaunchAngleControlMode() { return _launchAngleControlMode; }

	static void setComAngleControlMode(ControlMode controlMode) { _comAngleControlMode = controlMode; }
	static ControlMode getComAngleControlMode() { return _comAngleControlMode; }

	static void setRPM(float rpm);
	static float getRPM() { return _rpmSetpoint; }
	static float getBottomEncoderRPM() { return Conversions::toRpm( _bottomWheel.GetSelectedSensorVelocity()); }
	static float getTopEncoderRPM() { return Conversions::toRpm( _topWheel.GetSelectedSensorVelocity()); }

	static void setLaunchAngle(float setAngle);
	static void setComAngle(float setAngle);

	static float  getLaunchAngle();

	static void centerHorizontal();
};

#endif
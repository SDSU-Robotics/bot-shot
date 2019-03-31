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

class Launcher
{
private:
    TalonSRX _topWheel = {DeviceIDs::launcherTop};
	TalonSRX _bottomWheel = {DeviceIDs::launcherBottom};
    TalonSRX _comArm = {DeviceIDs::commencementArm};
	TalonSRX _angleMotor = {DeviceIDs::launcherAngle};

    PIDController _launchAnglePID;
    PIDController _comArmPID;

    ControlMode _launchAngleControlMode;
    ControlMode _comAngleControlMode;

    float _rpmSetpoint;

public:
    void init();

    void setLaunchAngleControlMode(ControlMode controlMode) { _launchAngleControlMode = controlMode; }
    ControlMode getLaunchAngleControlMode() { return _launchAngleControlMode; }

    void setComAngleControlMode(ControlMode controlMode) { _comAngleControlMode = controlMode; }
    ControlMode getComAngleControlMode() { return _comAngleControlMode; }

    void setRPM(float rpm);
    float getRPM() { return _rpmSetpoint; }

    void setLaunchAngle(float setAngle);
    void setComAngle(float setAngle);
};

#endif
#ifndef LAUNCHER_H
#define LAUNCHER_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "Conversions.h"
#include "PIDController.h"

class Launcher
{
private:
    TalonSRX _topWheel = {DeviceIDs::launcherTop};
	TalonSRX _bottomWheel = {DeviceIDs::launcherBottom};
    TalonSRX _comArm = {DeviceIDs::commencementArm};
	TalonSRX _angleMotor = {DeviceIDs::launcherAngle};

    PIDController _launchAnglePID;
    PIDController _comArmPID;

    ControlMode _controlMode;

    float _rpmSetpoint;

public:
    void init();

    void setControlMode(ControlMode controlMode) { _controlMode = controlMode; }
    ControlMode getControlMode() { return _controlMode; }

    void setRPM(float rpm);
    float getRPM() { return _rpmSetpoint; }

    void setLaunchAngle(float angle);
    void setComAngle(float angle);
    
    void update(float launchAngle, float comAngle);
};

#endif
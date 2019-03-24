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

    float _rpmSetpoint;

public:
    void init();
    void setRPM(float rpm);
    float getRPM() { return _rpmSetpoint; }

    void setLaunchAngle(float angle) { _launchAnglePID.setSetpoint(angle); };
    void setComAngle(float angle) { _comArmPID.setSetpoint(angle); };
    
    void update(float launchAngle, float comAngle);
};

#endif
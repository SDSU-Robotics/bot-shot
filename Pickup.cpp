#include "Pickup.h"
#include "PixyController.h"
#include "Display.h"

void Pickup::init()
{
    // configure PID controller for automatic centering
    _centeringPID.setKP(0.006);
    _centeringPID.setKI(0.0001);
    _centeringPID.setKD(0.01);
    _centeringPID.setILimit(1000.0);
    _centeringPID.setMaxOut(0.2);

    // set center of image as setpoint
    _centeringPID.setSetpoint(PixyController::getImageW() / 2.0);
}

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

float Pickup::centeringUpdate(struct Block block)
{
    return _centeringPID.calcOutput(block.x);
}
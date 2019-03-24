#include "Pickup.h"

void Pickup::active(bool active)
{
    if (active)
    {
        _motorL.Set(ControlMode::PercentOutput, INTAKE_POWER);
        _motorR.Set(ControlMode::PercentOutput, -1 * INTAKE_POWER);
    }
    else
    {
        _motorL.Set(ControlMode::PercentOutput, 0.0);
        _motorR.Set(ControlMode::PercentOutput, 0.0);
    }
}
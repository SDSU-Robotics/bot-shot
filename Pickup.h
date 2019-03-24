#ifndef PICKUP_H
#define PIKCUP_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

const float INTAKE_POWER = 0.3;

class Pickup
{
private:
    TalonSRX _motorL = {DeviceIDs::pickupL};
	TalonSRX _motorR = {DeviceIDs::pickupR};

public:
    void active(bool active);
};

#endif
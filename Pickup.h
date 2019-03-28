#ifndef PICKUP_H
#define PIKCUP_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "PIDController.h"
#include "pixy.h"

const float INTAKE_POWER = 0.5;

class Pickup
{
private:
    TalonSRX _motorL = {DeviceIDs::pickupL};
	TalonSRX _motorR = {DeviceIDs::pickupR};

    PIDController _centeringPID;

public:
    void init();
    void active(bool active);
    float centeringUpdate(struct Block block);
};

#endif
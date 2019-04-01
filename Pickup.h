#ifndef PICKUP_H
#define PIKCUP_H

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "PIDController.h"
#include "pixy.h"

const float INTAKE_POWER = 0.5;
const uint8_t PICKUP_PIXY_BRIGHTNESS = 80;

const int ORANGE_BALL_SIG = 2;
const int BLACK_BALL_SIG = 3;

class Pickup
{
private:
    static TalonSRX _motorL;
	static TalonSRX _motorR;

    static PIDController _centeringPID;

public:
    static void active(bool active);
    static void center();
};

#endif
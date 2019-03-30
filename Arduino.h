#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;

const char _comPort[] = "/dev/ttyUSB11";

const float LAUNCH_ANGLE_HOME = 35.0;

class Arduino
{
	private:	
		static int _serPort;
		static float _launcAngleOffset;

		static bool initSerial();
		static void home();

	public:
		static bool init();
		static bool getLaunchAngle(float &com);
};

#endif
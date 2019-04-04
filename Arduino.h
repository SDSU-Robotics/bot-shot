#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;


const float LAUNCH_ANGLE_HOME = 35.0;
const float SERVO_SCALING_FACTOR = 1.5;
const float SERVO_DEG_OFFSET = 0.0;
const int NUM_READINGS = 10;

class Arduino
{
	private:	
		static int _serPort;
		static float _launchAngleOffset;
		static bool _calibrated;
		static uint8_t _servoPos;
		static uint8_t _posReadings[NUM_READINGS];
		static int _servoTot;
		static int _readIndex;

		static bool initSerial();
		static void home();

	public:
		static bool init();
		static bool isCalibrated() { return _calibrated; }
		static bool getLaunchAngle(float &angle);

		static void setServoPos(uint8_t servoPos);
		static uint8_t getServoPos() { return _servoPos; }
		static float getServoAngle()
			{ return _servoPos * SERVO_SCALING_FACTOR + SERVO_DEG_OFFSET; }
};

#endif
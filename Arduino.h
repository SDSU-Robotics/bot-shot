#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;

class Arduino
{
	private:	
		char comPort[13] = "/dev/ttyUSB3";

	public:
		bool IMUread(float &com, float &launcher);
};

#endif
#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;

class Arduino
{
	private:	
		char _comPort[13] = "/dev/ttyUSB0";
		FILE *_serPort;

	public:
		bool init();
		bool IMUread(float &com, float &launcher);
};

#endif
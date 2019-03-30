#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;

const char _comPort[13] = "/dev/ttyUSB1";

class Arduino
{
	private:	
		FILE *_serPort;

	public:
		bool init();
		bool IMUread(float &com);
};

#endif
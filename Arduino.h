#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>

using namespace std;

const char _comPort[] = "/dev/ttyUSB11";

class Arduino
{
	private:	
		int _serPort;

	public:
		bool init();
		bool IMUread(float &com);
};

#endif
#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>


class Arduino
{
	private:	
		char comPort[] = "/dev/ttyUSB3";

	public:
		bool IMURead(float &com, float &launcher);
};

#endif
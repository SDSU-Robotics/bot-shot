#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include "Arduino.h"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "Display.h"
#include "Controller.h"
#include "Launcher.h"
#include "Enables.h"

using namespace std;

int Arduino::_serPort = 0;
float Arduino::_launchAngleOffset = 0.0;
bool Arduino::_calibrated = false;
uint8_t Arduino::_servoPos = 0;
uint8_t Arduino::_posReadings[NUM_READINGS] = {0};
int Arduino::_servoTot = 0;
int Arduino::_readIndex = 0;

bool Arduino::init()
{
	#ifndef ARDUINO
		Display::debug("[Arduino, init] WARNING: Arduino disabled!");
		return true;
	#endif // ARDUINO

	if (!initSerial())
		return false;

	home();
	return true;
}

bool Arduino::initSerial()
{
	char _comPort[] = "/dev/ttyUSB0";

	//Open up Serial Communication
	for (int i = 0; i < 10; ++i)
	{
		_comPort[11] = i + 48;
		_serPort = open(_comPort, O_RDWR);
		if (_serPort >= 0)
			break;
	}

	//If communication fails, print error
	if (_serPort < 0)
	{
		Display::debug("[Arduino, init] Error " + to_string(errno) + " from open: " + strerror(errno));
		Display::debug("[Arduino, init] Fatal error. Terminating.");

		return false;
	}

	Display::debug("[Arduino, init] Arduino connected.");
	
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	//tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(_serPort, TCSANOW, &tty) != 0) {
		Display::debug("[Arduino, init] Error from tcsetattr");
	}

	for (int zeroThings = 0; zeroThings < NUM_READINGS; zeroThings++)
	{
		_posReadings[zeroThings] = 0;
	}

	return true;
}

void Arduino::home()
{
	Display::debug("[Arduino, home] Homing angles... hit START when ready.");

	_calibrated = false;

	Launcher::setLaunchAngleControlMode(ControlMode::PercentOutput);

	bool waiting;
	do
	{
		Controller::poll();
		
		Launcher::setLaunchAngle(Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_Y));

		waiting = !(Controller::getButton(Controller::LAUNCH, Controller::START)
				 || Controller::getButton(Controller::DRIVE, Controller::START));
		
		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (waiting);
		

	Display::debug("[Arduino, home] Do not touch the robot. Homing...");

	float reading = 0.0;
	float total = 0.0;

	// discard junk data	
	for (int i = 0; i < 50; ++i)
	{
		getLaunchAngle(reading);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	for (int i = 0; i < 50; ++i)
	{
		if (getLaunchAngle(reading))
		{
			total += reading;
		}
		else
			--i;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	_launchAngleOffset = LAUNCH_ANGLE_HOME - total / 50.0;
	_calibrated = true;

	Display::debug("[Arduino, home] Homing complete. Launch angle offset: " + to_string(_launchAngleOffset));
	Display::debug("[Arduino, home] Angle readings:");
	float angle;
	for (int i = 0; i < 10; ++i)
	{
		if (!getLaunchAngle(angle))
		{
			--i;
			continue;
		}
		Display::debug(to_string(angle));
	}
	Display::debug("[Arduino, home] Hit SELECT to re-calibrate or START to continue...");

	do
	{
		Controller::poll();
		waiting = !(Controller::getButton(Controller::LAUNCH, Controller::START) ||
					Controller::getButton(Controller::LAUNCH, Controller::SEL) ||
					Controller::getButton(Controller::DRIVE, Controller::START) ||
					Controller::getButton(Controller::DRIVE, Controller::SEL));
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (waiting);
	
	if (Controller::getButton(Controller::LAUNCH, Controller::SEL) || Controller::getButton(Controller::DRIVE, Controller::SEL))
		home();

	Display::debug("[Arduino, home] Homing complete.");
}


bool Arduino::getLaunchAngle(float &angle)
{
	#ifndef ARDUINO
		angle = 0.0;
		return true;
	#endif // ARDUINO

	char buf[16];

	char msg[] = {'0'};

	int count = 0, bytes = 0;

	do
	{
		write(_serPort, msg, sizeof(msg));

		bytes = read(_serPort, buf, sizeof(buf));

		if (bytes != 0)
		{
			if (_calibrated)
				angle = (atof(buf) + _launchAngleOffset) * 1.72 - 25.2;
			else
				angle = atof(buf);
			
			return true;
		}

		++count;
	} while (bytes == 0 && count < 10);
	
	return false;
}

void Arduino::setServoPos(uint8_t pos)
{
	//if (pos > 255)
	//	pos = 255;
	
	_servoTot = _servoTot - _posReadings[_readIndex];
	_posReadings[_readIndex] = pos;
	_servoTot = _servoTot + _posReadings[_readIndex];
	_readIndex++;

	if (_readIndex >= NUM_READINGS){
		_readIndex = 0;
	}

	_servoPos = _servoTot / NUM_READINGS;
	Display::debug("Servo Avg: " + to_string(_servoPos));
	
	char msg[] = {'1', _servoPos};
	write(_serPort, msg, sizeof(msg));
}
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

using namespace std;

int Arduino::_serPort = 0;
float Arduino::_launchAngleOffset = 0.0;
bool Arduino::_calibrated = false;

bool Arduino::init()
{
	if (!initSerial())
		return false;

	home();
	return true;
}

bool Arduino::initSerial()
{
	//Open up Serial Communication
    _serPort = open(_comPort, O_RDWR);

	//If communication fails, print error
	if (_serPort < 0)
	{
        Display::print("[Arduino, init] Error " + to_string(errno) + "from open: " + strerror(errno));
		Display::print("[Arduino, init] Fatal error. Terminating.");          
		return false;
	}

	Display::print("[Arduino, init] Arduino connected.");
   
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
		Display::print("[Arduino, init] Error from tcsetattr");
	}

	return true;
}

void Arduino::home()
{
	Display::print("[Arduino, home] Homing angles... hit START when ready.");

	_calibrated = false;

	Launcher::setLaunchAngleControlMode(ControlMode::PercentOutput);

	bool waiting;
	do
	{
		Controller::poll();
		
		Launcher::setLaunchAngle(Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_Y));

		waiting = !(Controller::getButton(Controller::LAUNCH, Controller::START) || Controller::getButton(Controller::DRIVE, Controller::START));
		
		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	} while (waiting);
		

	Display::print("[Arduino, home] Do not touch the robot. Homing...");

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
		getLaunchAngle(reading);
		total += reading;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	_launchAngleOffset = LAUNCH_ANGLE_HOME - total / 50.0;
	_calibrated = true;

	Display::print("[Arduino, home] Homing complete. Launch angle offset: " + to_string(_launchAngleOffset));
	Display::print("[Arduino, home] Angle readings:");
	float angle;
	for (int i = 0; i < 10; ++i)
	{
		if (!getLaunchAngle(angle))
			;//--i;
		Display::print(to_string(angle));
	}
	Display::print("[Arduino, home] Hit SELECT to re-calibrate or START to continue...");

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
}


bool Arduino::getLaunchAngle(float &angle)
{
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
            	angle = (atof(buf) + _launchAngleOffset) * 1.72 - 25.9;
			else
				angle = atof(buf);
			
            return true;
        }

        ++count;
    } while (bytes == 0 && count < 10);
	
    return false;
}
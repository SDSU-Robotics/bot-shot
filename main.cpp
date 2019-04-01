#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Arduino.h"
#include <string>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <SDL2/SDL.h>

#include "DriveBase.h"
#include "Controller.h"
#include "Display.h"
#include "Pickup.h"
#include "Launcher.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const float FAST_SPEED = 0.99;
const float SLOW_SPEED = 0.2;

void inline sleepApp(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void updateDrive();
void updateLaunchWheels();
void updateLaunchAngle();
void updateComAngle();

int main()
{
	ctre::phoenix::platform::can::SetCANInterface("can0");

	bool running = true;

	Display::init();
	
	// wait for Talons to get ready
	sleepApp(2000);

	while (running) {
		// we are looking for gamepad (first time or after disconnect),
		// neutral drive until gamepad (re)connected.
		DriveBase::stop();
		Controller::init();

		if(!Arduino::init())
			break;

		Launcher::init();
		Launcher::setComAngleControlMode(ControlMode::PercentOutput); // manual control
		Launcher::setLaunchAngleControlMode(ControlMode::PercentOutput);   // manual mode

		// Keep reading the state of the joystick in a loop
		while (true) {
			// poll for disconnects or bad things
			SDL_Event event;
			if (Controller::poll(event)) {
				if (event.type == SDL_QUIT) { running = false; break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			if (Controller::getButton(Controller::DRIVE, Controller::Y)) // pickup centering
				Pickup::center();
			else if (Controller::getButton(Controller::LAUNCH, Controller::Y)) // launch horizontal centering
				Launcher::centerHorizontal();
			else 
				updateDrive(); // drivebase control

			updateComAngle();
			
			Pickup::active(Controller::getButton(Controller::LAUNCH, Controller::A));
			updateLaunchAngle();
			updateLaunchWheels();

			Display::update();

			ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

			/* loop yield for a bit */
			sleepApp(20);
		}

		// we've left the loop, likely due to gamepad disconnect
		DriveBase::stop();
	}

	SDL_Quit();
	return 0;
}

void updateDrive()
{
	// get controller values
	float speed = Controller::getAxis(Controller::DRIVE, Controller::LEFT_Y);
	float turn = -1 * Controller::getAxis(Controller::DRIVE, Controller::RIGHT_X);

	float speedFactor;

	if (Controller::getAxis(Controller::DRIVE, Controller::LEFT_T) < 0.0 &&
		Controller::getAxis(Controller::DRIVE, Controller::RIGHT_T) < 0.0)
	{
		speedFactor = FAST_SPEED;
	}	
	else
		speedFactor = SLOW_SPEED;
	

	float lSpeed = speedFactor * 0.5 * speed + 0.25 * turn;
	float rSpeed = speedFactor * 0.5 * speed - 0.25 * turn;

	DriveBase::setLeftPercent(lSpeed);
	DriveBase::setRightPercent(rSpeed);
}


void updateLaunchWheels()
{
	// get controller values
	float lt = Controller::getAxis(Controller::LAUNCH, Controller::LEFT_T);
	float rt = Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_T);
	bool stop = Controller::getButton(Controller::LAUNCH, Controller::SEL);

	float newRPM = 0.0;

	if (!stop)
		newRPM = Launcher::getRPM() + (lt - 1) * 5.0 + (rt - 1) * -5.0;
	
	Launcher::setRPM(newRPM);
}

void updateLaunchAngle()
{
	float angle;
	bool success;

	switch(Launcher::getLaunchAngleControlMode())
	{
		case ControlMode::Position:
			Launcher::setLaunchAngle(42.5);
			break;
		
		case ControlMode::PercentOutput:
			Launcher::setLaunchAngle(Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_Y));
			break;

		default:
			Display::debug("[main, updateAngles] Invalid control mode returned for launchAngleControlMode.");
	}
}

void updateComAngle()
{
	switch(Launcher::getComAngleControlMode())
	{
		case ControlMode::Position:
			// not ready
		
		case ControlMode::PercentOutput:
			Launcher::setComAngle(Controller::getAxis(Controller::LAUNCH, Controller::LEFT_Y));
			break;

		default:
			Display::debug("[main, updateAngles] Invalid control mode returned from comAngleControlMode.");
	}
}
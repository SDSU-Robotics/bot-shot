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
#include "Enables.h"

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
	Display::init();

	ctre::phoenix::platform::can::SetCANInterface("can0");

	bool running = true;

	// wait for Talons to get ready
	sleepApp(2000);

	while (running) {
		// we are looking for gamepad (first time or after disconnect),
		// neutral drive until gamepad (re)connected.
		DriveBase::stop();
		Launcher::stop();

		Controller::init();

		if(!Arduino::init())
			break;

		Launcher::init();

		// Keep reading the state of the joystick in a loop
		while (true) {
			// poll for disconnects or bad things
			SDL_Event event;
			if (Controller::poll(event)) {
				if (event.type == SDL_QUIT) { running = false; break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			updateDrive(); // drivebase control

			// camera servo control
			if (Controller::getButton(Controller::LAUNCH, Controller::X))
			{
				int newPos = Arduino::getServoPos() + 10.0 * Controller::getAxis(Controller::LAUNCH, Controller::LEFT_Y);
				if (newPos > 255)
					newPos = 255;
				else if (newPos < 0)
					newPos = 0;
				
				Arduino::setServoPos(newPos);
			}
			else
				Launcher::setComSpeed(Controller::getAxis(Controller::LAUNCH, Controller::LEFT_Y));
			
			Pickup::active(Controller::getButton(Controller::LAUNCH, Controller::A ));
			Launcher::setAngleSpeed(Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_Y));

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
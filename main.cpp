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
#include "PixyController.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const float FAST_SPEED = 0.99;
const float SLOW_SPEED = 0.2;

const int HOOP_SIG = 1;
const int ORANGE_BALL_SIG = 2;
const int BLACK_BALL_SIG = 3;

const uint8_t LAUNCH_PIXY_BRIGHTNESS = 80;

void inline sleepApp(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void updateDrive();
void updatePickup();
void updateLauncher();
void updateAngles();

Launcher launcher;

int main()
{
	ctre::phoenix::platform::can::SetCANInterface("can0");

	bool running = true;

	PixyController::init();
	Pickup::init();

	// wait for Talons to get ready
	sleepApp(2000);

	while (running) {
		// we are looking for gamepad (first time or after disconnect),
		// neutral drive until gamepad (re)connected.
		DriveBase::stop();

		Controller::init();
		Arduino::init();
		launcher.init();

		launcher.setComAngleControlMode(ControlMode::PercentOutput); // manual control
		launcher.setLaunchAngleControlMode(ControlMode::PercentOutput);   // manual mode

		// Keep reading the state of the joystick in a loop
		while (true) {
			// poll for disconnects or bad things
			SDL_Event event;
			if (Controller::poll(event)) {
				if (event.type == SDL_QUIT) { running = false; break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			if (Controller::getButton(Controller::DRIVE, Controller::Y)) // pickup centering
			{
				Pickup::center();
			}
			else
			{
				if (pixy_rcs_get_position(0) != 999)
				{
					pixy_rcs_set_position(0, 999);
					sleepApp(1000);
				}
				
				updateDrive(); // drivebase control
			}

			updatePickup();
			updateLauncher();

			Display::print("");

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

void updatePickup()
{
	// get controller values
	bool active = Controller::getButton(Controller::LAUNCH, Controller::A);

	Pickup::active(active);
}

void updateLauncher()
{
	// get controller values
	float lt = Controller::getAxis(Controller::LAUNCH, Controller::LEFT_T);
	float rt = Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_T);
	bool stop = Controller::getButton(Controller::LAUNCH, Controller::SEL);

	float newRPM = 0.0;

	if (!stop)
		newRPM = launcher.getRPM() + (lt - 1) * 5.0 + (rt - 1) * -5.0;
	
	launcher.setRPM(newRPM);
	Display::print("RPM Setpoint: " + to_string(newRPM));

	updateAngles();
}

void updateAngles()
{
	float angle;
	bool success;

	switch(launcher.getLaunchAngleControlMode())
	{
		case ControlMode::Position:
			launcher.setLaunchAngle(42.5);
			break;
		
		case ControlMode::PercentOutput:
			launcher.setLaunchAngle(Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_Y));
			break;

		default:
			Display::print("[main, updateAngles] Invalid control mode returned for launchAngleControlMode.");
	}


	switch(launcher.getComAngleControlMode())
	{
		case ControlMode::Position:
			// not ready
		
		case ControlMode::PercentOutput:
			launcher.setComAngle(Controller::getAxis(Controller::LAUNCH, Controller::LEFT_Y));
			break;

		default:
			Display::print("[main, updateAngles] Invalid control mode returned from comAngleControlMode.");
	}
}

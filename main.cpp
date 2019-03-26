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

const float MAX_SPEED = 0.99;

void inline sleepApp(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void updateDrive();
void updatePickup();
void updateLauncher();
void updateAngles();

DriveBase drivebase;
Controller controller;
Pickup pickup;
Launcher launcher;

Arduino arduino;

int main() {
	ctre::phoenix::platform::can::SetCANInterface("can0");

	bool running = true;

	arduino.init();

	// wait for Talons to get ready
	sleepApp(2000);

	launcher.init();
	launcher.setControlMode(ControlMode::PercentOutput); // manual control

	while (running) {
		// we are looking for gamepad (first time or after disconnect),
		// neutral drive until gamepad (re)connected.
		drivebase.stop();
		controller.init();

		// Keep reading the state of the joystick in a loop
		while (true) {
			// poll for disconnects or bad things
			SDL_Event event;
			if (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT) { running = false; break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			updateDrive(); // drivebase control
			updatePickup();
			updateLauncher();
			//updateAngles();

			ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

			/* loop yield for a bit */
			sleepApp(20);
		}

		// we've left the loop, likely due to gamepad disconnect
		drivebase.stop();
	}

	SDL_Quit();
	return 0;
}

void updateDrive()
{
	// get controller values
	float speed = controller.getAxis(Controller::DRIVE, Controller::LEFT_Y);
	float turn = -1 * controller.getAxis(Controller::DRIVE, Controller::RIGHT_X);

	float lSpeed = MAX_SPEED * 0.5 * speed + MAX_SPEED * 0.5 * turn;
	float rSpeed = MAX_SPEED * 0.5 * speed - MAX_SPEED * 0.5 * turn;

	drivebase.setLeftPercent(lSpeed);
	drivebase.setRightPercent(rSpeed);
}

void updatePickup()
{
	// get controller values
	bool active = controller.getButton(Controller::LAUNCH, Controller::A);

	pickup.active(active);
}

void updateLauncher()
{
	// get controller values
	float lt = controller.getAxis(Controller::LAUNCH, Controller::LEFT_T);
	float rt = controller.getAxis(Controller::LAUNCH, Controller::RIGHT_T);
	bool stop = controller.getButton(Controller::LAUNCH, Controller::SEL);

	float newRPM = 0.0;

	if (!stop)
		newRPM = launcher.getRPM() + (lt - 1) * 5.0 + (rt - 1) * -5.0;
	
	launcher.setRPM(newRPM);
	Display::print("RPM Setpoint: " + to_string(newRPM));
}

void updateAngles()
{
	switch(launcher.getControlMode())
	{
		float comArmAngle, launcherAngle;
		bool success;

		case ControlMode::Position:
			//Get IMU values
			success = arduino.IMUread(comArmAngle, launcherAngle);

			if(success)
				Display::print("Commencement Arm: " + to_string(comArmAngle) + "\tLauncher: " + to_string(launcherAngle));
			else
				Display::print("UhOh, the IMUs aren't working :(");
			break;
		
		case ControlMode::PercentOutput:
			launcher.setLaunchAngle(controller.getAxis(Controller::LAUNCH, Controller::LEFT_Y));
			launcher.setComAngle(controller.getAxis(Controller::LAUNCH, Controller::RIGHT_Y));

		default:
			Display::print("[main, updateAngles] Invalid control mode returned from launcher.");
	}
	

	
}

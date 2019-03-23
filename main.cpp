#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <SDL2/SDL.h>

#include "DriveBase.h"
#include "Controller.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const float MAX_SPEED = 0.99;

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
	ctre::phoenix::platform::can::SetCANInterface("can0");

	DriveBase drivebase;
	Controller controller;

	bool running = true;

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

			// get controller values
			float speed = controller.getAxis(Controller::DRIVE, Controller::LEFT_Y);
			float turn = -1 * controller.getAxis(Controller::DRIVE, Controller::RIGHT_X);

			float lSpeed = MAX_SPEED * 0.5 * speed + MAX_SPEED * 0.5 * turn;
			float rSpeed = MAX_SPEED * 0.5 * speed - MAX_SPEED * 0.5 * turn;

			drivebase.setLeftPercent(lSpeed);
			drivebase.setRightPercent(rSpeed);

			ctre::phoenix::unmanaged::FeedEnable(100);

			/* loop yield for a bit */
			sleepApp(20);
		}

		// we've left the loop, likely due to gamepad disconnect
		drivebase.stop();
	}

	SDL_Quit();
	return 0;
}

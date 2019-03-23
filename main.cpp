#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
	ctre::phoenix::platform::can::SetCANInterface("can0");

	TalonSRX motorL(1);
	TalonSRX motorR(2);

	bool running = true;

	while (running) {
		/* we are looking for gamepad (first time or after disconnect),
			neutral drive until gamepad (re)connected. */
		motorL.Set(ControlMode::PercentOutput, 0.0);
		motorR.Set(ControlMode::PercentOutput, 0.0);

		// wait for gamepad
		printf("Waiting for gamepad...\n");
		while (true) {
			/* SDL seems somewhat fragile, shut it down and bring it up */
			SDL_Quit();
			SDL_Init(SDL_INIT_JOYSTICK);

			/* poll for gamepad */
			int res = SDL_NumJoysticks();
			if (res > 0) { break; }
			if (res < 0) { printf("Err = %i\n", res); }

			/* yield for a bit */
			sleepApp(20);
		}
		printf("Waiting for gamepad...Found one\n");

		// Open the joystick for reading and store its handle in the joy variable
		SDL_Joystick *joy = SDL_JoystickOpen(0);
		if (joy == NULL) {
			/* back to top of while loop */
			continue;
		}

		// Get information about the joystick
		const char *name = SDL_JoystickName(joy);
		const int num_axes = SDL_JoystickNumAxes(joy);
		const int num_buttons = SDL_JoystickNumButtons(joy);
		const int num_hats = SDL_JoystickNumHats(joy);
		printf("Now reading from joystick '%s' with:\n"
			"%d axes\n"
			"%d buttons\n"
			"%d hats\n\n",
			name,
			num_axes,
			num_buttons,
			num_hats);

		// Keep reading the state of the joystick in a loop
		while (true) {
			/* poll for disconnects or bad things */
			SDL_Event event;
			if (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT) { running = false; break; }
				if (event.jdevice.type == SDL_JOYDEVICEREMOVED) { break; }
			}

			// grab some stick values
			double speed = ((double)SDL_JoystickGetAxis(joy, 1)) / -32767.0;

			motorL.Set(ControlMode::PercentOutput, speed);
			motorR.Set(ControlMode::PercentOutput, speed);

			ctre::phoenix::unmanaged::FeedEnable(100);

			/* loop yield for a bit */
			sleepApp(20);
		}

		/* we've left the loop, likely due to gamepad disconnect */
		motorL.Set(ControlMode::PercentOutput, 0.0);
		motorR.Set(ControlMode::PercentOutput, 0.0);
		SDL_JoystickClose(joy);
		printf("gamepad disconnected\n");
	}

	SDL_Quit();
	return 0;
}

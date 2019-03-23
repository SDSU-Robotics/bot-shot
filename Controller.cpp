#include "Controller.h"
#include <iostream>
#include <thread>
#include <chrono>

bool Controller::init()
{
    // wait for gamepad
	std::cout << "Waiting for gamepad...\n";
	while (true) {
        /* SDL seems somewhat fragile, shut it down and bring it up */
        SDL_Quit();
        SDL_Init(SDL_INIT_JOYSTICK);

		/* poll for gamepad */
		int res = SDL_NumJoysticks();
		if (res > 0) { break; }
		if (res < 0) { printf("Err = %i\n", res); }

		/* yield for a bit */
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	std::cout << "Waiting for gamepad...Found one\n";

	// Open the joystick for reading and store its handle in the joy variable
	_driveJoy = SDL_JoystickOpen(0);
    _launchJoy = SDL_JoystickOpen(1);
	if (!_driveJoy || !_launchJoy) {
		// retry init
		init();
	}

    // Get information about the joystick
	char *name = (char*)SDL_JoystickName(_driveJoy);
	int num_axes = SDL_JoystickNumAxes(_driveJoy);
	int num_buttons = SDL_JoystickNumButtons(_driveJoy);
	int num_hats = SDL_JoystickNumHats(_driveJoy);
	printf("Drive controller '%s' with:\n"
		"%d axes\n"
		"%d buttons\n"
		"%d hats\n\n",
		name,
		num_axes,
		num_buttons,
		num_hats);

    // Get information about the joystick
	name = (char*)SDL_JoystickName(_driveJoy);
	num_axes = SDL_JoystickNumAxes(_driveJoy);
	num_buttons = SDL_JoystickNumButtons(_driveJoy);
	num_hats = SDL_JoystickNumHats(_driveJoy);
	printf("Launch controller '%s' with:\n"
		"%d axes\n"
		"%d buttons\n"
		"%d hats\n\n",
		name,
		num_axes,
		num_buttons,
		num_hats);
}

float Controller::getAxis(Controller_t controller, Axis_t axis)
{
    switch(controller)
    {
    case DRIVE:
        return SDL_JoystickGetAxis(_driveJoy, axis) / -32767.0;
        break;
    case LAUNCH:
        return SDL_JoystickGetAxis(_launchJoy, axis) / -32767.0;
        break;
    default:
        std::cout << "[Controller] Error: Invalid controller in getAxis.\n";
    }
}

Controller::~Controller()
{
    SDL_JoystickClose(_driveJoy);
    SDL_JoystickClose(_launchJoy);

    std::cout << "[Controller] Gamepads disconnected.\n";
}
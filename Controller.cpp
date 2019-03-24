#include "Controller.h"
#include <thread>
#include <chrono>
#include "Display.h"
#include <string>

bool Controller::init()
{
    // wait for gamepad
	Display::print("Waiting for gamepad...");
	while (true) {
        /* SDL seems somewhat fragile, shut it down and bring it up */
        SDL_Quit();
        SDL_Init(SDL_INIT_JOYSTICK);

		/* poll for gamepad */
		int res = SDL_NumJoysticks();
		if (res > 0) { break; }
		if (res < 0) { Display::print("Err = " + std::to_string(res)); }

		/* yield for a bit */
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	Display::print("Waiting for gamepad...Found one");

	// Open the joystick for reading and store its handle in the joy variable
	_driveJoy = SDL_JoystickOpen(0);
    _launchJoy = SDL_JoystickOpen(1);
	if (!_driveJoy || !_launchJoy) {
		// retry init
		init();
	}

    // Get information about the joystick
	std::string name = (std::string)SDL_JoystickName(_driveJoy);
	int num_axes = SDL_JoystickNumAxes(_driveJoy);
	int num_buttons = SDL_JoystickNumButtons(_driveJoy);
	int num_hats = SDL_JoystickNumHats(_driveJoy);
	Display::print("\nDrive controller " + name + " with:\n"
		+ std::to_string(num_axes) + " axes\n"
		+ std::to_string(num_buttons) + " buttons\n"
		+ std::to_string(num_hats) + " hats\n");

    // Get information about the joystick
	name = (std::string)SDL_JoystickName(_launchJoy);
	num_axes = SDL_JoystickNumAxes(_launchJoy);
	num_buttons = SDL_JoystickNumButtons(_launchJoy);
	num_hats = SDL_JoystickNumHats(_launchJoy);
	Display::print("Launch controller " + name + " with:\n"
		+ std::to_string(num_axes) + " axes\n"
		+ std::to_string(num_buttons) + " buttons\n"
		+ std::to_string(num_hats) + " hats\n");
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
        Display::print("[Controller] Error: Invalid controller in getAxis.");
    }
}

bool Controller::getButton(Controller_t controller, Button_t button)
{
	switch(controller)
    {
    case DRIVE:
		return SDL_JoystickGetButton(_driveJoy, button);
        break;
    case LAUNCH:
        return SDL_JoystickGetButton(_launchJoy, button);
        break;
    default:
        Display::print("[Controller] Error: Invalid controller in getButton.");
    }
}

Controller::~Controller()
{
    SDL_JoystickClose(_driveJoy);
    SDL_JoystickClose(_launchJoy);

    Display::print("[Controller] Gamepads disconnected.");
}
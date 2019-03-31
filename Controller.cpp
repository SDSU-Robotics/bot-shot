#include "Controller.h"
#include <thread>
#include <chrono>
#include <string>
#include "Display.h"

SDL_Joystick *Controller::_driveJoy = nullptr;
SDL_Joystick *Controller::_launchJoy = nullptr;

bool Controller::init()
{
    // wait for gamepad
	Display::debug("Waiting for gamepad...");
	while (true) {
        /* SDL seems somewhat fragile, shut it down and bring it up */
        SDL_Quit();
        SDL_Init(SDL_INIT_JOYSTICK);

		/* poll for gamepad */
		int res = SDL_NumJoysticks();
		if (res > 0) { break; }
		if (res < 0) { Display::debug("Err = " + std::to_string(res)); }

		/* yield for a bit */
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	Display::debug("Waiting for gamepad...Found one");

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
	Display::debug("\nDrive controller " + name + " with:");
	Display::debug(std::to_string(num_axes) + " axes");
	Display::debug(std::to_string(num_buttons) + " buttons");
	Display::debug(std::to_string(num_hats) + " hats");

    // Get information about the joystick
	name = (std::string)SDL_JoystickName(_launchJoy);
	num_axes = SDL_JoystickNumAxes(_launchJoy);
	num_buttons = SDL_JoystickNumButtons(_launchJoy);
	num_hats = SDL_JoystickNumHats(_launchJoy);
	Display::debug("Launch controller " + name + " with:");
	Display::debug(std::to_string(num_axes) + " axes");
	Display::debug(std::to_string(num_buttons) + " buttons");
	Display::debug(std::to_string(num_hats) + " hats");
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
        Display::debug("[Controller] Error: Invalid controller in getAxis.");
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
        Display::debug("[Controller] Error: Invalid controller in getButton.");
    }
}

Controller::~Controller()
{
    SDL_JoystickClose(_driveJoy);
    SDL_JoystickClose(_launchJoy);

    Display::debug("[Controller] Gamepads disconnected.");
}
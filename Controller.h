#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <SDL2/SDL.h>

class Controller
{
private:
    SDL_Joystick *_driveJoy;
    SDL_Joystick *_launchJoy;

public:
    enum Controller_t { DRIVE, LAUNCH };
    enum Axis_t { LEFT_X = 0, LEFT_Y = 1, LEFT_T = 2, RIGHT_X = 3, RIGHT_Y = 4, RIGHT_T = 5};

    bool init();

    float getAxis(Controller_t controller, Axis_t axis);

    ~Controller();
};

#endif
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <SDL2/SDL.h>

class Controller
{
private:
    static SDL_Joystick *_driveJoy;
    static SDL_Joystick *_launchJoy;

public:
    enum Controller_t { DRIVE, LAUNCH };
    enum Axis_t { LEFT_X = 0, LEFT_Y = 1, LEFT_T = 2, RIGHT_X = 3, RIGHT_Y = 4, RIGHT_T = 5 };
    //enum Button_t { A = 0, B = 1, X = 2, Y = 3, LB = 4, RB = 5, SEL = 6, START = 7, T = 8, LJ = 9, RJ = 10 };

    static bool init();

    static int poll(SDL_Event &event) { return SDL_PollEvent(&event); }
    static int poll() { SDL_Event event; return SDL_PollEvent(&event); }

    static float getAxis(Controller_t controller, Axis_t axis);
    static bool getButton(Controller_t controller, SDL_GameControllerButton button);

    ~Controller();
};

#endif
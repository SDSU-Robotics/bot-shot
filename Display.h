#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

const int CONSOLE_WIDTH = 142;
const int CONSOLE_HEIGHT = 50;
const int LABEL_WIDTH = 24;

class Display
{
private:
    static float _rpm;
    static float _launchAngle;
    static float _comAngle;
    static string _debug[10];

    // escape code wrapper functions
    static void clear() { cout << "\033[2J\033[1;1H";}
    static void location(int x, int y) { cout << "\033[" + to_string(x) + ";" + to_string(y) + "f"; }

public:
    static void init();

    static void setRPM(float rpm) {_rpm = rpm;}
    static float getRPM() {return _rpm;}

    static void setLaunchAngle(float launchAngle) {_launchAngle = launchAngle;}
    static float getLaunchAngle() {return _launchAngle;}

    static void setcomAngle(float comAngle) {_comAngle = comAngle;}
    static float getcomAngle() {return _comAngle;}

    static void debug(string message);

    static void update();
};

#endif
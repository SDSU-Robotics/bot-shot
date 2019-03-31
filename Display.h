#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

const int CONSOLE_WIDTH = 142;
const int CONSOLE_HEIGHT = 50;
const int LABEL_WIDTH = 24;
const int DEBUG_LINES = 10;

class Display
{
private:
    static float _rpm;
    static float _launchAngle;
    static float _comAngle;
    static int _enterCount;
    static int _debugCount;
    static string _debug[DEBUG_LINES];

    // escape code wrapper functions
    static void clear() { cout << "\033[2J\033[1;1H";}
    static void location(int x, int y) { cout << "\033[" + to_string(x) + ";" + to_string(y) + "f"; }
    static void shift(int num);

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
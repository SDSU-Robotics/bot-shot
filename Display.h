#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

const int CONSOLE_WIDTH = 142;
const int CONSOLE_HEIGHT = 38;

const int LABEL_WIDTH = 24;

const int VERTICAL_DIVISION = 50;

const int DEBUG_LINES = 10;

class Display
{
private:
    static int _debugCount;
    static string _debug[DEBUG_LINES];

    // escape code wrapper functions
    static void clear() { cout << "\033[2J\033[1;1H";}
    static void location(int x, int y) { cout << "\033[" + to_string(y) + ";" + to_string(x) + "f"; }
    static void shift(int num);

public:
    static void init();

    static void debug(string message);

    static void update();
};

#endif
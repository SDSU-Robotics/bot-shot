#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace std;

const int CONSOLE_WIDTH = 142;
const int CONSOLE_HEIGHT = 38;

const int LABEL_WIDTH = 24;

const int DEBUG_LINES = 25;

class Display
{
private:
    static int _debugCount;
    static string _debug[DEBUG_LINES];
    static int _menuSelection;

    // escape code wrapper functions
    static void clear() { cout << "\033[2J\033[1;1H";}
    static void clearLine() { cout << "\033[2K"; }
    static void location(int x, int y) { cout << "\033[" + to_string(y) + ";" + to_string(x) + "f"; }
    static void underline() { cout << "\033[4m"; }
    static void clearFormatting() { cout << "\033[m"; }
    
    static void shift(int num);

public:
    static void init();

    static void debug(string message);

    static void update();
};

#endif
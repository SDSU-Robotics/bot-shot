#include "Display.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>

#include "Launcher.h"
#include "Arduino.h"
#include "Controller.h"

using namespace std;

int Display::_debugCount = 0;
string Display::_debug[DEBUG_LINES];
int Display::_menuSelection = 0;

void Display::init()
{
	// clear the screen
	clear();

	// top line
	location(1, 1);
	for (int i=0; i < CONSOLE_WIDTH; ++i)
		cout << "-";

	// middle line
	location(1, CONSOLE_HEIGHT - DEBUG_LINES - 1);
	for(int i=0; i < CONSOLE_WIDTH; i++)
		cout << "-";

	// bottom line
	location(1, CONSOLE_HEIGHT);
	for (int i=0; i < CONSOLE_WIDTH; ++i)
		cout << "-";

	location(1, 2);
	cout << "RPM:";
	
	location(1,7);
	cout << "Servo Pos:";
	location(1,8);
	cout << "Servo Angle:";

	location(1, 4);
	cout << "Bottom Enc. RPM:";
	location(1, 5);
	cout << "Top Enc. RPM:";

	location(1, 9);
	cout << "Launcher Angle:";

	_debugCount = 0;

	location(1, CONSOLE_HEIGHT - DEBUG_LINES);
}

void Display::debug(string message)
{
	if(_debugCount >= DEBUG_LINES)
	{
		shift(1);
		_debug[DEBUG_LINES-1] = message;		
	}
	else
	{
		_debug[_debugCount] = message;
		_debugCount++;
	}
	update();
}

void Display::shift(int num)
{
	for(int i = 0; i < DEBUG_LINES-num; ++i)
		_debug[i] = _debug[i+num];
}

void Display::update()
{
	Controller::poll();
	
	// live values
	location(LABEL_WIDTH + 1, 2); cout << setw(4) << right << int(Launcher::getRPM()) << left;

	location(LABEL_WIDTH + 1, 7); cout << setw(3) << right << to_string(Arduino::getServoPos()) << left;
	location(LABEL_WIDTH + 1, 8); cout << Arduino::getServoAngle();

	location(LABEL_WIDTH + 1, 4); cout << Launcher::getBottomEncoderRPM();
	location(LABEL_WIDTH + 1, 5); cout << Launcher::getTopEncoderRPM();

	if (Arduino::isInitialized())
	{
		float angle;
		Arduino::getLauncherAngle(angle);
		location(LABEL_WIDTH + 1, 9); cout << setw(15) << angle;
	}

	if (Controller::getButton(Controller::LAUNCH, Controller::START))
	{
		location(LABEL_WIDTH + 1, 2);
		cout << "    ";
		location(LABEL_WIDTH + 1, 2);
		string input;
		getline(cin, input);
		Launcher::setRPM(stoi(input));
	}

	// reprint debug
	for (int i = 0; i < DEBUG_LINES; i++)
	{
		location(1, CONSOLE_HEIGHT - DEBUG_LINES + i);
		clearLine();
		cout << _debug[i];
	}

	// put any stray prints within debug bounds
	location(1, CONSOLE_HEIGHT - DEBUG_LINES);
}


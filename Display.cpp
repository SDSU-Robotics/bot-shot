#include "Display.h"

#include <iostream>
#include <iomanip>

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

	// vertical divider
	for (int i = 2; i < CONSOLE_HEIGHT - DEBUG_LINES - 1; ++i)
	{
		location(VERTICAL_DIVISION, i);
		cout << "|";
	}

	location(1, 2);
	cout << setw(LABEL_WIDTH) << left << "RPM:";
	
	location(1, 3);
	cout << setw(LABEL_WIDTH) << left << "Launch Angle:";

	location(1, 5);
	cout << setw(LABEL_WIDTH) << left << "Commencement Arm Angle";
	
	location(1,7);
	cout << setw(LABEL_WIDTH) << left << "Servo Pos:";
	location(1,8);
	cout << setw(LABEL_WIDTH) << left << "Servo Angle:";

	_debugCount = 0;

	update();
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
	// live values
	location(LABEL_WIDTH + 1, 2); cout << Launcher::getRPM() << endl;
	location(LABEL_WIDTH + 1, 3); cout << Launcher::getLaunchAngle() << endl;

	location(LABEL_WIDTH + 1, 5); cout << "Unknown" << endl;

	location(LABEL_WIDTH + 1, 7); cout << int(Arduino::getServoPos()) << endl;
	location(LABEL_WIDTH + 1, 8); cout << Arduino::getServoAngle() << endl;
	
	Controller::poll();

	if (Launcher::getLaunchAngleControlMode() == ControlMode::Position)
	{
		if (Controller::getButton(Controller::LAUNCH, Controller::SEL))
		{
			_menuSelection = _menuSelection - 1;
			if (_menuSelection < 0)
				_menuSelection = 0;
		}	

		if (Controller::getButton(Controller::LAUNCH, Controller::START))
		{
			_menuSelection = _menuSelection + 1;
			if (_menuSelection > 1)
				_menuSelection = 1;
		}	
	}
	


	// menu
	if (_menuSelection == 0) underline();
	location(VERTICAL_DIVISION + 3, 2);
	cout << "RPM:";
	clearFormatting();
	
	if (_menuSelection == 1) underline();
	location(VERTICAL_DIVISION + 3, 3);
	cout << "Angle:";
	clearFormatting();

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


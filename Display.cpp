#include "Display.h"

#include <iostream>
#include <iomanip>

#include "Launcher.h"
#include "Arduino.h"

using namespace std;

int Display::_debugCount = 0;
string Display::_debug[DEBUG_LINES];

void Display::init()
{
	// clear the screen
	clear();

	location(1, 1);
	cout << setw(LABEL_WIDTH) << left << "RPM:" << endl << endl;
	
	location(1, 2);
	cout << setw(LABEL_WIDTH) << left << "Launch Angle:" << endl << endl;

	location(1, 4);
	cout << setw(LABEL_WIDTH) << left << "Commencement Arm Angle";
	
	location(1,6);
	cout << setw(LABEL_WIDTH) << left << "Servo Pos:";
	location(1,7);
	cout << setw(LABEL_WIDTH) << left << "Servo Angle:";

	location(1, MIDDLE_DIVISION);
	for(int i=0; i < CONSOLE_WIDTH; i++)
		cout << "-";

	location(1, MIDDLE_DIVISION + DEBUG_LINES + 1);
	for(int i=0; i < CONSOLE_WIDTH; i++)
		cout << "-";

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
	location(LABEL_WIDTH + 1, 1); cout << Launcher::getRPM() << endl;
	location(LABEL_WIDTH + 1, 2); cout << Launcher::getLaunchAngle() << endl;

	location(LABEL_WIDTH + 1, 4); cout << "Unknown" << endl;

	location(LABEL_WIDTH + 1, 6); cout << int(Arduino::getServoPos()) << endl;
	location(LABEL_WIDTH + 1, 7); cout << Arduino::getServoAngle() << endl;

	// clear debug
	location(1, MIDDLE_DIVISION + 1);
	for (int i = 0; i < DEBUG_LINES; i++)
	{
		for(int j=0; j < CONSOLE_WIDTH; j++)
			cout << " ";
		cout << endl;
	}
	
	// reprint debug
	location(1, MIDDLE_DIVISION + 1);
	for (int i = 0; i < DEBUG_LINES - 1; i++)
		cout << _debug[i] << endl;

	// put any stray prints within debug bounds
	location(1, MIDDLE_DIVISION + 1);
}


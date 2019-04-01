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
	clear();
	cout << setw(LABEL_WIDTH) << left << "RPM:" << endl << endl;
	cout << setw(LABEL_WIDTH) << left << "Launch Angle:" << endl << endl;
	cout << setw(LABEL_WIDTH) << left << "Commencement Arm Angle:" << "\n\n\n";
	for(int i=0; i < CONSOLE_WIDTH; i++)
		cout << "-";

	_debugCount = 0;
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
	location(1,LABEL_WIDTH + 1);
	cout << Launcher::getRPM() << endl;

	location(3,LABEL_WIDTH + 1);
	cout << Launcher::getLaunchAngle() << endl;

	location(5,LABEL_WIDTH + 1);
	cout << "Unknown" << endl;

	// clear debug
	location(9,1);
	for (int i = 0; i < DEBUG_LINES; i++)
	{
		for(int j=0; j < CONSOLE_WIDTH; j++)
			cout << " ";
		cout << endl;
	}
	
	// reprint debug
	location(9,1);
	for (int i = 0; i < DEBUG_LINES; i++)
		cout << _debug[i] << endl;

	// put any stray prints within debug bounds
	location(9,1);
}


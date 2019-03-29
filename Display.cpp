#include "Display.h"

#include <iostream>
#include <iomanip>

using namespace std;

float Display::_rpm;
float Display::_launchAngle;
float Display::_comAngle;
int Display::_enterCount;
int Display::_debugCount;
string Display::_debug[10];

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
	if(_debugCount > DEBUG_LINES)
		shift(4);
	else
	{
		_debug[_debugCount] = message;
		_debugCount++;
	}

	cout << message << endl;
}

void Display::shift(int num)
{
	for(int i = 0; i < DEBUG_LINES-num; ++i)
		_debug[i] = _debug[i+num];
}

void Display::update()
{
	location(1,LABEL_WIDTH + 1);
	cout << _rpm << endl;

	location(3,LABEL_WIDTH + 1);
	cout << _launchAngle << endl;

	location(5,LABEL_WIDTH + 1);
	cout << _comAngle << endl;

	location(9,1);
	for (int i = 0; i < DEBUG_LINES; i++)
		cout << _debug[i] << endl;
}


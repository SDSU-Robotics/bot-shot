#include "Display.h"

#include <iostream>
#include <iomanip>

using namespace std;

float Display::_rpm;
float Display::_launchAngle;
float Display::_comAngle;
string Display::_debug[10];

void Display::init()
{
	clear();
	cout << setw(LABEL_WIDTH) << left << "RPM:" << endl << endl;
	cout << setw(LABEL_WIDTH) << left << "Launch Angle:" << endl << endl;
	cout << setw(LABEL_WIDTH) << left << "Commencement Arm Angle:" << "\n\n\n";
	for(int i=0; i < CONSOLE_WIDTH; i++)
		cout << "-";

	for (int i = 0; i < 10; i++)
		_debug[i] = "test" + to_string(i);
}

void Display::debug(string message)
{
	//cout << message << endl;
}

void Display::update()
{
	location(1,LABEL_WIDTH + 1);
	cout << _rpm;

	location(3,LABEL_WIDTH + 1);
	cout << _launchAngle;

	location(5,LABEL_WIDTH + 1);
	cout << _comAngle;

	location(9,1);
	for (int i = 0; i < 10; i++)
		cout << _debug[i];
}


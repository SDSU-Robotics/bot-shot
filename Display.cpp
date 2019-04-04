#include "Display.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

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

	// move up and down menu
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
	else
		_menuSelection = 0;
	

	// print menu
	if (_menuSelection == 0) underline();
	location(VERTICAL_DIVISION + 3, 2);
	cout << "RPM:";
	clearFormatting();
	location(VERTICAL_DIVISION + 11, 2);
	cout << right << setw(4) << int(Launcher::getRPM());
	
	if (_menuSelection == 1) underline();
	location(VERTICAL_DIVISION + 3, 3);
	cout << "Angle:";
	clearFormatting();
	location(VERTICAL_DIVISION + 12, 3);
	cout << right << setw(2) << int(Launcher::getLaunchAngle());

	location(VERTICAL_DIVISION + 9, 2);
	cout << " ";
	location(VERTICAL_DIVISION + 17, 2);
	cout << " ";
	location(VERTICAL_DIVISION + 10, 3);
	cout << " ";
	location(VERTICAL_DIVISION + 16, 3);
	cout << " ";

	int adjustment = 0;

	if (Controller::getButton(Controller::LAUNCH, Controller::A))
	{
		do {
			adjustment = 0;
			if (_menuSelection == 0)
			{
				location(VERTICAL_DIVISION + 9, 2);
				cout << "<";
				location(VERTICAL_DIVISION + 17, 2);
				cout << ">";
				location(VERTICAL_DIVISION + 10, 3);
				cout << " ";
				location(VERTICAL_DIVISION + 16, 3);
				cout << " ";

				if (Controller::getButton(Controller::LAUNCH, Controller::RB))
					adjustment += 5;
				if (Controller::getButton(Controller::LAUNCH, Controller::LB))
					adjustment -= 5;
				if (Controller::getAxis(Controller::LAUNCH, Controller::RIGHT_T) < 0)
					adjustment += 50;
				if (Controller::getAxis(Controller::LAUNCH, Controller::LEFT_T) < 0)
					adjustment -= 50;

				location(VERTICAL_DIVISION + 11, 2);
				cout << right << setw(4) << int(Launcher::getRPM());

				//debug("Adjustment: " + to_string(adjustment));
				Launcher::setRPM(Launcher::getRPM() + adjustment);
			}
			if (_menuSelection == 1)
			{
				location(VERTICAL_DIVISION + 9, 2);
				cout << " ";
				location(VERTICAL_DIVISION + 17, 2);
				cout << " ";
				location(VERTICAL_DIVISION + 10, 3);
				cout << "<";
				location(VERTICAL_DIVISION + 16, 3);
				cout << ">";
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			Controller::poll();
		} while(!Controller::getButton(Controller::LAUNCH, Controller::B));
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


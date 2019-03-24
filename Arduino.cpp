#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include "Arduino.h"
#include "Display.h"

using namespace std;

bool Arduino::init()
{
	//Open up Serial Communication
    _serPort = fopen(_comPort, "r");

	//If communication fails, print error
	if (_serPort == NULL)
	{
		Display::print("[Arduino] Communication Failed!");	
		return false;
	}

	return true;
}

bool Arduino::IMUread(float &com, float &launcher)
{
    char buf[64];
    char _com[7];
    char _launcher[7];
   	int numBytesRead;
   	int error = 0;

	do
	{
		//Wait until start (:) charector is read 
		char ch[1];
		do 
		{
			fread(ch, 1, 1, _serPort);
		} while (ch[0] != ':');

		int i = 0;

		//Now read in all charectors until the new line 
		do 
		{
			fread(ch, 1, 1, _serPort);
			buf[i] = ch[0];
			++i;
		} while (ch[0] != '\n');

		// If i == 16 then gather the two variables
		// Commencement arm IMU followed by launch angle
		if (i == 16)
		{
			//Takes the first 7 charectors
			for (int k = 0; k < 7; ++k)
			{
				_com[k] = buf[k];
			}

			//Skips the 8th charector sent and grabs the next 7 
			for (int j = 8; j < 16; ++j)
			{
				_launcher[j-8] = buf[j];
			}

			//Assigns the charectors to their respective float variables
			com = atof(_com);
			launcher = atof(_launcher);
			return true;
		}
		//if buffer error, the connection is closed and reopened, also throws an error message
		else
		{
			error++;
			fclose(_serPort);
			init();
		}
	} while (error < 5);

	return false;
}


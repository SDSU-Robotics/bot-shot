#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include "Arduino.h"
#includ "Display.h"

using namespace std;

char serialPortFilename[] = "/dev/ttyUSB3";

bool Arduino::IMUread(float &com, float &launcher)
{

	std::cout << "Initialize" << std::endl;
    char buf[1024];
    char _com[7];
    char _launcher[7];
    //float imuYaw;
    //float imuPitch;
   	int numBytesRead;
   	int error = 0;

   	//Open up Serial Communication
    FILE *serPort = fopen(serialPortFilename, "r");

    //If communication fails, print error
	if (serPort == NULL)
	{
		Display::print("[Arduino] Communication Failed!")	
		return 0;
	}

	//Output which serial port opens
	std::cout << serialPortFilename;

	printf(":\n");


	while(1)
	{
		//Wait until start (:) charector is read 
		char ch[1];
		do 
		{
			fread(ch, 1, 1, serPort);
		} while (ch[0] != ':');

		int i = 0;

		//Now read in all charectors until the new line 
		do 
		{
			fread(ch, 1, 1, serPort);
			buf[i] = ch[0];
			++i;
		} while (ch[0] != '\n');

		//If i == 16 then gather the two variables
		//Right now this is Yaw and Pitch
		//In the future this will be IMU1 and IMU2
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

			//Outputs the variables
			//cout << "Commencement Arm Angle: " << com << "\t" << "Launcer Angle: " << imuPitch << "\tWe have had " << error << " error(s)" << endl;
		}

		//if buffer error, the connection is closed and reopened, also throws an error message
		else
		{
			error++;
			//cout << "Error!" << endl;
			fclose(serPort);
			FILE *serPort = fopen(serialPortFilename, "r");
  
			if (serPort == NULL)
			{
				Display::print("[Arduino] Communication Failed!")	
			}
		}
	}

	if (error >= 5)
		return 0;
	else
		return 0;
}


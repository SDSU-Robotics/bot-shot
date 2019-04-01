#include "PixyController.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include "pixy.h"

using namespace std;

bool PixyController::init()
{
	// Connect to Pixy //
	int pixy_init_status = pixy_init();

	// Was there an error initializing pixy? //
	if(!pixy_init_status == 0)
	{
		// Error initializing Pixy //
		printf("pixy_init(): ");
		pixy_error(pixy_init_status);
		return false;
	}

	// Request Pixy firmware version //
	uint16_t major;
	uint16_t minor;
	uint16_t build;
	int      return_value;

	return_value = pixy_get_firmware_version(&major, &minor, &build);

	if (return_value) 
	{
		// Error //
		printf("Failed to retrieve Pixy firmware version. ");
		pixy_error(return_value);
		return false;
	}
	
	// Success //
	printf(" Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
	return true;
}


struct Block PixyController::getLatestBlock()
{
	struct Block block;
	pixy_get_blocks(1, &block);
	return block;
}
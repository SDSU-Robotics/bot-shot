#ifndef PIXYCONTROLLER_H
#define PIXYCONTROLLER_H

#include "pixy.h"

const int IMAGE_WIDTH = 319;
const int BLOCK_BUFFER_SIZE = 1;

class PixyController
{
public:
    static bool init();
    static int getImageW() { return IMAGE_WIDTH; }
    static struct Block getLatestBlock();
};

#endif
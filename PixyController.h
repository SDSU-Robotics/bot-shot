#ifndef PIXYCONTROLLER_H
#define PIXYCONTROLLER_H

#include "pixy.h"

const int IMAGE_WIDTH = 319;
const int BLOCK_BUFFER_SIZE = 1;

class PixyController
{
private:
    // Pixy Block buffer // 
    struct Block _blockBuffer[BLOCK_BUFFER_SIZE];

    struct Block _latestBlock;
    
public:
    void init();
    static int getImageW() { return IMAGE_WIDTH; }
    struct Block getLatestBlock();
};

#endif
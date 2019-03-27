#ifndef PIXYCONTROLLER_H
#define PIXYCONTROLLER_H

#include "pixy.h"

#define BLOCK_BUFFER_SIZE 25

class PixyController
{
private:
    // Pixy Block buffer // 
    struct Block _blockBuffer[BLOCK_BUFFER_SIZE];

    struct Block _latestBlock;
public:
    void init();
    struct Block getLatestBlock();
};

#endif
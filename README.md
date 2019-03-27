Based on https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example from CTR-Electronics.

## CAN USB Adapter
Deploy the SocketCAN firmware to a HERO
- Firmware : https://github.com/CrossTheRoadElec/HERO-STM32F4
- Hardware : http://www.ctr-electronics.com/control-system/hro.html

## Setup

 1. Install git `sudo apt install git`.
 2. Install necessary utilities
     - `sudo apt install can-utils`.
     - `sudo apt install net-tools`.
 3. Install necessary libs to build.
     -  `sudo apt install cmake`
     -  `sudo apt install libsdl2-dev`
     -  `sudo apt-get install libboost-all-dev`
     -  `sudo apt-get install libusb-1.0-0.dev`
 4. Install PixyCam libraries
     - Clone the repository `git clone https://github.com/charmedlabs/pixy.git`.
     - Build the library `cd pixy/scripts; /build_libpixyusb.sh`.
     - Install the library `sudo ./install_libpixyusb.sh`.
     - Clean up `cd ../..; sudo rm -rf pixy`
     - Fix bug in pixy code: add `#include <cstdio>` to `/usr/local/include/pixy.h`
 4. Clone repo into user directory `git clone https://github.com/SDSU-Robotics/bot-shot.git`.
 5. Navigate into repo `cd bot-shot`.

## Building/Running
 1. Run build.sh `./build.sh`
 2. Run program `./run.sh`

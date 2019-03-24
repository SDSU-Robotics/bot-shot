Based on https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example from CTR-Electronics.

Note: Requires libsdl2-dev (sudo apt-get install libsdl2-dev) for USB Gamepad interface on Raspberry PI.

#### CAN USB Adapter
Deploy the SocketCAN firmware to a HERO
- Firmware : https://github.com/CrossTheRoadElec/HERO-STM32F4
- Hardware : http://www.ctr-electronics.com/control-system/hro.html

## Setup

 1. Install CAN tools `sudo apt-get install can-utils`.
 2. Install git `sudo apt-get install git`.
 3. Install necessary libs to build example.
     -  `sudo apt-get install cmake`
     -  `sudo apt-get install libsdl2-dev `
 4. Clone repo into user directory `git clone https://github.com/SDSU-Robotics/bot-shot.git`
 5. Navigate into repo `cd bot-shot`.
 6. Chmod shell scripts to allow you to use them:
     -  `chmod +x build.sh`
     -  `chmod +x clean.sh`
     -  `chmod +x canableStart.sh`
 7. Bring up CAN0 `./canableStart.sh` -> `sudo ifconfig can0 up` 
 8. Run build.sh `./build.sh`
 9. Run program `./bin/run`

 ## Update to g++ 6
 sudo add-apt-repository ppa:ubuntu-toolchain-r/test
 sudo apt update
 sudo apt install gcc-6
 sudo apt install g++-6
 sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6
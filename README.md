Based on https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example from CTR-Electronics.

## CAN USB Adapter
Deploy the SocketCAN firmware to a HERO
- Firmware : https://github.com/CrossTheRoadElec/HERO-STM32F4
- Hardware : http://www.ctr-electronics.com/control-system/hro.html

## Prerequisites

 1. Install necessary tools and utilities
     - `sudo apt install git`
     - `sudo apt install can-utils`
     - `sudo apt install net-tools`
 2. Install ROS Melodic
     - http://wiki.ros.org/melodic/Installation/Ubuntu
     - Perform full desktop installation
 3. Install ROS packages
     - `sudo apt install ros-melodic-joy`
     - `sudo apt install ros-melodic-cv-camera`
 
 ## Setup
 1. Setup catkin workspace
     - `mkdir -p ~/catkin_ws/src`
     - `cd ~/catkin_ws/src`
     - `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
     - `source ~/.bashrc`
 2. Clone repo into user directory `git clone https://github.com/SDSU-Robotics/bot-shot.git`.
 3. Navigate into repo `cd bot-shot`.

## Building
 1. `cd ~/catkin_ws`
 2. `catkin_make`

 ## Running
 1. Laptop on the robot
     - Connect to Robotics wifi
     - `cd ~/catkin_ws`
     - `src/bot-shot/runMaster.sh`
 2. Interface laptop
     - `cd ~/catkin_ws/`
     - `src/bot-shot/runInterface.sh`
    

# Set up CAN communication
sudo ifconfig can0 down
./canableStart.sh

# Run program
rosrun bot-shot DriveBase

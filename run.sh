# Set up CAN communication
sudo ifconfig can0 down
./canableStart.sh

# Run program
source ~/catkin_ws/devel/setup.bash
roscore &
rosrun joy joy_node &
rosrun bot-shot DriveBase &
rosrun bot-shot controller &
wait

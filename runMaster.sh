# Set up CAN communication
sudo ifconfig can0 down
./canableStart.sh

# Set up ROS parameters
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER=http://localhost:11311/
export ROS_HOSTNAME=192.168.1.100
export ROS_IP=192.168.1.100

# Run the necessary nodes
roscore &
rosrun bot-shot DriveBase &
rosrun bot-shot controller &
wait

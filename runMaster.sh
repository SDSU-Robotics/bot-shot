# Set up CAN communication
sudo ifconfig can0 down
src/bot-shot/canableStart.sh
sleep 2

# Set up ROS parameters
killall -9 rosmaster
sleep 1
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_HOSTNAME=192.168.1.100
export ROS_IP=192.168.1.100

# Run the necessary nodes
roscore &
sleep 2
rosrun bot-shot Controller &
rosrun bot-shot DriveBase &
rosrun bot-shot Launcher &
rosparam set cv_camera/device_id 1
rosparam set cv_camera/image_width 1920
rosparam set cv_camera/image_height 720
rosrun cv_camera cv_camera_node
wait

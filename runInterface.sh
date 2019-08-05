# Set up ROS parameters
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_HOSTNAME=192.168.1.101
export ROS_IP=192.168.1.101

# Run the necessary nodes
killall -9 joy_node
killall -9 python
sleep 2
rosrun joy joy_node &
rosrun bot-shot WebCamViewer.py &
rosrun bot-shot PhysicsModel.py &
rosrun bot-shot Interface
wait
killall -9 python
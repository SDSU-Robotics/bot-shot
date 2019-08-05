# Set up ROS parameters
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_HOSTNAME=192.168.1.102
export ROS_IP=192.168.1.102

# Run the necessary nodes
pkill -9 -f WebCamViewer.py
wait(2)
rosrun joy joy_node &
rosrun bot-shot WebCamViewer.py &
rosrun bot-shot PhysicsModel.py &
rosrun bot-shot Interface

wait


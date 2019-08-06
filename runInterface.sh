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
<<<<<<< HEAD
wait
=======
wait
killall -9 python
>>>>>>> 1eb55f7bb82304130352f9e48257e7e765d85aa0

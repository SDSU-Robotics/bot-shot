# --------- Master -------

# Set up CAN communication
sudo ifconfig can0 down
~/catkin_ws/src/bot-shot/canableStart.sh
#~/catkin_ws/src/bot-shot/canableStart.sh
sleep 2

# Kill old processes
killall -9 rosmaster
killall -9 Controller
killall -9 DriveBase
killall -9 Launcher
killall -9 cv_camera_node
killall -9 joy_node
killall -9 python

sleep 2
source ~/catkin_ws/devel/setup.bash

# Run the necessary nodes
roscore &
sleep 5
rosrun bot-shot DriveBase &
rosrun bot-shot Launcher &
rosparam set cv_camera/rate 5
rosparam set cv_camera/device_id 1
rosparam set cv_camera/image_width 600
rosparam set cv_camera/image_height 300
rosrun cv_camera cv_camera_node

# --------- inteface 
# Run the necessary nodes
sleep 2
rosrun joy joy_node &
rosrun bot-shot WebCamViewer.py &
rosrun bot-shot PhysicsModel.py &
gnome-terminal -x sh  -c "rosrun bot-shot Interface"
wait
killall -9 python

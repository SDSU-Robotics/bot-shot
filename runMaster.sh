# Set up CAN communication
sudo ifconfig can0 down
src/bot-shot/canableStart.sh
sleep 2

# Set up ROS parameters
killall -9 rosmaster
killall -9 roscore
killall -9 Controller
killall -9 DriveBase
killall -9 Launcher
killall -9 cv_camera_node
killall -9 usb_cam_node
sleep 2
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_HOSTNAME=192.168.1.100
export ROS_IP=192.168.1.100

# Run the necessary nodes
roscore &
sleep 2
#rosrun bot-shot Controller &
#rosrun bot-shot DriveBase &
#rosrun bot-shot Launcher &
#rosparam set cv_camera/rate 5
#rosparam set cv_camera/device_id 3
sleep 2
#rosparam set cv_camera/image_width 600
#rosparam set cv_camera/image_height 300
#rosrun cv_camera cv_camera_node _device_id:=2
#rosrun cv_camera cv_camera_node _device_id:=1

#rosparam set usb_cam/image_width 600
#rosparam set usb_cam/image_height 300

rosparam set usb_cam/camera_frame_id head_camera
rosparam set usb_cam/video_device 0
rosparam set usb_cam/pixel_format yuyv
rosrun usb_cam usb_cam_node

wait

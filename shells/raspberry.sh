sudo ntpdate NTP-server-host
export ROS_IP=192.168.1.102
export ROS_MASTER_URI=http://192.168.1.167:11311
source devel/setup.bash

roslaunch launch run_raspberry.launch

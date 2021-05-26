cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.zsh
rosmsg show node_bridge_ros/gameResult 

roslaunch node_bridge_ros node_bridge.launch
xterm -hold -e "source /opt/ros/noetic/setup.bash; source robomaster_ws/devel/setup.bash; roslaunch robomaster_driver robomaster_driver.launch" && /bin/bash &
sleep 4
xterm -hold -e "source /opt/ros/noetic/setup.bash; source ../ORB_SLAM3_ROS/catkin_ws/devel/setup.bash; roslaunch orb_slam3_ros cam_mono.launch" && /bin/bash &
xterm -hold -e "source /opt/ros/noetic/setup.bash; source ros_ws/devel/setup.bash; rosrun driver_node controller" && /bin/bash &


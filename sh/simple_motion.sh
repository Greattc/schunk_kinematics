roslaunch schunk_gazebo schunk_gazebo.launch &
sleep 10
roslaunch schunk_controllers schunk_trajectory_controller.launch &
sleep 5
rosrun schunk_kinematics simple_motion


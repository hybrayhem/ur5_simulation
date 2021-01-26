Run Gazebo demo:
cd <workspace>/
source devel/setup.bash
roslaunch ur_gazebo ur5.launch limited:=true
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
roslaunch ur5_moveit_config moveit_rviz.launch config:=true

On code changes:
catkin build <packagename>
source devel/setup.bash

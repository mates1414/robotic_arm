# gripper control, directly connected to ros computer
dmesg | grep tty 
    # check USB port of gripper
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# ur5 + robotiq 85 simulation
roslaunch icl_ur5_setup_gazebo icl_ur5_gripper_noetic.launch 
roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch sim:=true
roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true
# or 
roslaunch icl_ur5_setup_gazebo ur5_gripper_simulation.launch

# apriltag
roslaunch icl_ur5_setup_bringup apriltag.launch
rosrun icl_ur5_setup_bringup pick_and_place_task.py

# ur5 + robotiq 85 hardware
roslaunch icl_ur5_setup_bringup ur5_gripper_noetic.launch 
roslaunch icl_ur5_setup_moveit_config ur5_gripper_moveit_planning_execution.launch
roslaunch icl_ur5_setup_moveit_config moveit_rviz.launch config:=true

# to do
   
-   camera
-   orientation and default pick and place
-      
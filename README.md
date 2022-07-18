# RBE-501-Project

## Step 1:

Add the fridge_bot folder to your src of ROS workspace

## Step 2:
### Run the following commands:
1. `roslaunch fridge_bot gazebo.launch`
2. `roslaunch fridge_bot fridge_bot_control.launch`

## Step 3:
### Open Matlab and Run the following Scripts:
### (Please Make sure you have Matlab Robotics Toolbox and Matlab ROS Toolbox)
1. `matlab_ros_bridge.m`
### Position Control Script
2. `Position_Control.m`
## To view in Rviz run the following command:

`roslaunch fridge_bot display.launch`

## To Compare Torques
1. Run Position_control.m
2. save the variables collected
3. import them into 'Final_project_2.m'
4. change the array size variable in Final_project_2.m to be the size of the data imported
5. Run 'Final_project_2.m'

## To view in Rviz run the following command:

`roslaunch fridge_bot display.launch`

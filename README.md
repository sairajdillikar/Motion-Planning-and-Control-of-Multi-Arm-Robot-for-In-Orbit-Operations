# Motion Planning and Control of Multi-Arm Robot for In-Orbit Operations
Private Github repository for Individual Research Project

# Build the Package

````
mkdir -p test3_ws/src
cd test3_ws/src
catkin_init_workspace
git clone https://github.com/sairajdillikar/Motion_Planning_and_Control_of_Multi_Arm_Robot_for_In_Orbit_Operations.git
cd ..
catkin build
````

# Modifications to be made in the SRS spawning package

Before launching the gazebo world, navigate through the packages to find the `src/simplified_df_bot_description/src/exe_horizontal_coordmotion_1.cpp` file and make sure to add your PC Username within the `<uri>` tag. Same goes for the `src/exe_vertical_coordmotion_1.cpp` and `src/exe_combined_coordmotion_1.cpp`


    <uri>/home/ $USERNAME /test3_ws/src/srs_modules_description/meshes/base_link.stl</uri>

# Launch the Gazebo Environment

## Horizontal locomotion.

    cd test3_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_combined_locomotion.launch

## Vertical locomotion.

    cd test3_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_combined_locomotion.launch

## Combined locomotion (both horizontal and vertical).

    cd test3_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_combined_locomotion.launch







# dual_arm_demo

![dual arm](https://github.com/bi3ri/dual_arm_demo/blob/main/dual_arm_demo.gif?raw=true)

Small demo project planning and executing random poses for two UR robots simultaneously. 

Tested successfully under noetic.

## Installation
```bash
mkdir -p ~/dual_arm_ws/src
cd ~/dual_arm_ws/src
git clone https://github.com/bi3ri/dual_arm_demo.git
git submodule update --init --recursive
catkin build && source ~/dual_arm_ws/devel/setup.bash

#run in two shells
roslaunch dual_arm_demo app.launch 
rosrun dual_arm_demo moveit_demo
```

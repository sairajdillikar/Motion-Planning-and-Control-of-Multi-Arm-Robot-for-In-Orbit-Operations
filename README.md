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

Before launching the gazebo world, navigate through the packages to find the `src/simplified_df_bot_description/src/moveit_demo3.cpp` file and make sure to add your PC Username within the `<uri>` tag.

    <uri>/home/ $USERNAME /test3_ws/src/srs_modules_description/meshes/base_link.stl</uri>

# Launch the Gazebo Environment

## To run the combined locomotion (both horizontal and vertical).

    cd test3_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_combined_locomotion.launch

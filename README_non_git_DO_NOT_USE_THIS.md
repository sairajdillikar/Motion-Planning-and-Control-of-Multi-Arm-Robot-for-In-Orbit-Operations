# Motion Planning and Control of Multi-Arm Robot for In-Orbit Operations


# Build the Package

````
mkdir -p mario_mpcc_ws
cd mario_mpcc_ws
````

copy the src folder here, then perform catkin build.

````
catkin build
````

# Modifications to be made in the SRS spawning package

Before launching the gazebo world, navigate through the packages to find the `src/simplified_df_bot_description/src/exe_serial_coordmotion_1.cpp` file and make sure to add your PC Username within the `<uri>` tag. Same goes for the `src/exe_serial_coordmotion_2.cpp`, `src/exe_parallel_coordmotion_1.cpp`, and `src/exe_combined_coordmotion_1.cpp`


    <uri>/home/ $USERNAME /mario_mpcc_ws/src/srs_modules_description/meshes/base_link.stl</uri>


# Launch the Gazebo Environment

## Execute Serial (a) locomotion.

    cd mario_mpcc_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_serial_locomotion.launch
    rosrun simplified_df_bot_description exe_serial_coordmotion_1

## Execute Serial (b) locomotion.

    cd mario_mpcc_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_serial_locomotion.launch
    rosrun simplified_df_bot_description exe_serial_coordmotion_2

## Execute Parallel locomotion.

    cd mario_mpcc_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_parallel_locomotion.launch
    rosrun simplified_df_bot_description exe_parallel_coordmotion_1

## Execute Combined locomotion (both serial and parallel).

    cd mario_mpcc_ws
    source devel/setup.bash
    roslaunch simplified_df_bot_description execute_combined_locomotion.launch
    rosrun simplified_df_bot_description exe_combined_coordmotion_1
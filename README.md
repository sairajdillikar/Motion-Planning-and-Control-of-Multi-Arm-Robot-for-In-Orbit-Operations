# Motion Planning and Control of Multi-Arm Robot for In-Orbit Operations
A ROS-based simulation showcasing the motion planning and control framework for a multi-manipulator system for in-orbit operations.

Developed as part of MSc thesis titled "Motion Planning and Control of Multi-Arm Robot for In-Orbit Operations" 
by ```Sairaj R Dillikar (MSc Robotics 2022-23, Cranfield University, United Kingdom)```

# Build the Package

````
mkdir -p mario_mpcc_ws/src
cd mario_mpcc_ws/src
catkin_init_workspace
git clone https://github.com/sairajdillikar/Motion_Planning_and_Control_of_Multi_Arm_Robot_for_In_Orbit_Operations.git
cd ..
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


## Citation

If you use the Motion Planning architecture as part of a publication, please use the Bibtex below as a citation:

```bibtex
@software{dillikar_sairaj_2023,
  author       = {Dillikar, Sairaj.R.},
  title        = {Motion Planning and Control of Multi-Arm Robot for In-Orbit Operations},
  month        = Sep,
  year         = 2023,
  note         = {{If you use this package, please cite it using the 
                   metadata from this file.}},
  url          = {https://github.com/sairajdillikar/Motion-Planning-and-Control-of-Multi-Arm-Robot-for-In-Orbit-Operations}
}
```

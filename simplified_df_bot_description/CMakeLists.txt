cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW) # Set policy for project() command
project(simplified_df_bot_description VERSION 0.0.1)
# project(simplified_df_bot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  tf
  xacro
  gazebo_ros
  # gazebo_ros_link_attacher
  moveit_core
  moveit_ros_planning
  pluginlib
  geometric_shapes
  moveit_fake_controller_manager
  moveit_ros_planning_interface
)


find_package( Eigen3 REQUIRED )
find_package(moveit_visual_tools REQUIRED)
find_package(gazebo REQUIRED)
find_package(roslaunch)
find_package(gazebo_ros_link_attacher REQUIRED)

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)




catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    message_runtime
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
 ${Boost_INCLUDE_DIR} 
 ${catkin_INCLUDE_DIRS} 
 ${GAZEBO_INCLUDE_DIRS}
 ${moveit_visual_tools_INCLUDE_DIRS}
 ${gazebo_ros_link_attacher_INCLUDE_DIRS}
#  /home/peraspera/test2_ws/src/gazebo_ros_link_attacher
)

link_directories(${GAZEBO_LIBRARY_DIRS})


# include_directories(${gazebo_ros_link_attacher_INCLUDE_DIRS})


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fusion2urdf.cpp
# )


add_executable(moveit_demo src/moveit_demo.cpp)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fusion2urdf_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(moveit_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(moveit_demo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(moveit_demo3 src/moveit_demo3.cpp)
add_dependencies(moveit_demo3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_demo3
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gazebo_common
  gazebo_transport
  # gazebo_ros_link_attacher
  # moveit_visual_tools
  # rviz_visual_tools
)


add_executable(move_horizontal_5SRS src/move_horizontal_5SRS.cpp)
add_dependencies(move_horizontal_5SRS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_horizontal_5SRS
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(move_spiral_square src/move_spiral_square.cpp)
add_dependencies(move_spiral_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_spiral_square
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tf_move_horizontal_5SRS src/tf_move_horizontal_5SRS.cpp)
add_dependencies(tf_move_horizontal_5SRS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_move_horizontal_5SRS
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(srs_broadcaster src/srs_broadcaster.cpp)
add_dependencies(srs_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(srs_broadcaster
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tf_listen_move_horizontal_5SRS src/tf_listen_move_horizontal_5SRS.cpp)
add_dependencies(tf_listen_move_horizontal_5SRS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listen_move_horizontal_5SRS
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tf_listen_move_horizontal_5SRS_v2 src/tf_listen_move_horizontal_5SRS_v2.cpp)
add_dependencies(tf_listen_move_horizontal_5SRS_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listen_move_horizontal_5SRS_v2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_executable(tf_listen_move_horizontal_5SRS_v3 src/tf_listen_move_horizontal_5SRS_v3.cpp)
add_dependencies(tf_listen_move_horizontal_5SRS_v3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_listen_move_horizontal_5SRS_v3
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_horizontal_5SRS_1 src/main_horizontal_5SRS_1.cpp)
add_dependencies(main_horizontal_5SRS_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_horizontal_5SRS_1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(main_horizontal_5SRS_2 src/main_horizontal_5SRS_2.cpp)
add_dependencies(main_horizontal_5SRS_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_horizontal_5SRS_2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(main_horizontal_5SRS_3 src/main_horizontal_5SRS_3.cpp)
add_dependencies(main_horizontal_5SRS_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_horizontal_5SRS_3
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_horizontal_5SRS_4 src/main_horizontal_5SRS_4.cpp)
add_dependencies(main_horizontal_5SRS_4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_horizontal_5SRS_4
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_horizontal_5SRS_5 src/main_horizontal_5SRS_5.cpp)
add_dependencies(main_horizontal_5SRS_5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_horizontal_5SRS_5
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_vertical_6SRS_1 src/main_vertical_6SRS_1.cpp)
add_dependencies(main_vertical_6SRS_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_vertical_6SRS_1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(main_vertical_6SRS_2 src/main_vertical_6SRS_2.cpp)
add_dependencies(main_vertical_6SRS_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_vertical_6SRS_2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_vertical_6SRS_3 src/main_vertical_6SRS_3.cpp)
add_dependencies(main_vertical_6SRS_3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_vertical_6SRS_3
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(main_vertical_6SRS_4 src/main_vertical_6SRS_4.cpp)
add_dependencies(main_vertical_6SRS_4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main_vertical_6SRS_4
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)





add_executable(exe_horizontal_coordmotion_1 src/exe_horizontal_coordmotion_1.cpp)
add_dependencies(exe_horizontal_coordmotion_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exe_horizontal_coordmotion_1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(exe_horizontal_coordmotion_2 src/exe_horizontal_coordmotion_2.cpp)
add_dependencies(exe_horizontal_coordmotion_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exe_horizontal_coordmotion_2
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_executable(exe_vertical_coordmotion_1 src/exe_vertical_coordmotion_1.cpp)
add_dependencies(exe_vertical_coordmotion_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exe_vertical_coordmotion_1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(exe_combined_coordmotion_1 src/exe_combined_coordmotion_1.cpp)
add_dependencies(exe_combined_coordmotion_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exe_combined_coordmotion_1
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(PROGRAMS
  scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS moveit_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS moveit_demo3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS move_horizontal_5SRS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS move_spiral_square
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS tf_move_horizontal_5SRS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS srs_broadcaster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS tf_listen_move_horizontal_5SRS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS tf_listen_move_horizontal_5SRS_v2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS tf_listen_move_horizontal_5SRS_v3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS main_horizontal_5SRS_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS main_horizontal_5SRS_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS main_horizontal_5SRS_3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS main_horizontal_5SRS_4
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS main_horizontal_5SRS_5
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS main_vertical_6SRS_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS main_vertical_6SRS_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS main_vertical_6SRS_3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS main_vertical_6SRS_4
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)





install(TARGETS exe_horizontal_coordmotion_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS exe_horizontal_coordmotion_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS exe_vertical_coordmotion_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS exe_combined_coordmotion_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)





## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  # myfile1
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install the moveit_visual_tools and rviz_visual_tools libraries
# install(TARGETS moveit_visual_tools rviz_visual_tools
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fusion2urdf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

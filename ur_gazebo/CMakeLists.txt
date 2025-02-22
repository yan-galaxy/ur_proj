cmake_minimum_required(VERSION 3.0.2)

project(ur_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES probot_demo
   CATKIN_DEPENDS roscpp rospy moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(moveit_demo src/moveit_demo.cpp)
target_link_libraries(moveit_demo ${catkin_LIBRARIES})

add_executable(moveit_fk_demo src/moveit_fk_demo.cpp)
target_link_libraries(moveit_fk_demo ${catkin_LIBRARIES})

add_executable(add_boundary src/add_boundary.cpp)
target_link_libraries(add_boundary ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_ur3.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur3e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur16e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur20.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur30.xml)
endif()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

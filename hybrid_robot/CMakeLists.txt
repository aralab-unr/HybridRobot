cmake_minimum_required(VERSION 2.8.3)
project(hybrid_robot)


add_compile_options(-std=c++11)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find system dependencies
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
link_directories(${Eigen3_LIBRARY_DIRS})
add_definitions(${Eigen3_DEFINITIONS})

find_package(Boost REQUIRED)

# find_package(OpenCV REQUIRED)

# find catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  rospy
  cmake_modules
  geometry_msgs
  eigen_conversions
  tf
  urdf
  moveit_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  joy
  serial
)

add_message_files(
  FILES
  controllerCheck.msg
)

# add_definitions("-DUSE_ROS")

## Generate services in the 'srv' folder -> this one must be above generate_messages and catkin_package
add_service_files(
  FILES magarr_controller.srv image_cmd.srv pointcloud_cmd.srv mp_service.srv DynamixelCommand.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
# generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hybridRobot
  CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs moveit_msgs trajectory_msgs moveit_core moveit_ros_planning  moveit_ros_planning_interface 
  DEPENDS Eigen PCL Boost
)
# do not wildcard install files since the root folder of the package will contain a debian folder for releasing

#####################################################################################
# Build
#####################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

add_executable(hybridRobot src/hybridRobot.cpp)
# This line tells catkin to first build the messages in turtlesim before trying to build this node.
# That is because we rely on the turtlesim/Pose message, which becomes a header file after it's compiled
add_dependencies(hybridRobot ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} hybridRobot_gencpp)

# This links up the node with the compiled libraries catkin knows about.
# Basically, it allows the node to leverage ROS libraries.
target_link_libraries(hybridRobot ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pcl_server src/pcl_server.cpp)
add_dependencies(pcl_server ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} hybridRobot_gencpp)
target_link_libraries(pcl_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mp_server src/mp_server.cpp)
add_dependencies(mp_server ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} hybridRobot_gencpp)
target_link_libraries(mp_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(main_manual src/main_manual.cpp)
add_dependencies(main_manual ${${PPROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} hybridRobot_gencpp)
target_link_libraries(main_manual ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
################################################################################
# Install
################################################################################
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(PROGRAMS
#   scripts/create_udev_rules
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(FILES 99-open-manipulator-cdc.rules
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

################################################################################
# Test
################################################################################

cmake_minimum_required(VERSION 3.0.2)
project(rising_manipulation)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
	actionlib
	moveit_core
	moveit_ros_planning_interface
  roscpp
  std_msgs
	rosparam_shortcuts
  tf2_ros
  roscpp
  moveit_msgs
  rospy
  #topp_ros
  scene_manager
  moveit_visual_tools
  rviz_visual_tools
  rcomponent
  rising_manipulation_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS
 	roscpp
  rospy
  scene_manager
  #topp_ros
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(object_builder
 #src/object_builder.cpp
#)

#add_library(scene_manager
# src/scene_manager.cpp
#  src/object_builder.cpp
#)


# Declare a C++ executable

add_executable(manipulation_app_node 
              src/manipulation_app_node.cpp 
              src/manipulation_app.cpp
)

#catkin_install_python(PROGRAMS
#  scripts/manipulation_demo_python.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


                      


add_dependencies(manipulation_app_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(manipulation_app_node
                      ${catkin_LIBRARIES}
)



install(
  TARGETS manipulation_app_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
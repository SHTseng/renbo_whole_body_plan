cmake_minimum_required(VERSION 2.8.3)

project(renbo_whole_body_plan)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hrl_kinematics
  moveit_core
  moveit_ros_planning
  rrt_planner_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  trac_ik_lib
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    moveit_core 
    moveit_ros_planning_interface 
    hrl_kinematics moveit_ros_planning 
    moveit_visual_tools 
    rviz_visual_tools 
    rrt_planner_msgs 
    trac_ik_lib
)

include_directories(
  include 
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

#Project Files
set(rrt_connect_planner_files
  src/stable_config_generator.cpp
  src/rrt_connect_planner.cpp
  src/renbo_planner.cpp
  src/double_support_constraint.cpp
  src/final_pose_planner.cpp

  include/renbo_whole_body_plan/stable_config_generator.h
  include/renbo_whole_body_plan/rrt_connect_planner.h
  include/renbo_whole_body_plan/renbo_planner.h
  include/renbo_whole_body_plan/double_support_constraint.h
  include/renbo_whole_body_plan/final_pose_planner.h

)


add_library(${PROJECT_NAME} ${rrt_connect_planner_files})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${PROJECT_NAME})

add_executable(whole_body_plan_demo src/whole_body_plan_demo.cpp)
target_link_libraries(whole_body_plan_demo ${PROJECT_NAME})

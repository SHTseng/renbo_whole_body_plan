cmake_minimum_required(VERSION 2.8.3)
project(renbo_whole_body_plan)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hrl_kinematics
  moveit_core
  rrt_planner_msgs
  renbo_msgs
  moveit_visual_tools
  rviz_visual_tools
  trac_ik_lib
)

find_package(PCL 1.7 REQUIRED)

set(EIGEN3_INCLUDE_DIR ${DRAKE_PATH}/include/eigen3)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    hrl_kinematics
    moveit_visual_tools
    rviz_visual_tools
    rrt_planner_msgs
    renbo_msgs
    trac_ik_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(
    ${catkin_LIBRARY_DIRS}
)

#Project Files
set(rrt_connect_planner_files
  src/stable_config_generator.cpp
  src/rrt_connect_planner.cpp
  src/renbo_planner.cpp
  src/double_support_constraint.cpp
  src/final_pose_planner.cpp
	src/stability_checker.cpp
#  src/nlp_ik_solver.cpp

  include/renbo_whole_body_plan/stable_config_generator.h
  include/renbo_whole_body_plan/rrt_connect_planner.h
  include/renbo_whole_body_plan/renbo_planner.h
  include/renbo_whole_body_plan/double_support_constraint.h
  include/renbo_whole_body_plan/final_pose_planner.h
	include/renbo_whole_body_plan/stability_checker.h
 # include/renbo_whole_body_plan/nlp_ik_solver.h
)

add_library(${PROJECT_NAME} ${rrt_connect_planner_files})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${PROJECT_NAME})

add_executable(whole_body_plan_demo demo/whole_body_plan_demo.cpp)
target_link_libraries(whole_body_plan_demo ${PROJECT_NAME})

add_executable(compute_com src/util/compute_com.cpp)
target_link_libraries(compute_com ${PROJECT_NAME})

add_executable(generate_ss_database src/util/generate_ss_database.cpp)
target_link_libraries(generate_ss_database ${PROJECT_NAME})

add_executable(plan_single_path demo/plan_single_path.cpp)
target_link_libraries(plan_single_path ${PROJECT_NAME})

add_executable(generate_valid_config demo/generate_valid_config.cpp)
target_link_libraries(generate_valid_config ${PROJECT_NAME})

add_executable(generate_grasping_posture demo/generate_grasping_posture.cpp)
target_link_libraries(generate_grasping_posture ${PROJECT_NAME})

add_executable(visualize_trajectory demo/visualize_trajectory.cpp)
target_link_libraries(visualize_trajectory ${PROJECT_NAME})

add_executable(ss_motion_plan demo/ss_motion_plan.cpp)
target_link_libraries(ss_motion_plan ${PROJECT_NAME})



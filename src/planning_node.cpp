#include <ros/ros.h>

#include <renbo_whole_body_plan/renbo_planner.h>

using namespace planner_control;


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "planning_node");

  ros::NodeHandle nh;

  RenboPlanner renbo_planner_;

  ros::ServiceServer generate_ds_database_ = nh.advertiseService("generate_ds_database", &RenboPlanner::generate_ds_database, &renbo_planner_);

  ros::ServiceServer sc_generator_test = nh.advertiseService("sc_generator_test", &RenboPlanner::sc_generator_test, &renbo_planner_);

  ros::ServiceServer rrt_planner_test = nh.advertiseService("rrt_planner_test", &RenboPlanner::rrt_planner_test, &renbo_planner_);

  ros::ServiceServer final_pose_planning_ = nh.advertiseService("final_pose_planning", &RenboPlanner::final_pose_planning, &renbo_planner_);

  ros::ServiceServer pick_place_motion_plan = nh.advertiseService("pick_place_motion_plan", &RenboPlanner::pick_place_motion_plan, &renbo_planner_);

  ros::spin();

  return 0;
}


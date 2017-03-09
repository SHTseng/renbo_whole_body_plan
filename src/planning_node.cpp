#include <ros/ros.h>

#include <renbo_whole_body_plan/renbo_planner.h>

using namespace planner_control;


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "planning_node");

  ros::NodeHandle nh;

  RenboPlanner renbo_planner_;

  ros::ServiceServer generate_ds_database_ = nh.advertiseService("generate_ds_database", &RenboPlanner::generate_ds_database, &renbo_planner_);

  ros::ServiceServer compute_robot_com = nh.advertiseService("compute_robot_com", &RenboPlanner::compute_robot_com, &renbo_planner_);

  ros::ServiceServer plan_single_target = nh.advertiseService("plan_single_target", &RenboPlanner::BiRRT_motion_plan, &renbo_planner_);

  ros::ServiceServer final_pose_planning_ = nh.advertiseService("final_pose_planning", &RenboPlanner::final_pose_planning, &renbo_planner_);

  ros::ServiceServer generate_whole_body_posture = nh.advertiseService("generate_whole_body_posture",
                                                                       &RenboPlanner::generate_whole_body_posture, &renbo_planner_);

  ros::ServiceServer pick_place_motion_plan = nh.advertiseService("pick_place_motion_plan", &RenboPlanner::pick_place_motion_plan, &renbo_planner_);

  ros::ServiceServer multi_goal_rrt_planner = nh.advertiseService("multi_goal_rrt_planner", &RenboPlanner::multi_goal_rrt_planner, &renbo_planner_);

  ros::spin();

  return 0;
}


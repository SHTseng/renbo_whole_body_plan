#include <ros/ros.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <renbo_msgs/compute_motion_plan.h>
#include <renbo_msgs/generate_whole_body_posture.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "whole_body_motion_plan");

  ros::NodeHandle nh_;

  int ACTIVE_SERVICE = 0, SCENERIO = 0;
  if (argc == 2)
  {
    std::stringstream s(argv[1]);
    s >> SCENERIO;
  }
  else if (argc == 3)
  {
    ACTIVE_SERVICE = std::atoi(argv[1]);
    SCENERIO = std::atoi(argv[2]);
  }

  ros::ServiceClient generate_posture = nh_.serviceClient<renbo_msgs::generate_whole_body_posture>("generate_whole_body_posture");
  renbo_msgs::generate_whole_body_posture generate_posture_srv;
  generate_posture_srv.request.scenario = SCENERIO;

  std::vector<double> goal_config;
  if (generate_posture.call(generate_posture_srv))
  {
    ROS_INFO_STREAM("generate whole-body posture return state: " << generate_posture_srv.response.success);
    goal_config = generate_posture_srv.response.solved_config;
  }

  ros::ServiceClient plan_single_target = nh_.serviceClient<renbo_msgs::compute_motion_plan>("plan_single_target");
  renbo_msgs::compute_motion_plan plang_single_target_srv;

  plang_single_target_srv.request.scenario = SCENERIO;

  std::vector<double> intial_config(27);
  plang_single_target_srv.request.initial_config = intial_config;
  plang_single_target_srv.request.goal_config = goal_config;

  if (plan_single_target.call(plang_single_target_srv))
  {
    ROS_INFO_STREAM("BiRRT planner return state: " << plang_single_target_srv.response.success);
  }
  else
  {
    ROS_ERROR("plan single target fail");
    exit(1);
  }

  ros::shutdown();

  return 0;

}

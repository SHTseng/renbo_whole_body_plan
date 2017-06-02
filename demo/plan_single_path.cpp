#include <ros/ros.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <renbo_msgs/compute_motion_plan.h>
#include <renbo_msgs/generate_whole_body_posture.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "plan_single_path");

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

  std::vector<double> goal_config, goal_config_b;
  if (generate_posture.call(generate_posture_srv))
  {
    ROS_INFO_STREAM("generate whole-body posture return state: " << (int)generate_posture_srv.response.success);
    goal_config = generate_posture_srv.response.solved_config_a;
    goal_config_b = generate_posture_srv.response.solved_config_b;
  }
  else
  {
    ROS_INFO_STREAM("generate whole-body posture return state: " << (int)generate_posture_srv.response.success);
    exit(1);
  }

  ros::ServiceClient plan_single_target = nh_.serviceClient<renbo_msgs::compute_motion_plan>("BiRRT_motion_planner");
  renbo_msgs::compute_motion_plan plang_single_target_srv;

  plang_single_target_srv.request.scenario = SCENERIO;

  std::vector<double> intial_config(29);
  plang_single_target_srv.request.initial_config = intial_config;
  plang_single_target_srv.request.goal_config = goal_config;

  if (plan_single_target.call(plang_single_target_srv))
  {
    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
  }
  else
  {
    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
    exit(1);
  }
//  plang_single_target_srv.request.initial_config = goal_config;
//  plang_single_target_srv.request.goal_config = goal_config_b;
//  plang_single_target_srv.request.attach_object = 1;

//  if (plan_single_target.call(plang_single_target_srv))
//  {
//    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
//  }
//  else
//  {
//    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
//    exit(1);
//  }

//  plang_single_target_srv.request.initial_config = goal_config;
//  plang_single_target_srv.request.goal_config = intial_config;
//  plang_single_target_srv.request.attach_object = 0;

//  if (plan_single_target.call(plang_single_target_srv))
//  {
//    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
//  }
//  else
//  {
//    ROS_INFO_STREAM("BiRRT planner return state: " << (int)plang_single_target_srv.response.success);
//    exit(1);
//  }


  ros::shutdown();

  return 0;

}

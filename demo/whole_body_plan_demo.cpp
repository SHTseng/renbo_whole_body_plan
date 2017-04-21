#include <ros/ros.h>

#include <rrt_planner_msgs/Generate_DS_Configs.h>
#include <rrt_planner_msgs/Compute_Goal_Config.h>
#include <rrt_planner_msgs/compute_motion_plan.h>
#include <rrt_planner_msgs/SC_Generator_Test.h>
#include <rrt_planner_msgs/RRT_Planner_Test.h>
#include <rrt_planner_msgs/Final_Pose_Planning.h>

#include <renbo_msgs/compute_motion_plan.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "whole_body_motion_plan");

  ros::NodeHandle nh_;

  int ACTIVE_SERVICE = 0, SCENERIO = 0;
  if (argc == 2)
  {
    std::stringstream s(argv[1]);
    s >> ACTIVE_SERVICE;
  }
  else if (argc == 3)
  {
    //std::stringstream s(argv[1]), ss(argv[2]);
    ACTIVE_SERVICE = std::atoi(argv[1]);
    SCENERIO = std::atoi(argv[2]);
  }

  if (ACTIVE_SERVICE == 0)
  {
    ros::ServiceClient sc_generator = nh_.serviceClient<rrt_planner_msgs::Generate_DS_Configs>("generate_ds_database");
    rrt_planner_msgs::Generate_DS_Configs ds_config_srv;

    ds_config_srv.request.max_samples = 5000;
    ds_config_srv.request.max_ik_iterations = 5;

    if(sc_generator.call(ds_config_srv))
    {
      ROS_INFO_STREAM(ds_config_srv.response.num_configs_generated << " statically stabe config has generated");
    }
    else
    {
      ROS_ERROR("Fail to generate statically stable data");
      exit(1);
    }
  }
  else if (ACTIVE_SERVICE == 1)
  {
    ros::ServiceClient mg_rrt_planner = nh_.serviceClient<renbo_msgs::compute_motion_plan>("multi_goal_rrt_planner");
    renbo_msgs::compute_motion_plan mg_rrt_planner_srv;

    mg_rrt_planner_srv.request.scenario = SCENERIO;
    if(mg_rrt_planner.call(mg_rrt_planner_srv))
    {
    }
    else
    {
      ROS_ERROR("Run test fail");
      exit(1);
    }
  }
//  else if (ACTIVE_SERVICE == 1)
//  {
//    ros::ServiceClient sc_test = nh_.serviceClient<rrt_planner_msgs::SC_Generator_Test>("sc_generator_test");
//    rrt_planner_msgs::SC_Generator_Test test_srv;

//    test_srv.request.state = 1;

//    if(sc_test.call(test_srv))
//    {
//      ROS_INFO_STREAM("return state: " << test_srv.response.result);
//    }
//    else
//    {
//      ROS_ERROR("Run test fail");
//      exit(1);
//    }
//  }
  else if (ACTIVE_SERVICE == 2)
  {
    ros::ServiceClient rrt_planner_test_ = nh_.serviceClient<rrt_planner_msgs::compute_motion_plan>("rrt_planner_test");

    rrt_planner_msgs::compute_motion_plan rrt_planner_test_srv;
    rrt_planner_test_srv.request.scenerio = SCENERIO;

    if (rrt_planner_test_.call(rrt_planner_test_srv))
    {
      ROS_INFO_STREAM("return state: " << rrt_planner_test_srv.response.success);
    }
    else
    {
      ROS_ERROR("rrt planner test fail");
      exit(1);
    }
  }
  else if (ACTIVE_SERVICE == 3)
  {
    ros::ServiceClient final_pose_planning_ = nh_.serviceClient<rrt_planner_msgs::Final_Pose_Planning>("final_pose_planning");

    rrt_planner_msgs::Final_Pose_Planning final_pose_planning_srv;
    final_pose_planning_srv.request.scenerio = SCENERIO;

    if (final_pose_planning_.call(final_pose_planning_srv))
    {
      ROS_INFO_STREAM("return state: " << final_pose_planning_srv.response.success);
    }
    else
    {
      ROS_ERROR("final pose planner fail");
      exit(1);
    }
  }
  else if (ACTIVE_SERVICE == 4)
  {
    ros::ServiceClient pick_place_motion_plan_ = nh_.serviceClient<renbo_msgs::compute_motion_plan>("pick_place_motion_plan");
    renbo_msgs::compute_motion_plan pick_place_motion_plan_srv;

    pick_place_motion_plan_srv.request.scenario = SCENERIO;

    if (pick_place_motion_plan_.call(pick_place_motion_plan_srv))
    {
      ROS_INFO_STREAM("return state: " << pick_place_motion_plan_srv.response.success);
    }
    else
    {
      ROS_ERROR("pick and place motion plan fail");
      exit(1);
    }

  }

  ros::shutdown();

  return 0;

}

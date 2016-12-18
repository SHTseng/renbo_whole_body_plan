#ifndef RENBO_PLANNER_H_
#define RENBO_PLANNER_H_

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <rrt_planner_msgs/Generate_DS_Configs.h>
#include <rrt_planner_msgs/Compute_Goal_Config.h>
#include <rrt_planner_msgs/Compute_Motion_Plan.h>
#include <rrt_planner_msgs/Compute_Linear_Manipulation_Plan.h>
#include <rrt_planner_msgs/Compute_Circular_Manipulation_Plan.h>
#include <rrt_planner_msgs/SC_Generator_Test.h>
#include <rrt_planner_msgs/RRT_Planner_Test.h>
#include <rrt_planner_msgs/Final_Pose_Planning.h>

#include <renbo_whole_body_plan/stable_config_generator.h>
#include <renbo_whole_body_plan/rrt_connect_planner.h>
#include <renbo_whole_body_plan/final_pose_planner.h>

#include <geometric_shapes/shapes.h>
#include <shape_msgs/Mesh.h>
#include <eigen_conversions/eigen_msg.h>

namespace planner_control
{

class RenboPlanner
{
public:

  RenboPlanner();

  ~RenboPlanner();

  bool generate_ds_database(rrt_planner_msgs::Generate_DS_Configs::Request &req, rrt_planner_msgs::Generate_DS_Configs::Response &res);

  bool compute_goal_config(rrt_planner_msgs::Compute_Goal_Config::Request &req, rrt_planner_msgs::Compute_Goal_Config::Response &res);

  bool compute_motion_plan(rrt_planner_msgs::Compute_Motion_Plan::Request &req, rrt_planner_msgs::Compute_Motion_Plan::Response &res);

  bool sc_generator_test(rrt_planner_msgs::SC_Generator_Test::Request &req, rrt_planner_msgs::SC_Generator_Test::Response &res);

  bool rrt_planner_test(rrt_planner_msgs::RRT_Planner_Test::Request &req, rrt_planner_msgs::RRT_Planner_Test::Response &res);

  bool final_pose_planning(rrt_planner_msgs::Final_Pose_Planning::Request &req, rrt_planner_msgs::Final_Pose_Planning::Response &res);

  bool demo(rrt_planner_msgs::Final_Pose_Planning::Request &req, rrt_planner_msgs::Final_Pose_Planning::Response &res);

private:

  moveit_msgs::CollisionObject loadMeshFromSource(const std::__cxx11::string file_name, const geometry_msgs::Pose &pose);

  void loadCollisionEnvironment(int type);

  bool checkCollision(const planning_scene::PlanningScenePtr ps_);

  void triggerPlanningSceneUpade();

  void loadYamlParameter();

  void PAUSE();

  std_msgs::ColorRGBA getColor(float r, float g, float b, float a);

  ros::NodeHandle nh_;

  ros::Publisher robot_state_publisher_;
  ros::Publisher goal_state_publisher_;

  ros::Publisher scene_publisher_;

  ros::Publisher trajectory_publisher_;

  const robot_state::JointModelGroup* wb_jmg_;

  std::shared_ptr<renbo_constraint_sampler::StableConfigGenerator> scg_;

  std::shared_ptr<renbo_planner::RRTConnectPlanner> rrt_;

  std::shared_ptr<renbo_planner::FinalPosePlanner> fpp_;

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  planning_scene::PlanningScenePtr ps_;

  moveit::planning_interface::PlanningSceneInterface pci_;

  std::vector<moveit_msgs::CollisionObject> cos_msg_;

  moveit_msgs::PlanningScene planning_scene_msg_;

  std::string base_frame_;

  std::string eef_name_;

  std::string package_name_;

  std::string package_path_;

  std::vector<std::string> wb_joint_names_;

  moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

  //GENERAL PARAMS
  std::string ROBOT_DESCRIPTION;      // name of the robot description (a param name, so it can be changed externally)
  std::string PLANNING_GROUP;  		// group to plan for
  double SCALE_SP ;					//Scaling factor for the support polygon (scaling to avoid projected COM to reach SP border)

  std::string scene_path_;

  std::string solution_file_path_;

  // STABLE CONFIG GENERATOR PARAMS
  int MAX_SAMPLES;
  int MAX_IK_ITERATIONS;

  // RRT CONNECT PLANNER PARAMS
  double ADVANCE_STEPS;
  int MAX_EXPAND_ITERATIONS;
  bool VISUALIZE_PLANNING_PATH;

  std::string ds_database_path_;
  int DS_CONFIG_COUNT;

  int scenario_;

  int test_flag_;
  bool write_pose_;
  bool verbose_;

};

}



#endif

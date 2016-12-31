#ifndef FINAL_POSE_PLANNER_H_
#define FINAL_POSE_PLANNER_H_

#include <ros/ros.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>

#include <robot_state_publisher/robot_state_publisher.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <trac_ik/trac_ik.hpp>

#include <random_numbers/random_numbers.h>

#include <renbo_whole_body_plan/double_support_constraint.h>
#include <renbo_whole_body_plan/nlp_ik_solver.h>

#include <fstream>

namespace renbo_planner
{

class FinalPosePlanner
{

public:

  FinalPosePlanner();

  ~FinalPosePlanner();

  bool solveFinalPose(moveit::core::RobotState& robot_state_,
                      const Eigen::Affine3d &r_eef_config_,
                      std::vector<double> &solution);

  Eigen::Affine3d setRightGripperConfig(Eigen::Affine3d original_config);

  void updateScene(const planning_scene::PlanningScenePtr &scene);

  bool TEST();

private:

  void initilize();

  bool isConfigValid();

  bool checkCollision();

  bool checkStability();

  Eigen::Affine3d setWaistConfig(const Eigen::Affine3d &original_config);

  KDL::Frame EigenToKDL(const Eigen::Affine3d& affine_);

  ros::NodeHandle nh_;

  ros::Publisher robot_state_publisher_;
  ros::Publisher scene_publisher_;

  planning_scene::PlanningScenePtr ps_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  robot_model::RobotModelPtr robot_model_;

  const robot_state::JointModelGroup* wb_jmg_;

  const robot_state::JointModelGroup* wb_r_arm_jmg_;

  const robot_state::JointModelGroup* right_arm_torso_jmg_;

  const robot_state::JointModelGroup* l_leg_jmg_;

  const robot_state::JointModelGroup* r_leg_jmg_;

  std::map<std::string, double> wb_jnt_pos_map_;

  std::vector<std::string> wb_joint_names_;

  std::vector<std::string> wb_link_names_;

  std::vector<double> wb_joint_values_;

  int wb_num_joint_;

  std::string group_name_;

  KDL::Tree kdl_tree_;

  KDL::Chain right_leg_chain_;

  boost::shared_ptr<const urdf::ModelInterface> renbo_urdf_;

  renbo_constraint_sampler::DoubleSupportConstraint ds_constraint_;

//  NLPIKSolver nlp_ik_solver_;
  std::shared_ptr<NLPIKSolver> nlp_ik_solver_;

  Eigen::Affine3d r_eef_config_;

};


}



#endif

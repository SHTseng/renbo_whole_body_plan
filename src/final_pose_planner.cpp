#include <renbo_whole_body_plan/final_pose_planner.h>

namespace renbo_planner
{

FinalPosePlanner::FinalPosePlanner():
  nh_("~")
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  robot_model_ = robot_model_loader_->getModel();

  ps_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_));

  wb_jmg_ = ps_->getRobotModel()->getJointModelGroup("whole_body_fixed");

  wb_r_arm_jmg_ = ps_->getRobotModel()->getJointModelGroup("whole_body_r_arm");

  right_arm_torso_jmg_ = ps_->getRobotModel()->getJointModelGroup("right_arm_torso");

  l_leg_jmg_ = ps_->getRobotModel()->getJointModelGroup("left_leg");

  r_leg_jmg_ = ps_->getRobotModel()->getJointModelGroup("right_leg");

  wb_joint_names_ = wb_jmg_->getActiveJointModelNames();

  wb_link_names_ = wb_jmg_->getLinkModelNames();

  wb_num_joint_ = wb_joint_names_.size();

  nlp_ik_solver_.reset(new NLPIKSolver());

  initilize();

}

FinalPosePlanner::~FinalPosePlanner()
{

}


void FinalPosePlanner::initilize()
{
  renbo_urdf_ = ps_->getRobotModel()->getURDF();

  if (!kdl_parser::treeFromUrdfModel(*renbo_urdf_, kdl_tree_))
  {
    ROS_ERROR_STREAM("Could not initialize tree object");
    exit(0);
  }

  bool setRightLegChain = kdl_tree_.getChain("r_sole", "waist", right_leg_chain_);
  if (!setRightLegChain)
  {
    ROS_ERROR("Can't get right leg chain");
  }

  robot_state::RobotState robot_state_(ps_->getRobotModel());
  robot_state_.setToDefaultValues();
  robot_state_.update();

  r_eef_config_ = robot_state_.getGlobalLinkTransform("r_gripper");


}

bool FinalPosePlanner::solveFinalPose(moveit::core::RobotState &robot_state_,
                                      const Eigen::Affine3d& r_eef_config_,
                                      std::vector<double>& solution)
{
  Eigen::Affine3d desired_waist_config_eigen = setWaistConfig(robot_state_.getGlobalLinkTransform("waist"));

  KDL::Frame desired_waist_config = EigenToKDL(desired_waist_config_eigen);

  KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(right_leg_chain_);
  KDL::ChainIkSolverVel_pinv ik_pinv = KDL::ChainIkSolverVel_pinv(right_leg_chain_);
  KDL::ChainIkSolverPos_NR ik = KDL::ChainIkSolverPos_NR(right_leg_chain_, fk_solver, ik_pinv, 100, 1e-3);

  unsigned int num_joints = right_leg_chain_.getNrOfJoints();

  KDL::JntArray q_out = KDL::JntArray(num_joints);
  KDL::JntArray q_init = KDL::JntArray(num_joints);

  for (int i = 0; i < num_joints; i++)
  {
    q_init(i, 0) = 0.0;
  }

  bool ik_check;
  ik_check = ik.CartToJnt(q_init, desired_waist_config, q_out);

  robot_state_.copyJointGroupPositions("whole_body_fixed", wb_joint_values_);
  wb_jnt_pos_map_.clear();
  for (int i = 0; i < wb_joint_names_.size(); i++)
  {
    wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_joint_names_[i], wb_joint_values_[i]));
  }

  std::vector<std::string> r_leg_joint_names = r_leg_jmg_->getJointModelNames();

  for (int i = 0; i < r_leg_joint_names.size(); i++)
  {
    wb_jnt_pos_map_[r_leg_joint_names[i]] = q_out(i)*(-1);
  }

  wb_jnt_pos_map_["head_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["head_pitch_joint"] = 0.0;
  wb_jnt_pos_map_["l_shoulder_pitch_joint"] = -0.785;
  wb_jnt_pos_map_["l_shoulder_roll_joint"] = 0.0;
  wb_jnt_pos_map_["l_shoulder_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["l_elbow_joint"] = 1.0472;
  wb_jnt_pos_map_["l_wrist_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["l_wrist_pitch_joint"] = -0.523;

  robot_state_.setVariablePositions(wb_jnt_pos_map_);
  robot_state_.update();

  Eigen::Affine3d left_foot_config = robot_state_.getGlobalLinkTransform("l_sole");
  Eigen::Affine3d r_foot_config = robot_state_.getGlobalLinkTransform("r_sole");

  Eigen::Matrix3d rot_x, rot_z;
  rot_x = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX());
  rot_z = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());

  left_foot_config.linear() = rot_z*rot_x*r_foot_config.linear();
  left_foot_config.translation() << 0.0, 0.2477, 0.15005;

  ik_check = robot_state_.setFromIK(l_leg_jmg_, left_foot_config, 1, 0);

  if (ik_check)
  {
    robot_state_.update();
  }
  else
  {
    ROS_WARN_NAMED("final pose planner", "solve left leg ik fail");
    ik_check = false;
    return ik_check;
  }

  ik_check = robot_state_.setFromIK(right_arm_torso_jmg_, r_eef_config_, 1, 0);
  if (ik_check)
  {
    robot_state_.update();
  }
  else
  {
    ROS_WARN_NAMED("final pose planner", "solve right eef ik fail");
    ik_check = false;
  }

  robot_state_.copyJointGroupPositions(wb_jmg_, solution);

  return ik_check;

}

Eigen::Affine3d FinalPosePlanner::setRightGripperConfig(Eigen::Affine3d original_config)
{
  Eigen::Affine3d read_config;
  read_config = original_config;

  std::ifstream read_file("/home/shtseng/catkin_ws/src/renbo_whole_body_plan/database/right_eef_config.dat", std::ios::in);

  if (!read_file)
  {
    ROS_ERROR("Can not open right eef configuration file");
    return read_config;
  }

  std::vector<double> config_;
  double temp = 0.0;
  while(read_file >> temp)
  {
    config_.push_back(temp);
  }

  read_file.close();

  read_config.translation().x() = config_[0];
  read_config.translation().y() = config_[1];
  read_config.translation().z() = config_[2];

  Eigen::Matrix3d rotation_;
  rotation_ = Eigen::AngleAxisd(config_[3], Eigen::Vector3d::UnitX())*
              Eigen::AngleAxisd(config_[4], Eigen::Vector3d::UnitY())*
              Eigen::AngleAxisd(config_[5], Eigen::Vector3d::UnitZ());

  read_config.rotate(rotation_);

  return read_config;

}

Eigen::Affine3d FinalPosePlanner::setWaistConfig(const Eigen::Affine3d& original_config)
{
  std::string config_path = ros::package::getPath("renbo_whole_body_plan");
  config_path.append("/database/waist_config.dat");

  Eigen::Affine3d read_config = original_config;

  std::ifstream read_file(config_path, std::ios::in);
  if (!read_file)
  {
    ROS_ERROR("Can not open waist configuration file");
    return read_config;
  }

  std::vector<double> config_;
  double temp = 0.0;
  while(read_file >> temp)
  {
    config_.push_back(temp);
  }

  read_file.close();

  read_config.translation().x() -= config_[0];
  read_config.translation().y() -= config_[1];
  read_config.translation().z() -= config_[2];

  return read_config;
}

KDL::Frame FinalPosePlanner::EigenToKDL(const Eigen::Affine3d& affine_)
{
  Eigen::Matrix3d eigen_rotation_ = affine_.rotation();
  KDL::Rotation kdl_rotation_;

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      kdl_rotation_(i, j) = eigen_rotation_(i, j);
    }
  }

  KDL::Vector kdl_position_(affine_.translation()[0], affine_.translation()[1], affine_.translation()[2]);

  KDL::Frame frame_(kdl_rotation_, kdl_position_);

  return frame_;
}

void FinalPosePlanner::updateScene(const planning_scene::PlanningScenePtr &scene)
{
  ps_ = scene;
}

bool FinalPosePlanner::TEST()
{
  robot_state::RobotState robot_state_(ps_->getRobotModel());
  robot_state_.setToDefaultValues();
  robot_state_.update();

  r_eef_config_ = robot_state_.getGlobalLinkTransform("r_gripper");
//
//  Eigen::Affine3d read_r_eef_config = setRightGripperConfig(r_eef_config_);

//  Eigen::Affine3d r_eef_config = r_eef_config_;
//  std::vector<double> solution;
//  solution.resize(wb_num_joint_);

//  bool solve_final_pose = solveFinalPose(robot_state_, read_r_eef_config, solution);
//  TRAC_IK::TRAC_IK trac_ik_solver_("r_sole", "r_gripper", "/robot_description", 0.005, 1e-5, TRAC_IK::Manip1);
//
//  KDL::JntArray q_init = KDL::JntArray(wb_r_arm_jmg_->getVariableCount());
//
//  for (int i = 0; i < wb_r_arm_jmg_->getVariableCount(); i++)
//  {
//    q_init(i) = 0.0;
//  }
//
//  KDL::Frame kdl_eef_pose = EigenToKDL(read_r_eef_config);
//
//  KDL::JntArray q_out = KDL::JntArray(wb_r_arm_jmg_->getVariableCount());
//  int rc = trac_ik_solver_.CartToJnt(q_init, kdl_eef_pose, q_out);
//
//  if (rc)
//  {
//    std::vector<std::string> wb_r_arm_names = wb_r_arm_jmg_->getJointModelNames();
//
//    wb_jnt_pos_map_.clear();
//    for (int i = 0; i < wb_r_arm_names.size(); i++)
//    {
//      wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_r_arm_names[i], q_out(i)));
//    }
//
//    robot_state_.setVariablePositions(wb_jnt_pos_map_);
//    robot_state_.update();

//  }

  bool rc = true;

  std::vector<double> sln_pose;
  nlp_ik_solver_->solve(r_eef_config_, sln_pose);

  return rc;

}

}

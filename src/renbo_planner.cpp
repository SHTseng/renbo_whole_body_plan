#include <renbo_whole_body_plan/renbo_planner.h>

namespace planner_control
{

RenboPlanner::RenboPlanner():
  nh_("~"),
  package_name_("renbo_whole_body_plan"),
  base_frame_("r_sole"),
  eef_name_("r_gripper"),
  waist_name_("waist")
{
  loadYamlParameter();

  package_path_ = ros::package::getPath(package_name_);

  boost::shared_ptr<tf::TransformListener> tf;
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, tf, "renbo_psm_planning_scene"));

  ros::spinOnce();
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  if (psm_->getPlanningScene())
  {
    psm_->startStateMonitor();
    psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                       "renbo_planning_scene_monitor");

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm_);
    planning_scene->setName("renbo_psm_planning_scene");

    psm_->startSceneMonitor("renbo_planning_scene_monitor");
  }

  ps_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(psm_->getRobotModel()));

  wb_jmg_ = ps_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);

  wb_joint_names_ = wb_jmg_->getJointModelNames();

  scg_ = std::shared_ptr<renbo_constraint_sampler::StableConfigGenerator>
      (new renbo_constraint_sampler::StableConfigGenerator(PLANNING_GROUP, SCALE_SP));

  fpp_.reset(new renbo_planner::FinalPosePlanner());

  robot_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("renbo_robot_state", 1);

  goal_state_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("goal_state", 1);

  init_pick_trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("init_pick_trajectory", 1);

  pick_place_trajectory_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("pick_place_trajectory", 1);

  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame_, "/rviz_visual_markers"));

  ros::Duration(1.0).sleep();

  rviz_visual_tools_->deleteAllMarkers();

  rviz_visual_tools_->enableBatchPublishing(true);

  mg_rrt_.reset(new renbo_planner::MultiGoalRRTPlanner(PLANNING_GROUP, ds_database_path_, write_file_));

  mg_rrt_->setVerbose(verbose_);
  mg_rrt_->setVisualizationSwtich(VISUALIZE_PLANNING_PATH);

  rrt_.reset(new renbo_planner::RRTConnectPlanner(PLANNING_GROUP, ds_database_path_, write_file_));

  rrt_->setVerbose(verbose_);

  rrt_->setVisualizationSwtich(VISUALIZE_PLANNING_PATH);

  ROS_INFO_STREAM("Ready to start");
}

RenboPlanner::~RenboPlanner()
{
  psm_->stopPublishingPlanningScene();
}

bool RenboPlanner::generate_ds_database(
    rrt_planner_msgs::Generate_DS_Configs::Request &req,
    rrt_planner_msgs::Generate_DS_Configs::Response &res)
{
  renbo_constraint_sampler::StableConfigGenerator::FootSupport support_mode = renbo_constraint_sampler::StableConfigGenerator::DOUBLE_SUPPORT;

  scg_->setVerbose(verbose_);
  scg_->setSupportMode(support_mode);

  if (!scg_->sampleDSConfig(DS_CONFIG_COUNT, ds_database_path_, write_file_))
  {
    ROS_ERROR("Could not generate stable config");
    exit(1);
  }

  res.num_configs_generated = scg_->getNumConfig();

  return true;
}

bool RenboPlanner::compute_robot_com(
    rrt_planner_msgs::Generate_DS_Configs::Request &req,
    rrt_planner_msgs::Generate_DS_Configs::Response &res)
{

  robot_state::RobotState state(ps_->getRobotModel());
  state.setToDefaultValues();
  state.update();

  if (!scg_->computeRobotCoM(state))
  {
    ROS_ERROR("compute robot CoM fail");
    return false;
  }

  return true;
}

bool RenboPlanner::generate_valid_config(
    renbo_msgs::generate_ss_config::Request &req,
    renbo_msgs::generate_ss_config::Response &res)
{
  if(!scg_->generateConfig())
  {
    res.result = 0;
    return true;
  }

  res.result = 1;
  return true;
}


bool RenboPlanner::rrt_planner_test(rrt_planner_msgs::compute_motion_plan::Request &req, rrt_planner_msgs::compute_motion_plan::Response &res)
{
  scenario_ = req.scenerio;
  rviz_visual_tools_->deleteAllMarkers();

  loadCollisionEnvironment(scenario_);

  robot_state::RobotState robot_state_ = psm_->getPlanningScene()->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();
  robot_state_.update();

//  updatePSMRobotState(robot_state_);

  eef_original_config_ = robot_state_.getGlobalLinkTransform(eef_name_);
  waist_original_config_ = robot_state_.getGlobalLinkTransform(waist_name_);

  Eigen::Affine3d eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose;
  if (!updatePickPlacePose(scenario_, eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose))
  {
    ROS_INFO_STREAM("Can't update pick and place poses");
    return false;
  }

  fpp_->updateScene(psm_->getPlanningScene());

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "RRT planner: Start pick planning");

  std::vector<double> pick_config(wb_jmg_->getVariableCount());
  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_pick_pose, waist_pick_pose, pick_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve pick pose fail");
    return false;
  }

  ROS_INFO_STREAM("Solved pick pose");

  std::vector<double> place_config(wb_jmg_->getVariableCount());
  final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_place_pose, waist_place_pose, place_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve place pose fail");
    return false;
  }

  ROS_INFO_STREAM("Solved place pose");

//  robot_state_.setVariablePositions(wb_joint_names_, place_config);
//  robot_state_.update();
//  updatePSMRobotState(robot_state_);


  Eigen::Affine3d grasp_object_pose;
  grasp_object_pose = robot_state_.getGlobalLinkTransform(eef_name_);
  grasp_object_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d transformed_translation = grasp_object_pose.rotation() * Eigen::Vector3d(0.0, -0.16, 0.0);
  grasp_object_pose.translation() += transformed_translation;

  shape_msgs::SolidPrimitive target_object;
  target_object.type = target_object.CYLINDER;
  target_object.dimensions.resize(2);
  target_object.dimensions[0] = 0.15;
  target_object.dimensions[1] = 0.025;

  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(grasp_object_pose, pose);

  moveit_msgs::CollisionObject collision_target_object;
  collision_target_object.id = "cup";
  collision_target_object.header.frame_id = base_frame_;
  collision_target_object.primitives.push_back(target_object);
  collision_target_object.primitive_poses.push_back(pose);
  collision_target_object.operation = moveit_msgs::CollisionObject::ADD;

  addPSMCollisionObject(collision_target_object, getColor(169.0, 169.0, 169.0, 1.0));
  ROS_INFO_STREAM("Add target collision object to scene");

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.object = collision_target_object;
  attached_object.link_name = eef_name_;


  rrt_->is_grasped = false;

  rrt_->updateEnvironment(psm_->getPlanningScene());

  std::vector<double> initial_configuration(wb_jmg_->getVariableCount());
  rrt_->setStartGoalConfigs(initial_configuration, place_config);

  rrt_->setAttachCollsionObject(attached_object);
  rrt_->TEST(test_flag_);

//  moveit_msgs::DisplayTrajectory display_trajectory_ = rrt_->solveQuery(20000, 0.1);
//  trajectory_publisher_.publish(display_trajectory_);

  rrt_->is_grasped = false;

  res.success = true;

  return true;

}

bool RenboPlanner::final_pose_planning(rrt_planner_msgs::Final_Pose_Planning::Request &req, rrt_planner_msgs::Final_Pose_Planning::Response &res)
{
  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN << "Goal pose planner: Start goal pose planning");

  scenario_ = req.scenerio;
  rviz_visual_tools_->deleteAllMarkers();

  robot_state::RobotState robot_state_ = psm_->getPlanningScene()->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();
  robot_state_.update();

  updatePSMRobotState(robot_state_);
  loadCollisionEnvironment(scenario_);

  eef_original_config_ = robot_state_.getGlobalLinkTransform(eef_name_);
  waist_original_config_ = robot_state_.getGlobalLinkTransform(waist_name_);

  Eigen::Affine3d eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose;
  if (!updatePickPlacePose(scenario_, eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose))
  {
    ROS_INFO_STREAM("Can't update picking and placing poses");
    return false;
  }

  fpp_->updateScene(psm_->getPlanningScene());

  ROS_INFO("FPP: Start solving picking pose");
  std::vector<double> pick_config(wb_jmg_->getVariableCount());
  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_pick_pose, waist_pick_pose, pick_config);
  if (!final_pose_flag)
  {
    ROS_ERROR("Solve picking pose fail");
//    return false;
  }
  ROS_INFO_STREAM("Solved picking pose");

  robot_state_.setVariablePositions(wb_joint_names_, pick_config);
  robot_state_.update();

  moveit_msgs::DisplayRobotState picking_state_msgs;
  robot_state::robotStateToRobotStateMsg(robot_state_, picking_state_msgs.state);
  goal_state_pub_.publish(picking_state_msgs);

  ros::Duration(1.0).sleep();

  ROS_INFO("FPP: Start solving placing pose");
  std::vector<double> place_config(wb_jmg_->getVariableCount());
  final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_place_pose, waist_place_pose, place_config);
  if (!final_pose_flag)
  {
    ROS_ERROR("Solve placing pose fail");
//    return false;
  }
  ROS_INFO_STREAM("Solved placing pose");

  robot_state_.setVariablePositions(wb_joint_names_, place_config);
  robot_state_.update();

  moveit_msgs::DisplayRobotState placing_state_msgs;
  robot_state::robotStateToRobotStateMsg(robot_state_, placing_state_msgs.state);
  robot_state_pub_.publish(placing_state_msgs);

  ros::Duration(1.0).sleep();

  res.success = true;

  return true;
}

bool RenboPlanner::generate_whole_body_posture(renbo_msgs::generate_whole_body_posture::Request &req,
                                               renbo_msgs::generate_whole_body_posture::Response &res)
{
  scenario_ = req.scenario;

  robot_state::RobotState robot_state_ = psm_->getPlanningScene()->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();
  robot_state_.update();

  eef_original_config_ = robot_state_.getGlobalLinkTransform(eef_name_);
  waist_original_config_ = robot_state_.getGlobalLinkTransform(waist_name_);

  Eigen::Affine3d eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose;
  if (!updatePickPlacePose(scenario_, eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose))
  {
    ROS_INFO_STREAM("Can't update picking and placing poses");
    return false;
  }

  std::vector<double> whole_body_config(wb_jmg_->getVariableCount());
  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_pick_pose, waist_pick_pose, whole_body_config);
  if (!final_pose_flag)
  {
    ROS_ERROR("solve whole-body posture fail");
    res.success = 0;
    return false;
  }

  res.success = 1;
  robot_state_.copyJointGroupPositions(PLANNING_GROUP, res.solved_config);
  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "solved whole-body posture");

  return true;
}

bool RenboPlanner::pick_place_motion_plan(rrt_planner_msgs::compute_motion_plan::Request &req, rrt_planner_msgs::compute_motion_plan::Response &res)
{
  scenario_ = req.scenerio;
  rviz_visual_tools_->deleteAllMarkers();

  robot_state::RobotState robot_state_ = psm_->getPlanningScene()->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();
  robot_state_.update();

  updatePSMRobotState(robot_state_);
  loadCollisionEnvironment(scenario_);

  eef_original_config_ = robot_state_.getGlobalLinkTransform(eef_name_);
  waist_original_config_ = robot_state_.getGlobalLinkTransform(waist_name_);

  Eigen::Affine3d eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose;
  if (!updatePickPlacePose(scenario_, eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose))
  {
    ROS_INFO_STREAM("Can't update pick and place poses");
    return false;
  }

  fpp_->updateScene(psm_->getPlanningScene());

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "RRT planner: Start pick planning");

  std::vector<double> pick_config(wb_jmg_->getVariableCount());
  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_pick_pose, waist_pick_pose, pick_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve pick pose fail");
    return false;
  }

  ROS_INFO_STREAM("Solved pick pose");

  //Setup place pose
  std::vector<double> place_config(wb_jmg_->getVariableCount());
  final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_place_pose, waist_place_pose, place_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve place pose fail");
    return false;
  }

  robot_state_.setVariablePositions(wb_jmg_->getJointModelNames(), pick_config);
  robot_state_.update();
  updatePSMRobotState(robot_state_);

  // Publish goal posture to rviz
  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  goal_state_pub_.publish(robot_state_msg_);

  Eigen::Affine3d grasp_object_pose;
  grasp_object_pose = robot_state_.getGlobalLinkTransform(eef_name_);
  grasp_object_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d transformed_translation = grasp_object_pose.rotation() * Eigen::Vector3d(0.0, -0.16, 0.0);


//  grasp_object_pose.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
//  Eigen::Vector3d transformed_translation = grasp_object_pose.rotation() * Eigen::Vector3d(0.0, 0.0, -0.22);

  grasp_object_pose.translation() += transformed_translation;

  /*
   *  Adding collision grasp object to the scene
   */
  shape_msgs::SolidPrimitive target_object;
  target_object.type = target_object.CYLINDER;
  target_object.dimensions.resize(2);
  target_object.dimensions[0] = 0.15;
  target_object.dimensions[1] = 0.025;

  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(grasp_object_pose, pose);

  moveit_msgs::CollisionObject collision_target_object;
  collision_target_object.id = "cup";
  collision_target_object.header.frame_id = base_frame_;
  collision_target_object.primitives.push_back(target_object);
  collision_target_object.primitive_poses.push_back(pose);
  collision_target_object.operation = moveit_msgs::CollisionObject::ADD;

//  addPSMCollisionObject(collision_target_object, getColor(169.0, 169.0, 169.0, 1.0));

  //  Check robot state collision
  bool collision_free = checkCollision(psm_->getPlanningScene());
  if (!collision_free)
  {
    ROS_WARN_STREAM("pick pose is in collision");
    return false;
  }

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_BROWN << "RRT planner: pick pose is valid, start planning");

  //RRT-Connect path planning, initial state to pick place.
  std::vector<double> initial_configuration(wb_jmg_->getVariableCount());

  rrt_->updateEnvironment(psm_->getPlanningScene());
  rrt_->setStartGoalConfigs(initial_configuration, pick_config);
//  rrt_->setStartGoalConfigs(pick_config, place_config);


  moveit_msgs::DisplayTrajectory display_trajectory_ = rrt_->solveQuery(20000, 0.08);
  init_pick_trajectory_pub_.publish(display_trajectory_);

  ros::Duration(2.0).sleep();

//  moveit_msgs::AttachedCollisionObject attached_object;
//  attached_object.object = collision_target_object;
//  attached_object.link_name = eef_name_;

//  // RRT-Connect path planning, pick to place.
//  rrt_->is_grasped = true;
//  rrt_->updateEnvironment(psm_->getPlanningScene());
//  rrt_->setAttachCollsionObject(attached_object);
//  rrt_->setStartGoalConfigs(pick_config, place_config);

//  moveit_msgs::DisplayTrajectory display_trajectory_second_ = rrt_->solveQuery(20000, 0.1);
//  pick_place_trajectory_pub_.publish(display_trajectory_second_);

//  ros::Duration(2.0).sleep();

  rrt_->is_grasped = false;

  ps_->removeAllCollisionObjects();

  res.success = true;

  return true;
}

bool RenboPlanner::BiRRT_motion_plan(renbo_msgs::compute_motion_plan::Request &req,
                                     renbo_msgs::compute_motion_plan::Response &res)
{
  loadCollisionEnvironment(req.scenario);

  robot_state::RobotState initial_state = psm_->getPlanningScene()->getCurrentStateNonConst();
  initial_state.setVariablePositions(wb_joint_names_, req.initial_config);
  initial_state.update();

  robot_state::RobotState goal_state = psm_->getPlanningScene()->getCurrentStateNonConst();
  goal_state.setVariablePositions(wb_joint_names_, req.goal_config);
  goal_state.update();

  if (!checkCollision(initial_state) || !checkCollision(goal_state))
  {
    ROS_ERROR("start config or goal config is in collision");
    res.success = 0;
    return false;
  }

  rrt_->updateEnvironment(psm_->getPlanningScene());
  rrt_->setStartGoalConfigs(req.initial_config, req.goal_config);

  moveit_msgs::DisplayTrajectory display_trajectory = rrt_->solveQuery(20000, 0.1);
  init_pick_trajectory_pub_.publish(display_trajectory);

//  ros::Duration(0.5).sleep();

  res.whole_body_trajectory = display_trajectory;

  res.success = true;
  return res.success;
}

bool RenboPlanner::multi_goal_rrt_planner(rrt_planner_msgs::compute_motion_plan::Request &req,
                                          rrt_planner_msgs::compute_motion_plan::Response &res)
{
  scenario_ = req.scenerio;
  rviz_visual_tools_->deleteAllMarkers();

  robot_state::RobotState robot_state_ = psm_->getPlanningScene()->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();
  robot_state_.update();

  updatePSMRobotState(robot_state_);
  loadCollisionEnvironment(scenario_);

  eef_original_config_ = robot_state_.getGlobalLinkTransform(eef_name_);
  waist_original_config_ = robot_state_.getGlobalLinkTransform(waist_name_);

  Eigen::Affine3d eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose;
  if (!updatePickPlacePose(scenario_, eef_pick_pose, eef_place_pose, waist_pick_pose, waist_place_pose))
  {
    ROS_INFO_STREAM("Can't update pick and place poses");
    return false;
  }

  fpp_->updateScene(psm_->getPlanningScene());

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "Multi-Goal RRT planner: start planning");

  std::vector<double> pick_config(wb_jmg_->getVariableCount());
  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_pick_pose, waist_pick_pose, pick_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve pick pose fail");
    return false;
  }

  std::vector<double> place_config(wb_jmg_->getVariableCount());
  final_pose_flag = fpp_->solveFinalPose(robot_state_, eef_place_pose, waist_place_pose, place_config);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve place pose fail");
    return false;
  }

  ROS_INFO_STREAM("Solved pick and place pose");

  robot_state_.setVariablePositions(wb_jmg_->getJointModelNames(), pick_config);
  robot_state_.update();
  updatePSMRobotState(robot_state_);

  // Publish goal posture to rviz
  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  goal_state_pub_.publish(robot_state_msg_);

  Eigen::Affine3d grasp_object_pose;
  grasp_object_pose = robot_state_.getGlobalLinkTransform(eef_name_);
  Eigen::Vector3d transformed_translation = grasp_object_pose.rotation() * Eigen::Vector3d(-0.16, 0.0, 0.0);
  grasp_object_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  grasp_object_pose.translation() += transformed_translation;

  shape_msgs::SolidPrimitive target_object;
  target_object.type = target_object.CYLINDER;
  target_object.dimensions.resize(2);
  target_object.dimensions[0] = 0.15;
  target_object.dimensions[1] = 0.025;

  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(grasp_object_pose, pose);

  moveit_msgs::CollisionObject collision_target_object;
  collision_target_object.id = "cup";
  collision_target_object.header.frame_id = base_frame_;
  collision_target_object.primitives.push_back(target_object);
  collision_target_object.primitive_poses.push_back(pose);
  collision_target_object.operation = moveit_msgs::CollisionObject::ADD;

//  saddPSMCollisionObject(collision_target_object, getColor(169.0, 169.0, 169.0, 1.0));
  ROS_INFO_STREAM("Add target collision object to scene");

  bool collision_free = checkCollision(psm_->getPlanningScene());
  if (!collision_free)
  {
    ROS_ERROR_STREAM("pick pose is in collision");
    return false;
  }

  std::vector<double> initial_config(wb_jmg_->getVariableCount());
  std::vector< std::vector<double> > goal_configs;
  goal_configs.resize(5);
//  for (int i = 0; i < 5; i++)
//  {
//    goal_configs[i] = initial_config;
//  }

  goal_configs[0] = pick_config;
  goal_configs[1] = place_config;
  goal_configs[2] = place_config;
  goal_configs[3] = place_config;
  goal_configs[4] = place_config;

  mg_rrt_->updateEnvironment(psm_->getPlanningScene());
  mg_rrt_ ->setStartGoalConfigs(initial_config, goal_configs);

  std::vector<moveit_msgs::DisplayTrajectory> display_trajectory_ = mg_rrt_->solve(TIME_OUT, 0.1);

  init_pick_trajectory_pub_.publish(display_trajectory_[0]);
  ros::Duration(0.5).sleep();

  pick_place_trajectory_pub_.publish(display_trajectory_[1]);
  ros::Duration(0.5).sleep();

  return true;
}

void RenboPlanner::loadCollisionEnvironment(int type)
{
  geometry_msgs::Pose table_pose;
  geometry_msgs::Pose pose;

  std::string package_path = package_path_;
  std::ifstream environment_description;

  switch(type)
  {
  case 0:
  {
    package_path = package_path.append("/database/env_0.dat");
    environment_description.open(package_path.c_str());

    std::vector<double> temp_poses;
    double temp = 0.0;
    while (environment_description >> temp)
    {
      temp_poses.push_back(temp);
    }
    environment_description.close();

    table_pose.position.x = temp_poses[0];
    table_pose.position.y = temp_poses[1];
    table_pose.position.z = temp_poses[2];
    table_pose.orientation.w = temp_poses[3];
    table_pose.orientation.x = temp_poses[4];
    table_pose.orientation.y = temp_poses[5];
    table_pose.orientation.z = temp_poses[6];

    moveit_msgs::CollisionObject collision_mesh_table = loadMeshFromSource("ikea_table.stl", table_pose);
    addPSMCollisionObject(collision_mesh_table, getColor(222.0, 184.0, 135.0, 1.0));

    geometry_msgs::Pose mug_pose;
    mug_pose.position.x = temp_poses[7];
    mug_pose.position.y = temp_poses[8];
    mug_pose.position.z = temp_poses[9];
    mug_pose.orientation.w = temp_poses[10];
    mug_pose.orientation.x = temp_poses[11];
    mug_pose.orientation.y = temp_poses[12];
    mug_pose.orientation.z = temp_poses[13];

    moveit_msgs::CollisionObject collision_mesh_mug = loadMeshFromSource("mug.stl", mug_pose);
    addPSMCollisionObject(collision_mesh_mug, getColor(255.0, 255.0, 255.0, 1.0));

    geometry_msgs::Pose box_pose;
    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.345;
    box.dimensions[1] = 0.165;
    box.dimensions[2] = 0.165;

    box_pose.position.x = temp_poses[14];
    box_pose.position.y = temp_poses[15];
    box_pose.position.z = temp_poses[16];
    box_pose.orientation.w = temp_poses[17];
    box_pose.orientation.x = temp_poses[18];
    box_pose.orientation.y = temp_poses[19];
    box_pose.orientation.z = temp_poses[20];

    moveit_msgs::CollisionObject collision_box;
    collision_box.id = "box";
    collision_box.header.frame_id = "r_sole";
    collision_box.primitives.push_back(box);
    collision_box.primitive_poses.push_back(box_pose);
    collision_box.operation = collision_box.ADD;

    addPSMCollisionObject(collision_box, getColor(255.0, 255.0, 255.0, 1.0));

    break;
  }
  case 1:
  {
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.125;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_mesh_table = loadMeshFromSource("ikea_table.stl", table_pose);

    addPSMCollisionObject(collision_mesh_table, getColor(222.0, 184.0, 135.0, 1.0));

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.3;
    box.dimensions[1] = 0.04;
    box.dimensions[2] = 0.15;

    pose.position.x = 0.65;
    pose.position.y = 0.15;
    pose.position.z = 0.85;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_box;
    collision_box.id = "box";
    collision_box.header.frame_id = "r_sole";
    collision_box.primitives.push_back(box);
    collision_box.primitive_poses.push_back(pose);
    collision_box.operation = collision_box.ADD;

    addPSMCollisionObject(collision_box, getColor(222.0, 184.0, 135.0, 1.0));

    break;
  }
  case 2:
  {
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.125;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_mesh_table = loadMeshFromSource("ikea_table.stl", table_pose);

    addPSMCollisionObject(collision_mesh_table, getColor(222.0, 184.0, 135.0, 1.0));

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.304;
    box.dimensions[1] = 0.338;
    box.dimensions[2] = 0.167;

    pose.position.x = 0.6;
    pose.position.y = 0.125+0.1;
    pose.position.z = 0.74+0.0835;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_box;
    collision_box.id = "box";
    collision_box.header.frame_id = "r_sole";
    collision_box.primitives.push_back(box);
    collision_box.primitive_poses.push_back(pose);
    collision_box.operation = collision_box.ADD;

    addPSMCollisionObject(collision_box, getColor(255.0, 255.0, 255.0, 1.0));

    break;
  }
  case 3:
  {
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 0.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 1.0;

    moveit_msgs::CollisionObject collision_mesh_closet = loadMeshFromSource("closet_v3.stl", table_pose);
    addPSMCollisionObject(collision_mesh_closet, getColor(222.0, 184.0, 135.0, 1.0));

    break;
  }
  case 4:
  {
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.125;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_mesh_table = loadMeshFromSource("ikea_table.stl", table_pose);

    addPSMCollisionObject(collision_mesh_table, getColor(222.0, 184.0, 135.0, 1.0));

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.3;
    box.dimensions[1] = 0.02;
    box.dimensions[2] = 0.25;

    pose.position.x = 0.55;
    pose.position.y = -0.28;
    pose.position.z = 0.85;
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_box;
    collision_box.id = "box";
    collision_box.header.frame_id = "r_sole";
    collision_box.primitives.push_back(box);
    collision_box.primitive_poses.push_back(pose);
    collision_box.operation = collision_box.ADD;

    addPSMCollisionObject(collision_box, getColor(255.0, 255.0, 255.0, 1.0));
    break;
  }
  case 9:
  {
    ROS_INFO("load empty space");
    break;
  }

  } // end switch

}

bool RenboPlanner::checkCollision(const robot_state::RobotState& state)
{
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;

  collision_req.group_name = PLANNING_GROUP;
  collision_res.clear();

  ps_->checkCollision(collision_req, collision_res, state);

  if (collision_res.collision != 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool RenboPlanner::checkCollision(const planning_scene::PlanningScenePtr ps_)
{
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;

  collision_req.group_name = PLANNING_GROUP;
  collision_res.clear();

  ps_->checkCollision(collision_req, collision_res);

  if (collision_res.collision != 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

moveit_msgs::CollisionObject RenboPlanner::loadMeshFromSource(const std::string file_name, const geometry_msgs::Pose& pose)
{
  std::string file_path(scene_path_);

  file_path.append(file_name);
  shapes::Mesh* source_mesh = shapes::createMeshFromResource(file_path);

  ROS_INFO_STREAM("Loaded mesh from source");

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(source_mesh, mesh_msg);
  delete source_mesh;

  shape_msgs::Mesh mesh;
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject mesh_co;

  mesh_co.id = file_name;
  mesh_co.header.frame_id = "r_sole"; // "odom_combined ??
  mesh_co.meshes.push_back(mesh);
  mesh_co.mesh_poses.push_back(pose);
  mesh_co.operation = mesh_co.ADD;

  return mesh_co;
}

void RenboPlanner::addPSMCollisionObject(const moveit_msgs::CollisionObject& msg, const std_msgs::ColorRGBA& color)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->getCurrentStateNonConst().update();
    scene->processCollisionObjectMsg(msg);
    scene->setObjectColor(msg.id, color);

    triggerPlanningSceneUpade();
  }
}

void RenboPlanner::updatePSMRobotState(const robot_state::RobotState& state)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->setCurrentState(state);
    scene->getCurrentStateNonConst().update();

    triggerPlanningSceneUpade();
  }
}

void RenboPlanner::triggerPlanningSceneUpade()
{
  psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  ros::spinOnce();
}

bool RenboPlanner::updatePickPlacePose(const int& scenerio, Eigen::Affine3d& pick_pose, Eigen::Affine3d& place_pose,
                                       Eigen::Affine3d& pick_waist_pose, Eigen::Affine3d& place_waist_pose)
{
  std::string file_name;
  std::string package_path = package_path_;

  switch (scenerio)
  {
  case 0:
    file_name = package_path.append("/database/scene_0.dat");
    break;

  case 1:
    file_name = package_path.append("/database/scene_1.dat");
    break;

  case 2:
    file_name = package_path.append("/database/scene_2.dat");
    break;

  case 3:
    file_name = package_path.append("/database/scene_3.dat");
    break;

  case 4:
    file_name = package_path.append("/database/scene_4.dat");
    break;

  case 9:
    file_name = package_path.append("/database/scene_9.dat");
    break;
  }

  std::ifstream read_file(file_name, std::ios::in);
  if (!read_file)
  {
    ROS_ERROR("Can not open database file");
    return false;
  }

  std::vector<double> config_;
  double temp = 0.0;
  while (read_file >> temp)
  {
    config_.push_back(temp);
  }

  read_file.close();

  pick_pose.translation().x() = config_[0];
  pick_pose.translation().y() = config_[1];
  pick_pose.translation().z() = config_[2];

  pick_pose.linear() = eef_original_config_.linear();

//  Eigen::Matrix3d rot_a = eef_original_config_.linear();
//  Eigen::Vector3d eular_a = rot_a.eulerAngles(0, 1, 2);
//  ROS_INFO_STREAM(eular_a(0) << " " << eular_a(1) << " " << eular_a(2));

  Eigen::Matrix3d rotation_;
  rotation_ = Eigen::AngleAxisd(config_[3], Eigen::Vector3d::UnitX())*
              Eigen::AngleAxisd(config_[4], Eigen::Vector3d::UnitY())*
              Eigen::AngleAxisd(config_[5], Eigen::Vector3d::UnitZ());

  pick_pose.rotate(rotation_);

  rotation_ = Eigen::Matrix3d::Identity(3, 3);

  place_pose.translation().x() = config_[12];
  place_pose.translation().y() = config_[13];
  place_pose.translation().z() = config_[14];

  place_pose.linear() = eef_original_config_.linear();

  rotation_ = Eigen::AngleAxisd(config_[15], Eigen::Vector3d::UnitX())*
              Eigen::AngleAxisd(config_[16], Eigen::Vector3d::UnitY())*
              Eigen::AngleAxisd(config_[17], Eigen::Vector3d::UnitZ());

  place_pose.rotate(rotation_);

  pick_waist_pose.translation() = waist_original_config_.translation();
  pick_waist_pose.translation().x() -= config_[6];
  pick_waist_pose.translation().y() -= config_[7];
  pick_waist_pose.translation().z() -= config_[8];

  pick_waist_pose.linear() = waist_original_config_.linear();

  rotation_ = Eigen::Matrix3d::Identity(3, 3);
  rotation_ = Eigen::AngleAxisd(config_[9], Eigen::Vector3d::UnitX())*
              Eigen::AngleAxisd(config_[10], Eigen::Vector3d::UnitY())*
              Eigen::AngleAxisd(config_[11], Eigen::Vector3d::UnitZ());

  pick_waist_pose.rotate(rotation_);

  place_waist_pose.translation() = waist_original_config_.translation();
  place_waist_pose.translation().x() -= config_[18];
  place_waist_pose.translation().y() -= config_[19];
  place_waist_pose.translation().z() -= config_[20];

  place_waist_pose.linear() = waist_original_config_.linear();

  rotation_ = Eigen::Matrix3d::Identity(3, 3);
  rotation_ = Eigen::AngleAxisd(config_[21], Eigen::Vector3d::UnitX())*
              Eigen::AngleAxisd(config_[22], Eigen::Vector3d::UnitY())*
              Eigen::AngleAxisd(config_[23], Eigen::Vector3d::UnitZ());

  place_waist_pose.rotate(rotation_);

  return true;
}

void RenboPlanner::loadYamlParameter()
{
  nh_.param("robot_description", ROBOT_DESCRIPTION, std::string("robot_description"));
  nh_.param("planning_group", PLANNING_GROUP, std::string("whole_body_fixed"));

  // stable configuration generator parameters
  nh_.param("scg_max_samples", MAX_SAMPLES, 3000);
  nh_.param("scg_max_ik_iterations", MAX_IK_ITERATIONS, 5);
  nh_.param("scale_sp", SCALE_SP, 0.8);
  nh_.param("ds_config_count", DS_CONFIG_COUNT, 1);

  // rrt planner parameters
  nh_.param("planner_step_factor", ADVANCE_STEPS, 0.1);
  nh_.param("planner_max_iterations", MAX_EXPAND_ITERATIONS, 8000);
  nh_.param("visualize_planning_path", VISUALIZE_PLANNING_PATH, false);
  nh_.param("rrt_time_out", TIME_OUT, 60.0);

  // file paths
  nh_.param("file_path_DS_database", ds_database_path_, std::string("package://renbo_whole_body_plan/database/ds_dataset.dat"));
  nh_.param("scene_path", scene_path_, std::string("package://renbo_whole_body_plan/scene/"));

  nh_.param("solution_file_path", solution_file_path_, std::string("package://renbo_whole_body_plan/trajectory/"));

  // general parameter
  nh_.param("scenario", scenario_, 0);
  nh_.param("test_flag", test_flag_, 0);
  nh_.param("write_file", write_file_, false);
  nh_.param("verbose", verbose_, false);
}

std_msgs::ColorRGBA RenboPlanner::getColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;

  color.r = r/255;
  color.g = g/255;
  color.b = b/255;
  color.a = a;

  return color;
}

void RenboPlanner::PAUSE()
{
  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_BROWN << "Press any key to continue");
  std::string dummy;
  std::getline(std::cin, dummy);
}



}

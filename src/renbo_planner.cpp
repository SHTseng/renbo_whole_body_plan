#include <renbo_whole_body_plan/renbo_planner.h>

namespace planner_control
{

RenboPlanner::RenboPlanner():
  nh_("~"),
  package_name_("renbo_whole_body_plan"),
  base_frame_("r_sole"),
  eef_name_("r_gripper")
{
  loadYamlParameter();

  package_path_ = ros::package::getPath(package_name_);

  psm_ = std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ps_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(psm_->getRobotModel()));

  wb_jmg_ = ps_->getRobotModel()->getJointModelGroup(PLANNING_GROUP);

  scg_ = std::shared_ptr<renbo_constraint_sampler::StableConfigGenerator>
      (new renbo_constraint_sampler::StableConfigGenerator(PLANNING_GROUP, SCALE_SP));

  fpp_.reset(new renbo_planner::FinalPosePlanner());

  rrt_.reset(new renbo_planner::RRTConnectPlanner(PLANNING_GROUP, ds_database_path_, solution_file_path_));

  rrt_->setVerbose(verbose_);

  rrt_->setVisualizationSwtich(VISUALIZE_PLANNING_PATH);

  robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("renbo_robot_state", 1);

  goal_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("goal_state", 1);

  trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("renbo_trajectory", 1);

  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame_, "/rviz_visual_markers"));

  ros::Duration(2.0).sleep();

  rviz_visual_tools_->deleteAllMarkers();
  rviz_visual_tools_->enableBatchPublishing();


  if (psm_->getPlanningScene())
  {
    psm_->startStateMonitor();
    //psm_->startSceneMonitor();
    psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                       "renbo_planning_scene_monitor");

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm_);
    planning_scene->setName("renbo_psm_planning_scene");
  }

  ROS_INFO_STREAM("Ready to start");
}

RenboPlanner::~RenboPlanner()
{

}

bool RenboPlanner::generate_ds_database(rrt_planner_msgs::Generate_DS_Configs::Request &req, rrt_planner_msgs::Generate_DS_Configs::Response &res)
{
  renbo_constraint_sampler::StableConfigGenerator::FootSupport support_mode = renbo_constraint_sampler::StableConfigGenerator::DOUBLE_SUPPORT;

  scg_->setVerbose(verbose_);
  scg_->setSupportMode(support_mode);

  if (!scg_->sampleDSConfig(DS_CONFIG_COUNT, ds_database_path_, write_pose_))
  {
    ROS_ERROR("Could not generate stable config");
    exit(1);
  }

  res.num_configs_generated = scg_->getNumConfig();

  return true;
}

bool RenboPlanner::sc_generator_test(rrt_planner_msgs::SC_Generator_Test::Request &req, rrt_planner_msgs::SC_Generator_Test::Response &res)
{
  scg_->setVerbose(verbose_);

  if (!scg_->test())
  {
    ROS_ERROR("Could not stable config generator test");
    exit(1);
  }

  res.result = 2;

  return true;
}

bool RenboPlanner::rrt_planner_test(rrt_planner_msgs::RRT_Planner_Test::Request &req, rrt_planner_msgs::RRT_Planner_Test::Response &res)
{

  loadCollisionEnvironment(scenario_);
  ROS_INFO_STREAM("Meshe is added into planning scene");

  const std::string planning_scene_srv = "get_planning_scene";
  psm_->requestPlanningSceneState(planning_scene_srv);
  planning_scene_monitor::LockedPlanningSceneRW locked_ps_rw(psm_);

  locked_ps_rw->getCurrentStateNonConst().update();
  ps_ = locked_ps_rw->diff();
  ps_->decoupleParent();

  rrt_->setVerbose(verbose_);
  rrt_->updateEnvironment(ps_);

  ROS_INFO_STREAM("Updated rrt planner environment");

  if (!rrt_->TEST(test_flag_))
  {
    ROS_ERROR("could not run rrt planner test");
    return false;
  }

  res.success = true;

  return true;

}

bool RenboPlanner::final_pose_planning(rrt_planner_msgs::Final_Pose_Planning::Request &req, rrt_planner_msgs::Final_Pose_Planning::Response &res)
{
  loadCollisionEnvironment(scenario_);

  const std::string planning_scene_srv = "get_planning_scene";
  psm_->requestPlanningSceneState(planning_scene_srv);
  planning_scene_monitor::LockedPlanningSceneRW locked_ps_rw(psm_);

  locked_ps_rw->getCurrentStateNonConst().update();
  ps_ = locked_ps_rw->diff();
  ps_->decoupleParent();

  fpp_->updateScene(ps_);

  if (!fpp_->TEST())
  {
    ROS_ERROR("could not run final pose planner");
    return false;
  }

  Eigen::Affine3d pose;
  pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ());
  pose.translation() = Eigen::Vector3d(0.5, -0.125, 0.8);

  pose.translation() = Eigen::Vector3d(0.6, 0.125, 0.8);
  rviz_visual_tools_->publishCylinder(pose, rviz_visual_tools::RED, 0.15, 0.03);
  rviz_visual_tools_->trigger();

  res.success = true;

  return true;
}

bool RenboPlanner::demo(rrt_planner_msgs::Final_Pose_Planning::Request &req, rrt_planner_msgs::Final_Pose_Planning::Response &res)
{
  rviz_visual_tools_->deleteAllMarkers();

  robot_state::RobotState robot_state_(ps_->getRobotModel());
  robot_state_.setToDefaultValues();
  robot_state_.update();

  loadCollisionEnvironment(scenario_);

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->setCurrentState(robot_state_);
    scene->getCurrentStateNonConst().update();

    psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    ros::spinOnce();
  }

  fpp_->updateScene(psm_->getPlanningScene());

  Eigen::Affine3d orig_r_eef_config = robot_state_.getGlobalLinkTransform(eef_name_);

  Eigen::Affine3d r_eef_config = fpp_->setRightGripperConfig(orig_r_eef_config);

  std::vector<double> pick_pose(wb_jmg_->getVariableCount());

  bool final_pose_flag = fpp_->solveFinalPose(robot_state_, r_eef_config, pick_pose);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve pick pose fail");
    return false;
  }

  robot_state_.setVariablePositions(wb_jmg_->getJointModelNames(), pick_pose);
  robot_state_.update();

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->setCurrentState(robot_state_);
    scene->getCurrentStateNonConst().update();

    psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE);
    ros::spinOnce();
  }

  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  goal_state_publisher_.publish(robot_state_msg_);

  Eigen::Affine3d grasp_object_pose;
  grasp_object_pose = robot_state_.getGlobalLinkTransform(eef_name_);
  grasp_object_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d transformed_translation = grasp_object_pose.rotation() * Eigen::Vector3d(0.0, -0.15, 0.0);
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
  collision_target_object.header.frame_id = "r_sole";
  collision_target_object.primitives.push_back(target_object);
  collision_target_object.primitive_poses.push_back(pose);
  collision_target_object.operation = moveit_msgs::CollisionObject::ADD;

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->getCurrentStateNonConst().update();
    scene->processCollisionObjectMsg(collision_target_object);
    scene->setObjectColor(collision_target_object.id, getColor(169.0, 169.0, 169.0, 1.0));

    triggerPlanningSceneUpade();
  }

  /*
   * Check final state collision
   */
  bool collision_free = checkCollision(psm_->getPlanningScene());
  if (!collision_free)
  {
    ROS_WARN_STREAM("pick pose is in collision");
    return false;
  }

  /*
   *  RRT-Connect path planning, initial state to pick place.
   */
  std::vector<double> initial_configuration(wb_jmg_->getVariableCount());

  rrt_->updateEnvironment(psm_->getPlanningScene());
  rrt_->setStartGoalConfigs(initial_configuration, pick_pose);

  moveit_msgs::DisplayTrajectory display_trajectory_ = rrt_->solveQuery(20000, 0.1);
  trajectory_publisher_.publish(display_trajectory_);

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.object.header.stamp = ros::Time::now();
  attached_object.object.header.frame_id = base_frame_;
  attached_object.object.id = "cup";
  attached_object.object.operation = moveit_msgs::CollisionObject::ADD;

  attached_object.link_name = eef_name_;

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->setCurrentState(robot_state_);
    scene->getCurrentStateNonConst().update();
    scene->processAttachedCollisionObjectMsg(attached_object);

    psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);\
    ros::spinOnce();
  }

  PAUSE();

  /*
   *  Setup place pose
   */
  r_eef_config.translation().x() += 0.1;
  r_eef_config.translation().y() += 0.2;

  std::vector<double> place_pose(wb_jmg_->getVariableCount());
  final_pose_flag = fpp_->solveFinalPose(robot_state_, r_eef_config, place_pose);
  if (!final_pose_flag)
  {
    ROS_INFO_STREAM("Solve place pose fail");
    return false;
  }

  /*
   *  RRT-Connect path planning, pick to place.
   */
  rrt_->updateEnvironment(psm_->getPlanningScene());
  rrt_->setStartGoalConfigs(pick_pose, place_pose);

  display_trajectory_ = rrt_->solveQuery(20000, 0.1);
  trajectory_publisher_.publish(display_trajectory_);

  return true;

}

void RenboPlanner::loadCollisionEnvironment(int type)
{
  geometry_msgs::Pose table_pose;
  geometry_msgs::Pose pose;

  switch(type)
  {
  case 0:
  {
    table_pose.position.x = 0.6;
    table_pose.position.y = 0.125;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;

    moveit_msgs::CollisionObject collision_mesh_table = loadMeshFromSource("ikea_table.stl", table_pose);

    {
      planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
      scene->getCurrentStateNonConst().update();  // TODO: remove hack to prevent bad transforms
      scene->processCollisionObjectMsg(collision_mesh_table);
      scene->setObjectColor("ikea_table.stl", getColor(222.0, 184.0, 135.0, 1.0));

      triggerPlanningSceneUpade();
    }

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

    publishCollisionObject(collision_mesh_table, getColor(222.0, 184.0, 135.0, 1.0));

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 0.5;
    box.dimensions[1] = 0.08;
    box.dimensions[2] = 0.3;

    pose.position.x = 0.6;
    pose.position.y = 0.125;
    pose.position.z = 0.9;
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

    publishCollisionObject(collision_box, getColor(222.0, 184.0, 135.0, 1.0));

    break;
  }
  case 2:
  {
    table_pose.position.x = 0.95;
    table_pose.position.y = 0.125;
    table_pose.position.z = 0.0;
    table_pose.orientation.w = 0.707;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = -0.707;

    moveit_msgs::CollisionObject collision_mesh_closet = loadMeshFromSource("closet.stl", table_pose);

    {
      planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
      scene->getCurrentStateNonConst().update();
      scene->processCollisionObjectMsg(collision_mesh_closet);
      scene->setObjectColor("closet.stl", getColor(222.0, 184.0, 135.0, 1.0));

      triggerPlanningSceneUpade();
    }

    break;
  }

  } // end switch

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
void RenboPlanner::publishCollisionObject(const moveit_msgs::CollisionObject& msg, const std_msgs::ColorRGBA& color)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);
    scene->getCurrentStateNonConst().update();
    scene->processCollisionObjectMsg(msg);
    scene->setObjectColor(msg.id, color);

    triggerPlanningSceneUpade();
  }

}

void RenboPlanner::triggerPlanningSceneUpade()
{
  psm_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  ros::spinOnce();
}

void RenboPlanner::loadYamlParameter()
{
  nh_.param("robot_description", ROBOT_DESCRIPTION, std::string("robot_description"));
  nh_.param("planning_group", PLANNING_GROUP, std::string("whole_body_fixed"));

  // Stable Configuration Generator Params
  nh_.param("scg_max_samples", MAX_SAMPLES, 3000);
  nh_.param("scg_max_ik_iterations", MAX_IK_ITERATIONS, 5);
  nh_.param("scale_sp", SCALE_SP, 0.8);
  nh_.param("ds_config_count", DS_CONFIG_COUNT, 1);

  // RRT-CONNECT Params
  nh_.param("planner_step_factor", ADVANCE_STEPS, 0.1);
  nh_.param("planner_max_iterations", MAX_EXPAND_ITERATIONS, 8000);
  nh_.param("visualize_planning_path", VISUALIZE_PLANNING_PATH, false);

  // File paths
  nh_.param("file_path_DS_database", ds_database_path_, std::string("package://renbo_whole_body_plan/database/ds_dataset.dat"));
  nh_.param("scene_path", scene_path_, std::string("package://renbo_whole_body_plan/scene/"));

  nh_.param("solution_file_path", solution_file_path_, std::string("package://renbo_whole_body_plan/trajectory/"));

  // Common parameter
  nh_.param("scenario", scenario_, 0);
  nh_.param("test_flag", test_flag_, 0);
  nh_.param("write_path", write_pose_, false);
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

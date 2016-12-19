#include <renbo_whole_body_plan/rrt_connect_planner.h>

namespace renbo_planner
{

RRTConnectPlanner::RRTConnectPlanner(std::string group_name, std::string database_pth, std::string solution_path):
  group_name_(group_name), database_path_(database_pth), visualize_path_(false), base_frame_("r_sole"),
  eef_name_("r_gripper"), nh_("~"), verbose_(false)
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  ps_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_loader_->getModel()));

  wb_jmg_ = ps_->getRobotModel()->getJointModelGroup(group_name_);

  l_leg_jmg_ = ps_->getRobotModel()->getJointModelGroup("left_leg");

  wb_joint_names_ = wb_jmg_->getActiveJointModelNames();

  wb_link_names_ = wb_jmg_->getLinkModelNames();

  wb_num_joint_ = wb_joint_names_.size();

  tree_start_.name = "start_tree";

  tree_goal_.name= "goal_tree";

  robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("renbo_robot_state", 1);

//  scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  trajectory_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("renbo_trajectory", 1);

  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame_, "planned_path_marker"));

  ros::Duration(2.0).sleep();

  rviz_visual_tools_->deleteAllMarkers();
  rviz_visual_tools_->enableBatchPublishing();

  ros::Duration(1.0).sleep();

}

RRTConnectPlanner::~RRTConnectPlanner()
{
  robot_state_publisher_.shutdown();
}

moveit_msgs::DisplayTrajectory RRTConnectPlanner::solveQuery(int max_iter, double max_step_size)
{
  robot_state::RobotState robot_state_(ps_->getRobotModel());

  moveit_msgs::DisplayTrajectory sln_traj;
  node q_rand, q_near;
  status rrt_status, path_found;
  bool swap = false;
  int num_node_generated = 0, swap_cnt = 0;

  ros::Time start_time, end_time;
  ros::Duration dura;

  start_time = ros::Time::now();

  rviz_visual_tools_->deleteAllMarkers();

  for (int i = 0; i < max_iter; i++)
  {
    getRandomStableConfig(q_rand);

    if (swap == false)
    {
      rrt_status = extendTree(tree_start_, q_rand, q_near, max_step_size);

      if (rrt_status != TRAPPED)
      {
        path_found = connectTree(tree_goal_, tree_start_.nodes.back(), q_near, max_step_size);

        if (path_found == REACHED)
        {
          if (verbose_)
          {
            ROS_INFO("Path found, T_goal is connect to T_start");
          }

          writePath(tree_start_, tree_goal_, q_near, 1, sln_traj, solution_file_path_);

          if (visualize_path_)
          {
            EigenSTL::vector_Affine3d path_pts;

            for (int i = 0; i < solution_path_configs_.size(); i++)
            {
              robot_state_.setVariablePositions(wb_joint_names_, solution_path_configs_[i]);
              Eigen::Affine3d eef_pose = robot_state_.getGlobalLinkTransform(eef_name_);

              rviz_visual_tools_->publishSphere(eef_pose, rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
              rviz_visual_tools_->trigger();

              path_pts.push_back(eef_pose);
            }

            rviz_visual_tools_->publishPath(path_pts, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
            rviz_visual_tools_->trigger();
          }

          end_time = ros::Time::now();
          dura =  end_time - start_time;
          num_node_generated =  tree_start_.num_nodes + tree_goal_.num_nodes;

          ROS_INFO_STREAM("\nSummary:\n Totoal elapsed time: " << dura
                          << " seconds \n Generated " << num_node_generated << " nodes \n"
                          << " Swapped " << swap_cnt << " times \n"
                          << " Iterate " << i << " times");

          resetTrees();
          return sln_traj;
        }
        else
        {
          swap = true;
          swap_cnt++;
        }

      }
      else
      {
        swap = true;
        swap_cnt++;
      }
    }
    else
    {
      rrt_status = extendTree(tree_goal_, q_rand, q_near, max_step_size);

      if (rrt_status != TRAPPED)
      {
        path_found = connectTree(tree_start_, tree_goal_.nodes.back(), q_near, max_step_size);

        if (path_found == REACHED)
        {
          ROS_INFO("Path found, T_start is connect to T_goal");

          writePath(tree_start_, tree_goal_, q_near, 2, sln_traj, solution_file_path_);

          if (visualize_path_)
          {
            EigenSTL::vector_Affine3d path_pts;

            for (int i = 0; i < solution_path_configs_.size(); i++)
            {
              robot_state_.setVariablePositions(wb_joint_names_, solution_path_configs_[i]);
              Eigen::Affine3d eef_pose = robot_state_.getGlobalLinkTransform(eef_name_);

              rviz_visual_tools_->publishSphere(eef_pose, rviz_visual_tools::ORANGE, rviz_visual_tools::MEDIUM);
              rviz_visual_tools_->trigger();

              path_pts.push_back(eef_pose);
            }

            rviz_visual_tools_->publishPath(path_pts, rviz_visual_tools::YELLOW, rviz_visual_tools::MEDIUM);
            rviz_visual_tools_->trigger();
          }

          end_time = ros::Time::now();
          dura =  end_time - start_time;
          num_node_generated =  tree_start_.num_nodes + tree_goal_.num_nodes;

          ROS_INFO_STREAM("\nSummary:\n Totoal elapsed time: " << dura
                          << " seconds \n Generated " << num_node_generated << " nodes \n"
                          << " Swapped " << swap_cnt << " times \n"
                          << " Iterate " << i << " times");


          resetTrees();
          return sln_traj;
        }
        else
        {
          swap = false;
          swap_cnt++;
        }

      }
      else
      {
        swap = false;
        swap_cnt++;
      }

    }

  }

  resetTrees();

  ROS_ERROR("No solution path found");

  return sln_traj;
}


bool RRTConnectPlanner::setStartGoalConfigs(std::vector<double> start_config , std::vector<double> goal_config)
{
  node start, goal;

  start.config = start_config;
  start.index = 0;
  start.predecessor_index = 0;

  goal.config = goal_config;
  goal.index = 0;
  goal.predecessor_index = 0;

  if (!checkCollision(ps_, start.config))
  {
    ROS_ERROR("start config is invalid");
    return false;
  }

  if (!checkCollision(ps_, goal.config))
  {
    ROS_ERROR("goal config is invalid");
    return false;
  }

  tree_start_.nodes.push_back(start);
  tree_start_.num_nodes = 1;

  tree_goal_.nodes.push_back(goal);
  tree_goal_.num_nodes = 1;

  return true;
}

status RRTConnectPlanner::extendTree(tree &input_tree, node q_rand, node &q_near, double max_step_size)
{
  status stat = ADVANCED;
  node q_new, q_new_modified;
  bool enforce_ds = false;

  robot_state::RobotState robot_state_(ps_->getRobotModel());
  robot_state_.setToDefaultValues();

  // finding the nearest node of q_rand and save as q_near
  findNearestNeighbour(input_tree, q_rand, q_near);

  // try to connect q_rand and q_near with fixed step size
  stat = generate_q_new(q_near, q_rand, q_new, max_step_size);

  if (stat == ADVANCED)
  {
    // enforce the new generate configuration is in double support phase
    wb_jnt_pos_map_.clear();
    for (int i = 0; i < wb_joint_names_.size(); i++)
    {
      wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_joint_names_[i], q_new.config[i]));
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

    enforce_ds = ds_constraint_.enforceDSLeftLeg(robot_state_, l_leg_jmg_);

    if (!enforce_ds)
    {
      if(verbose_)
      {
        ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "extend tree: can't enforce double support phase");
      }

      stat = TRAPPED;
    }
    else
    {
      ps_->setCurrentState(robot_state_);

      bool collision_free = false;
      collision_free = checkCollision(robot_state_);

      if (collision_free)
      {
        robot_state_.copyJointGroupPositions(wb_jmg_, q_new_modified.config);

        addConfigtoTree(input_tree, q_near, q_new_modified);

        stat = ADVANCED;

        if(verbose_)
        {
          ROS_INFO_STREAM("current proccesing node: " << input_tree.nodes.back().index);

          moveit_msgs::DisplayRobotState robot_state_msg_;
          robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
          robot_state_publisher_.publish(robot_state_msg_);

          ros::Duration(0.05).sleep();
        }
      }
      else
      {
        if(verbose_)
        {
          moveit_msgs::DisplayRobotState robot_state_msg_;
          std_msgs::ColorRGBA collide_color;

          collide_color.r = 1.0;
          collide_color.g = 0.0;
          collide_color.b = 0.0;
          collide_color.a = 1.0;

          robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);

          const std::vector<const moveit::core::LinkModel*>& link_models = robot_state_.getRobotModel()->getLinkModelsWithCollisionGeometry();

          robot_state_msg_.highlight_links.resize(link_models.size());

          for (std::size_t i = 0; i < wb_link_names_.size(); i++)
          {
            robot_state_msg_.highlight_links[i].id = link_models[i]->getName();
            robot_state_msg_.highlight_links[i].color = collide_color;
          }

          robot_state_publisher_.publish(robot_state_msg_);

          ROS_INFO_STREAM("extend tree: state in collision return trapped");
        }

        stat = TRAPPED;

      }
    }

  }
  else if (stat == REACHED)
  {
    if(verbose_)
      ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "Reached q_rand");
  }
  else
  {
    stat = TRAPPED;

    if(verbose_)
      ROS_INFO_STREAM("extend tree: state in trapped");
  }

  return stat;

}

status RRTConnectPlanner::connectTree(tree &input_tree, node q_connect, node &q_near, double max_step_size)
{
  status stat = ADVANCED;
  node current_q_near;
  int prev_node_index = -1;
  int iteration_cnt = 0;

  // test
  findNearestNeighbour(input_tree, q_connect, q_near);

  current_q_near = q_near;

  while (stat == ADVANCED)
  {
    stat = extendTree(input_tree, q_connect, current_q_near, max_step_size);

    if (stat == REACHED)
    {
      q_near = current_q_near;
    }
    else
    {
      if (current_q_near.index == prev_node_index)
      {
        // test for solving rrt loop stuck
        q_near = current_q_near;

        stat = TRAPPED;
        ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_GREEN << "Cant find closer node return TRAPPED");
        return stat;
      }
      else
      {
        current_q_near = input_tree.nodes.back();
        prev_node_index = current_q_near.index -1;
      }
    }

    iteration_cnt++;

    if (iteration_cnt == 300)
      return TRAPPED;
  }

  if (verbose_)
  {
    ROS_INFO_STREAM("Connect Tree State: " << stat);
  }

  return stat;
}

void RRTConnectPlanner::findNearestNeighbour(tree T_current, node q_rand, node &nearest_neighbor)
{
  int nn_index = -1;
  std::vector<double> distance(wb_num_joint_);
  double sum_sqrt = 0.0;
  double dist_norm = 0.0;
  double best_norm = 100000.0;

  // start calculating the distance from q_rand to each node
  for (std::size_t i = 0; i < T_current.num_nodes; i++)
  {
    sum_sqrt = 0.0;

    // calculate euclidean distance between q random to nodes in the input tree
    for (std::size_t n = 0; n < wb_num_joint_; n++)
    {
      distance[n] = q_rand.config[n] - T_current.nodes[i].config[n];
      sum_sqrt += distance[n]*distance[n];
    }

    dist_norm = std::sqrt(sum_sqrt);

    if (dist_norm < best_norm)
    {
      best_norm = dist_norm;
      nn_index = i;
    }
  }

  //Check if a neighbor has been found
  if (nn_index < 0)
  {
    if (verbose_)
      ROS_INFO_STREAM("No nearest neighbor found");
  }
  else
  {
    //Assign index of q_near found in the Tree to nearest_neighbor
    nearest_neighbor.index = T_current.nodes[nn_index].index;

    //Set the predecessor_index
    nearest_neighbor.predecessor_index = T_current.nodes[nn_index].predecessor_index;

    //Set the configuration of nearest_neighbor
    nearest_neighbor.config =  T_current.nodes[nn_index].config;


    //ROS_INFO_STREAM("nearest neighbor index: " << nearest_neighbor.index);

    //Set the cart_hand_pose_index of the nearest neighbor (only used for manipulation of articulated ojects)
//      if (articulation_constraint_active_ == true)
//        nearest_neighbor.cart_hand_pose_index =  T_current.nodes[nn_index].cart_hand_pose_index;
  }
}

// TODO: add collision and stability checking
status RRTConnectPlanner::generate_q_new(node q_near, node q_rand, node &q_new, double step_size)
{
  status stat = TRAPPED;
  std::vector<double> distance(wb_num_joint_);
  double sum_sqrt = 0.0;
  double dist_norm = 0.0;
  bool valid = false;

  for (std::size_t i = 0; i < wb_num_joint_; i++)
  {
    distance[i] = q_rand.config[i] - q_near.config[i];
    sum_sqrt += distance[i]*distance[i];
  }

  dist_norm = std::sqrt(sum_sqrt);

  std::vector<double> direction(wb_num_joint_); // unit vector of the distance
  std::vector<double> increment(wb_num_joint_);
  double sum_sqrt_incr = 0.0;
  double dist_norm_incr = 0.0;

  for (std::size_t i = 0; i < wb_num_joint_; i++)
  {
    direction[i] = distance[i] / dist_norm;
    increment[i] = step_size * direction[i]; // *joint_weight_?

    sum_sqrt_incr += increment[i]*increment[i];
  }

  dist_norm_incr = std::sqrt(sum_sqrt_incr);

  std::vector<double> q_new_temp(wb_num_joint_);

  if (dist_norm_incr < dist_norm)
  {
    for (std::size_t i = 0; i < wb_num_joint_; i++)
    {
      q_new_temp[i] = q_near.config[i] + increment[i];
    }

    q_new.config = q_new_temp;

    stat = ADVANCED;

  }
  else
  {
    q_new.config = q_rand.config;
    stat = REACHED;
  }

  if (verbose_)
  {
    ROS_INFO_STREAM("dist_norm: " << dist_norm);
  }

  return stat;
}

void RRTConnectPlanner::addConfigtoTree(tree &input_tree, node q_near, node q_new_modified)
{
  q_new_modified.index = input_tree.nodes.size();
  q_new_modified.predecessor_index = q_near.index;

  input_tree.nodes.push_back(q_new_modified);
  input_tree.num_nodes = input_tree.nodes.size();
}

void RRTConnectPlanner::writePath(tree T_start, tree T_goal, node q_near, int connector, moveit_msgs::DisplayTrajectory &trajectory, std::string solution_file_path)
{
  ROS_INFO_STREAM("Write path to trajectory file");

  std::vector< std::vector<double> > start_path;
  std::vector< std::vector<double> > goal_path;

  int index_start = 0;
  int index_goal = 0;

  if (connector == 1)
  {
    index_start = T_start.nodes.back().index;
    index_goal = q_near.index;
  }
  else if (connector == 2)
  {
    index_start = q_near.index;
    index_goal = T_goal.nodes.back().index;
  }
  else
  {
    ROS_ERROR("wrong connector value");
  }

  while (index_start > 0)
  {
    start_path.push_back(T_start.nodes[index_start].config);
    index_start = T_start.nodes[index_start].predecessor_index;
  }

  start_path.push_back(T_start.nodes[index_start].config);

  while (index_goal > 0)
  {
    goal_path.push_back(T_goal.nodes[index_goal].config);
    index_goal = T_goal.nodes[index_goal].predecessor_index;
  }

  goal_path.push_back(T_goal.nodes[index_goal].config);

  solution_path_configs_.clear();
  for (int i = (start_path.size()-1); i >= 0; i--)
  {
    solution_path_configs_.push_back(start_path[i]);
  }

  for (unsigned int i = 0; i < goal_path.size(); i++)
  {
    solution_path_configs_.push_back(goal_path[i]);
  }

  ROS_INFO_STREAM("Doing path short cutting...");

  /******************* PATH SHORTCUTTER *******************/
  Trajectory shortcutted_path;
  int num_intermediate_waypoints_approach = 100;

  bool shorcut_flag = false;
  shorcut_flag = pathShortCutter(solution_path_configs_, shortcutted_path, num_intermediate_waypoints_approach);

  ROS_INFO_STREAM("Found short cut path");

  if (visualize_path_)
  {
    robot_state::RobotState robot_state_(ps_->getRobotModel());
    EigenSTL::vector_Affine3d path_pts_;
    for (int i = 0; i < shortcutted_path.size(); i++)
    {
      robot_state_.setVariablePositions(wb_joint_names_, shortcutted_path[i]);
      path_pts_.push_back(robot_state_.getGlobalLinkTransform(eef_name_));
    }

    rviz_visual_tools_->publishPath(path_pts_, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
    rviz_visual_tools_->trigger();

  }

  if (shorcut_flag)
  {
    // generate trajectory from solution path
    generateTrajectory(shortcutted_path, shortcutted_path.size(), trajectory);

    //solution_file_path.append("whole_body_trajectory.txt");
    solution_file_path = "/home/shtseng/catkin_ws/src/renbo_whole_body_plan/trajectory/whole_body_trajectory.txt";
    std::ofstream solution_path;
    solution_path.open(solution_file_path.c_str());

    if (!solution_path)
    {
      ROS_ERROR("Could not open solution file");
      exit(1);
    }

    for (std::size_t i = 0; i < shortcutted_path.size(); i++)
    {
      for (std::size_t j = 0; j < wb_num_joint_; j++)
      {
        solution_path << shortcutted_path[i][j] << " ";
      }

      solution_path << std::endl;
    }
    solution_path.close();

    ROS_INFO_STREAM("Wrote data to file complete");
  }

}

void RRTConnectPlanner::generateTrajectory(std::vector<std::vector<double> > path, int num_configs, moveit_msgs::DisplayTrajectory &disp_traj)
{
  moveit_msgs::RobotState start_state;
  start_state.joint_state.name.resize(wb_num_joint_);
  start_state.joint_state.position.resize(wb_num_joint_);

  for (int i = 0; i < wb_num_joint_; i++)
  {
    start_state.joint_state.name[i] = wb_joint_names_[i];
    start_state.joint_state.position[i] = path[0][i];
  }

  //start_state.multi_dof_joint_state.joint_names.push_back("virtual_joint");

  trajectory_msgs::JointTrajectory jnt_traj_msgs;
  jnt_traj_msgs.joint_names.resize(wb_num_joint_);
  jnt_traj_msgs.points.resize(num_configs);

  jnt_traj_msgs.header.frame_id = "odom_combined";

  for (int i = 0; i < wb_num_joint_; i++)
  {
    jnt_traj_msgs.joint_names[i] = wb_joint_names_[i];
  }

  double increment = 2.0;
  for (int i = 0; i < num_configs; i++)
  {
    jnt_traj_msgs.points[i].positions.resize(wb_num_joint_);

    for (int j = 0; j < wb_num_joint_; j++)
    {
      jnt_traj_msgs.points[i].positions[j] = path[i][j];
      jnt_traj_msgs.points[i].time_from_start = ros::Duration(increment);
    }

    increment = increment + 1.0;
  }

  moveit_msgs::RobotTrajectory moveit_robot_traj;
  moveit_robot_traj.joint_trajectory = jnt_traj_msgs;

  std::map<std::string, double> initial_configuration;
  for (int i = 0; i < wb_num_joint_; i++)
  {
    initial_configuration[wb_joint_names_[i]] = moveit_robot_traj.joint_trajectory.points[0].positions[i];
  }

  robot_state::RobotState init_state(ps_->getRobotModel());

  init_state.setToDefaultValues();
  init_state.setVariablePositions(initial_configuration);

  robot_trajectory::RobotTrajectory robot_traj(ps_->getRobotModel(), group_name_);
  robot_traj.setRobotTrajectoryMsg(init_state, moveit_robot_traj);

  trajectory_processing::IterativeParabolicTimeParameterization traj_smoother;
  traj_smoother.computeTimeStamps(robot_traj);

  robot_traj.getRobotTrajectoryMsg(moveit_robot_traj);

  trajectory_msgs::JointTrajectory smooth_jnt_traj_msgs;
  smooth_jnt_traj_msgs.joint_names.resize(wb_num_joint_);
  smooth_jnt_traj_msgs.points.resize(num_configs);
  smooth_jnt_traj_msgs = moveit_robot_traj.joint_trajectory;

  ROS_INFO_STREAM("Number of waypoints in robot trajectory " << robot_traj.getWayPointCount());

  for (std::size_t i = 0; i < moveit_robot_traj.joint_trajectory.points.size(); i++)
  {
    smooth_jnt_traj_msgs.points[i].time_from_start = moveit_robot_traj.joint_trajectory.points[i].time_from_start + ros::Duration(2.0);
  }

  moveit_robot_traj.joint_trajectory = smooth_jnt_traj_msgs;

  // Setup input display trajectory
  disp_traj.model_id = ps_->getRobotModel()->getName();
  disp_traj.trajectory_start = start_state;
  disp_traj.trajectory.clear();
  disp_traj.trajectory.push_back(moveit_robot_traj);

}

bool RRTConnectPlanner::pathShortCutter(Trajectory raw_path, Trajectory &shortcutted_path, int num_intermediate_waypoints)
{
  Trajectory path_temp;
  std::vector<double> start_config, goal_config, current_goal_config;
  int current_start_index = 0;

  start_config = raw_path.front();
  goal_config = raw_path.back();

  bool reach_goal = false;
  int loop_count = 0, max_interation = 500;

  moveit_msgs::RobotTrajectory current_robot_trajectory_msgs;
  moveit_msgs::DisplayTrajectory waypoint_interpolate;

  path_temp.push_back(raw_path[0]);

  while (reach_goal != true && loop_count < max_interation)
  {
    waypoint_interpolate = interpolateWaypoints(start_config, goal_config, num_intermediate_waypoints);
    current_robot_trajectory_msgs = waypoint_interpolate.trajectory.at(0);

    if (ps_->isPathValid(waypoint_interpolate.trajectory_start, current_robot_trajectory_msgs))
    {
      path_temp.push_back(goal_config);
      reach_goal = true;
    }
    else
    {
      for (int i = (raw_path.size()-2); i >= 0; i--)
      {
        current_goal_config = raw_path[i];

        if (i == current_start_index)
        {
          start_config = raw_path[i+1];
          current_start_index = i+1;
          path_temp.push_back(start_config);
          break;
        }

        waypoint_interpolate = interpolateWaypoints(start_config, current_goal_config, num_intermediate_waypoints);
        current_robot_trajectory_msgs = waypoint_interpolate.trajectory.at(0);

        if (ps_->isPathValid(waypoint_interpolate.trajectory_start, current_robot_trajectory_msgs))
        {
          start_config = current_goal_config;
          current_start_index = i;
          path_temp.push_back(current_goal_config);
          break;
        }
      }
    }

    loop_count++;
  }

  if (reach_goal)
  {
    ROS_INFO_STREAM("Short cutted path has " << path_temp.size() << " way points");
    linearInterpolation(path_temp, shortcutted_path, num_intermediate_waypoints);
  }
  else
  {
    ROS_INFO_STREAM("Path shortcut fail");
    return false;
  }

  return true;
}

moveit_msgs::DisplayTrajectory RRTConnectPlanner::interpolateWaypoints(std::vector<double> waypoint_start,
                                                                       std::vector<double> waypoint_goal,
                                                                       int num_intermediate_waypoints)
{
  moveit_msgs::DisplayTrajectory interpolated_trajectory;
  moveit_msgs::RobotTrajectory robot_trajectory_msgs;
  moveit_msgs::RobotState initial_state_msgs;
  trajectory_msgs::JointTrajectory joint_trajectory_msgs;

  Trajectory interpolated_waypoints;

  std::vector<double> step_width(wb_num_joint_);
  for (int i = 0; i < wb_num_joint_; i++)
  {
    step_width[i] = (waypoint_goal[i] - waypoint_start[i]) / (num_intermediate_waypoints+1);
  }

  interpolated_waypoints.push_back(waypoint_start);

  std::vector<double> tmp_config(wb_num_joint_);
  for (int i = 1; i <= (num_intermediate_waypoints+1); i++)
  {
    for(int j = 0; j < wb_num_joint_; j++)
    {
      tmp_config[j] = waypoint_start[j] + step_width[j]*i;
    }

    interpolated_waypoints.push_back(tmp_config);
  }

  joint_trajectory_msgs.header.frame_id = "odom_combined";
  joint_trajectory_msgs.joint_names.resize(wb_num_joint_);
  joint_trajectory_msgs.points.resize(num_intermediate_waypoints+2);

  for (int i = 0; i < wb_num_joint_; i++)
  {
    joint_trajectory_msgs.joint_names[i] = wb_joint_names_[i];
  }

  float increment = 0.0;
  for (int i = 0; i < (num_intermediate_waypoints+2); i++)
  {
    joint_trajectory_msgs.points[i].positions.resize(wb_num_joint_);

    for (int j = 0; j < wb_num_joint_; j++)
    {
      joint_trajectory_msgs.points[i].positions[j] = interpolated_waypoints[i][j];
      joint_trajectory_msgs.points[i].time_from_start = ros::Duration(increment);
    }
    increment += 1.0;
  }

  initial_state_msgs.joint_state.name.resize(wb_num_joint_);
  initial_state_msgs.joint_state.position.resize(wb_num_joint_);

  for (int i = 0; i < wb_num_joint_; i++)
  {
    initial_state_msgs.joint_state.name[i] = wb_joint_names_[i];
    initial_state_msgs.joint_state.position[i] = waypoint_start[i];
  }

  //initial_state_msgs.multi_dof_joint_state.joint_names.push_back("virtual_joint");

  robot_trajectory_msgs.joint_trajectory = joint_trajectory_msgs;

  interpolated_trajectory.model_id = ps_->getRobotModel()->getName();
  interpolated_trajectory.trajectory_start = initial_state_msgs;
  interpolated_trajectory.trajectory.clear();
  interpolated_trajectory.trajectory.push_back(robot_trajectory_msgs);

  return interpolated_trajectory;
}

void RRTConnectPlanner::linearInterpolation(Trajectory short_path,
                                            Trajectory &interpolated_path,
                                            int num_intermediate_waypoints)
{
  interpolated_path.push_back(short_path[0]);

  std::vector<double> step_width(wb_num_joint_);
  std::vector<double> waypoints(wb_num_joint_);

  for (int i = 0; i < short_path.size()-1; i++)
  {
    for (int j = 0; j < wb_num_joint_; j++)
    {
      step_width[j] = (short_path[i+1][j]-short_path[i][j]) / (num_intermediate_waypoints+1);
    }

    for (int j = 1; j <= num_intermediate_waypoints+1; j++)
    {
      for (int k = 0; k < wb_num_joint_; k++)
      {
        waypoints[k] = short_path[i][k] + (step_width[k] * j);
      }

      interpolated_path.push_back(waypoints);
    }
  }
}

void RRTConnectPlanner::getRandomStableConfig(node &sample)
{
  if (ds_database_configs_.empty())
  {
    if (!loadDSDatabase(database_path_))
    {
      ROS_ERROR("Load database fail fail");
    }
  }

  int lb = 0, ub = ds_database_config_count_ - 1;
  int rnd_idx = floor(rng_.uniformReal(lb, ub));

  sample.config = ds_database_configs_[rnd_idx];

  if (verbose_)
  {
    ROS_INFO_STREAM("Random sampled index: " << rnd_idx);
  }
}

bool RRTConnectPlanner::loadDSDatabase(std::string path)
{
  std::string line;
  ds_database_config_count_ = 0;

  std::vector<double> config;
  double q_in;

  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_CYAN << "Loading double support database..." << MOVEIT_CONSOLE_COLOR_RESET);

  ds_config_file_.open(path.c_str());
  if (!ds_config_file_)
  {
    ROS_ERROR("Open dataset fail");
    return false;
  }

  ds_database_configs_.clear();

  while (ds_config_file_.good())
  {
    std::getline(ds_config_file_, line);
    std::istringstream in(line);

    while (in >> q_in)
    {
      config.push_back(q_in);
    }

    ds_database_configs_.push_back(config);
    config.clear();

    if (ds_config_file_.eof())
    {
      break;
    }
    ds_database_config_count_++;
  }

  ROS_INFO_STREAM("Load total " << ds_database_config_count_ << " configs");

  ds_config_file_.close();

  return true;
}

void RRTConnectPlanner::resetTrees()
{
  tree_start_.nodes.clear();
  tree_start_.num_nodes = 0;
  tree_goal_.nodes.clear();
  tree_goal_.num_nodes = 0;
}

void RRTConnectPlanner::updateEnvironment(const planning_scene::PlanningScenePtr& scene)
{
  ps_ = scene;
}

bool RRTConnectPlanner::checkCollision(planning_scene::PlanningScenePtr ps_, const std::vector<double> config)
{
  bool validity = false;

  robot_state::RobotState robot_state(ps_->getRobotModel());

  for (int i = 0; i < wb_joint_names_.size(); i++)
  {
    wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_joint_names_[i], config[i]));
  }

  wb_jnt_pos_map_["head_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["head_pitch_joint"] = 0.0;
  wb_jnt_pos_map_["l_shoulder_pitch_joint"] = -0.785;
  wb_jnt_pos_map_["l_shoulder_roll_joint"] = 0.0;
  wb_jnt_pos_map_["l_shoulder_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["l_elbow_joint"] = 1.0472;
  wb_jnt_pos_map_["l_wrist_yaw_joint"] = 0.0;
  wb_jnt_pos_map_["l_wrist_pitch_joint"] = -0.523;

  robot_state.setVariablePositions(wb_jnt_pos_map_);
  robot_state.update();

  ps_->setCurrentState(robot_state);

  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;
  collision_detection::AllowedCollisionMatrix acm = ps_->getAllowedCollisionMatrix();

  collision_req.group_name = group_name_;
  collision_res.clear();

  ps_->checkCollision(collision_req, collision_res, robot_state, acm);

  if (collision_res.collision == 0)
  {
    validity = true;
  }

  return validity;
}

bool RRTConnectPlanner::checkCollision(moveit::core::RobotState robot_state_)
{
  bool collision_free = false;

  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;
  collision_detection::AllowedCollisionMatrix acm_ = ps_->getAllowedCollisionMatrix();

  collision_req.group_name = group_name_;
  collision_res.clear();

  ps_->checkCollision(collision_req, collision_res, robot_state_, acm_);

  if (collision_res.collision == 0)
  {
    collision_free = true;
  }
  else
  {
    collision_free = false;
  }

  return collision_free;
}

bool RRTConnectPlanner::TEST(int test_flag)
{
  robot_state::RobotState robot_state_(ps_->getRobotModel());
  moveit_msgs::DisplayRobotState robot_state_msg_;

  robot_state_.setToDefaultValues();

  if (test_flag == 0)
  {
    if (!loadDSDatabase(database_path_))
    {
      ROS_ERROR("Load double support config fail");
      return false;
    }

    robot_state::RobotState current_state(ps_->getRobotModel());

    node q_rand, q_near, start_node;
    tree start_tree;

    std::vector<double> start_config;
    start_config.resize(wb_num_joint_);
    std::fill(start_config.begin(), start_config.end(), 0.0);

    start_node.config = start_config;
    start_node.index = 0;
    start_node.predecessor_index = 0;

    start_tree.nodes.push_back(start_node);
    start_tree.num_nodes = 1;

    getRandomStableConfig(q_rand);

    wb_jnt_pos_map_.clear();
    for (int i = 0; i < wb_joint_names_.size(); i++)
    {
      wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_joint_names_[i], q_rand.config[i]));
    }

    current_state.setVariablePositions(wb_jnt_pos_map_);
    current_state.update();

    robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);
    robot_state_publisher_.publish(robot_state_msg_);

    ps_->setCurrentState(current_state);

//    moveit_msgs::PlanningScene planning_scene_msg;
//    ps_->getPlanningSceneMsg(planning_scene_msg);

//    planning_scene_msg.is_diff = true;
//    planning_scene_msg.robot_state.is_diff = true;

//    scene_publisher_.publish(planning_scene_msg);

    bool collision_free = false;
    collision_free = checkCollision(current_state);

    if (collision_free)
    {
      ROS_INFO_STREAM("Robot is in collision free");

      PAUSE();

      status tree_stat = ADVANCED;

      findNearestNeighbour(start_tree, q_rand, q_near);

      tree_stat = connectTree(start_tree, q_rand, q_near, 0.1);

    }
    else
    {
      std_msgs::ColorRGBA collide_color;
      collide_color.r = 1.0;
      collide_color.g = 0.0;
      collide_color.b = 0.0;
      collide_color.a = 1.0;

      robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);

      const std::vector<const moveit::core::LinkModel*>& link_models = current_state.getRobotModel()->getLinkModelsWithCollisionGeometry();

      robot_state_msg_.highlight_links.resize(link_models.size());

      for (std::size_t i = 0; i < wb_link_names_.size(); i++)
      {
        robot_state_msg_.highlight_links[i].id = link_models[i]->getName();
        robot_state_msg_.highlight_links[i].color = collide_color;
      }

      robot_state_publisher_.publish(robot_state_msg_);
      ros::Duration(0.5).sleep();
    }
  }
  else if (test_flag == 1)
  {
    if (!loadDSDatabase(database_path_))
    {
      ROS_ERROR("Load database fail fail");
      return false;
    }

    std::vector<double> start_config;
    start_config.resize(wb_num_joint_);
    std::fill(start_config.begin(), start_config.end(), 0.0);

    node q_rand;
    getRandomStableConfig(q_rand);

    wb_jnt_pos_map_.clear();
    for (int i = 0; i < wb_joint_names_.size(); i++)
    {
      wb_jnt_pos_map_.insert(std::pair<std::string, double>(wb_joint_names_[i], q_rand.config[i]));
    }

    robot_state::RobotState current_state(ps_->getRobotModel());
    current_state.setVariablePositions(wb_jnt_pos_map_);
    current_state.update();

    robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);
    robot_state_publisher_.publish(robot_state_msg_);

    ps_->setCurrentState(current_state);

//    moveit_msgs::PlanningScene planning_scene_msg;
//    ps_->getPlanningSceneMsg(planning_scene_msg);

//    planning_scene_msg.is_diff = true;
//    planning_scene_msg.robot_state.is_diff = true;

//    scene_publisher_.publish(planning_scene_msg);

    bool collision_free = false;
    collision_free = checkCollision(current_state);

    if (collision_free)
    {
      if (!setStartGoalConfigs(start_config, q_rand.config))
      {
        ROS_INFO_STREAM("start or goal config invalid");
      }

      PAUSE();
      moveit_msgs::DisplayTrajectory  solution = solveQuery(20000, 0.1);

      if (verbose_)
      {
        trajectory_publisher_.publish(solution);
      }
    }
    else
    {
      std_msgs::ColorRGBA collide_color;
      collide_color.r = 1.0;
      collide_color.g = 0.0;
      collide_color.b = 0.0;
      collide_color.a = 1.0;

      robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);

      const std::vector<const moveit::core::LinkModel*>& link_models = current_state.getRobotModel()->getLinkModelsWithCollisionGeometry();

      robot_state_msg_.highlight_links.resize(link_models.size());

      for (std::size_t i = 0; i < wb_link_names_.size(); i++)
      {
        robot_state_msg_.highlight_links[i].id = link_models[i]->getName();
        robot_state_msg_.highlight_links[i].color = collide_color;
      }

      robot_state_publisher_.publish(robot_state_msg_);
      ros::Duration(0.1).sleep();
    }

  }
  else if (test_flag == 3)
  {
    if (!loadDSDatabase(database_path_))
    {
      ROS_ERROR("Load database fail fail");
      return false;
    }

    robot_state::RobotState current_state = ps_->getCurrentStateNonConst();
    int rnd_idx = floor(rng_.uniformReal(0, ds_database_config_count_-1));

    current_state.setVariablePositions(wb_joint_names_, ds_database_configs_[rnd_idx]);
    current_state.update();

    robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);

    ps_->setCurrentState(current_state);

//    moveit_msgs::PlanningScene planning_scene_msg;
//    ps_->getPlanningSceneMsg(planning_scene_msg);

//    planning_scene_msg.is_diff = true;
//    planning_scene_msg.robot_state.is_diff = true;

//    scene_publisher_.publish(planning_scene_msg);
//    robot_state_publisher_.publish(robot_state_msg_);

    ros::Duration(0.5).sleep();

    bool collision_free = false;
    collision_detection::AllowedCollisionMatrix acm = ps_->getAllowedCollisionMatrix();
    collision_free = checkCollision(current_state);

    if (collision_free)
    {
      ROS_INFO_STREAM("Robot is in collision free");
    }
    else
    {
      std_msgs::ColorRGBA collide_color;
      collide_color.r = 1.0;
      collide_color.g = 0.0;
      collide_color.b = 0.0;
      collide_color.a = 1.0;

      robot_state::robotStateToRobotStateMsg(current_state, robot_state_msg_.state);

      const std::vector<const moveit::core::LinkModel*>& link_models = current_state.getRobotModel()->getLinkModelsWithCollisionGeometry();

      robot_state_msg_.highlight_links.resize(link_models.size());

      for (std::size_t i = 0; i < wb_link_names_.size(); i++)
      {
        robot_state_msg_.highlight_links[i].id = link_models[i]->getName();
        robot_state_msg_.highlight_links[i].color = collide_color;
      }

      robot_state_publisher_.publish(robot_state_msg_);
      ros::Duration(0.5).sleep();
    }
  }

  ROS_INFO("Finished Test");

  return true;
}

void RRTConnectPlanner::setDatabasePath(const std::string &path)
{
  database_path_.clear();
  database_path_ = path;
}

void RRTConnectPlanner::setVisualizationSwtich(bool on_off)
{
  visualize_path_ = on_off;
}

void RRTConnectPlanner::setVerbose(bool verbose)
{
  verbose_ = verbose;
}

void RRTConnectPlanner::PAUSE()
{
  ROS_INFO_STREAM(MOVEIT_CONSOLE_COLOR_BROWN << "Press any key to continue");
  std::string dummy;
  std::getline(std::cin, dummy);
}

}


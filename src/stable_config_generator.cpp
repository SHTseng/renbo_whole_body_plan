#include <renbo_whole_body_plan/stable_config_generator.h>
#include <vector>

namespace renbo_constraint_sampler
{

StableConfigGenerator::StableConfigGenerator(const std::string &group_name, const double& scale_sp):
    nh_("~"),
    group_name_(group_name),
    support_mode_(DOUBLE_SUPPORT),
    verbose_(false)
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  robot_model_ = robot_model_loader_->getModel();

  ps_ = std::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_));

  whole_body_jmg_ = robot_model_->getJointModelGroup(group_name_);

  l_leg_jmg_ = robot_model_->getJointModelGroup("left_leg");

  root_link_name_ = robot_model_->getRootLinkName();

  whole_body_joint_names_ = whole_body_jmg_->getActiveJointModelNames();

  //support_mode_ = DOUBLE_SUPPORT;

  robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("renbo_robot_state", 1);

  goal_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("goal_state", 1);

  visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("stability_visualization", 1);

  support_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);

  com_pub_= nh_.advertise<visualization_msgs::Marker>("com", 10);

  pcom_pub_ = nh_.advertise<visualization_msgs::Marker>("projected_com", 10);

  initialize(scale_sp);

}

StableConfigGenerator::~StableConfigGenerator()
{

}

void StableConfigGenerator::initialize(double scale_sp)
{
  renbo_urdf_ = ps_->getRobotModel()->getURDF();

  if (!initSupportPolygon(scale_sp))
  {
    ROS_ERROR_STREAM("Initial support polygon fail");
    exit(0);
  }

  if (!kdl_parser::treeFromUrdfModel(*renbo_urdf_, kdl_tree_))
  {
    ROS_ERROR_STREAM("Could not initialize tree object");
    exit(0);
  }

  addChildren(kdl_tree_.getRootSegment());

}

bool StableConfigGenerator::sampleDSConfig(int config_count, std::string file_destination, bool write_pose)
{
  int ssc_count = 0;

  ds_database_.open(file_destination.c_str(), std::ios_base::app);
  if(!ds_database_)
  {
    ROS_ERROR("Could not open stable config database");
    exit(1);
  }

  robot_state::RobotState robot_state_(ps_->getRobotModel());

  robot_state_ = ps_->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();

  int loop_count = 0;
  moveit_msgs::DisplayRobotState robot_state_msg_;

  ros::Time start_time = ros::Time::now();

  while(1)
  {
    whole_body_jmg_->getVariableRandomPositions(rng_, whole_body_joint_values_);
    wb_jnt_pos_map_.clear();

    for (int i = 0; i < whole_body_joint_names_.size(); i++)
    {
      wb_jnt_pos_map_.insert(std::pair<std::string, double>(whole_body_joint_names_[i], whole_body_joint_values_[i]));
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

    bool enforce_ds = false;
    enforce_ds = ds_constraint_kin_.enforceDSLeftLeg(robot_state_, l_leg_jmg_);

    if(enforce_ds)
    {
      if(verbose_)
      {
        robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
        robot_state_publisher_.publish(robot_state_msg_);

        ROS_INFO_STREAM("state is in double support phase");
      }

      if (isFeasible(robot_state_))
      {
        // update the modified robot configuration to joint value vector
        robot_state_.copyJointGroupPositions(whole_body_jmg_, whole_body_joint_values_);
        robot_state_.update();

        if(verbose_)
        {
          robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
          geometry_msgs::PolygonStamped foot_polygon = getSupportPolygon();
          visualization_msgs::Marker com_marker = getCOMMarker();
          visualization_msgs::Marker p_com_marker = getPorjectedCOMMarker();

          robot_state_publisher_.publish(robot_state_msg_);
          support_polygon_pub_.publish(foot_polygon);
          pcom_pub_.publish(p_com_marker);
          com_pub_.publish(com_marker);

          ROS_INFO_STREAM("State is collision free and double support phase");

          ros::Duration(5.0).sleep();
        }

        if (write_pose)
        {
          writePose();
        }

        ssc_count++;
        ROS_INFO_STREAM("Current stable config: " << ssc_count);

      }
      else
      {
        if(verbose_)
        {
          ROS_INFO_STREAM("State is in collision, Not a feasible configuration");
        }
      }
    }

    if (ssc_count == config_count)
      break;

    if (loop_count%100 == 0)
    {
      ROS_INFO_STREAM("Current loop count: " << loop_count);
    }

    loop_count++;
  }

  ros::Time end_time = ros::Time::now();
  ros::Duration dura = end_time - start_time;

  num_config_ = ssc_count;
  ds_database_.close();

  ROS_INFO_STREAM("generate " << ssc_count << " in total loops " << loop_count);
  ROS_INFO_STREAM("cost " << dura << " seconds to finish whole process");
  ROS_INFO_STREAM("received config count " << config_count);

  return true;
}

bool StableConfigGenerator::sampleSSConfig(const int& max_samples, const std::string& file_destination, bool write_pose)
{
  std::ofstream ss_database;
  ss_database.open(file_destination.c_str(), std::ofstream::out|std::ios_base::app);
  if (!ss_database)
  {
    ROS_ERROR("unable to open single support database");
    return false;
  }

  robot_state::RobotState rstate(ps_->getRobotModel());
  rstate = ps_->getCurrentStateNonConst();
  rstate.setToDefaultValues();
  moveit_msgs::DisplayRobotState rstate_msg;

  // reset support mode to single
  if (!initSupportPolygon(0.8))
  {
    ROS_ERROR_STREAM("Initial support polygon fail");
    return false;
  }

  ros::Time start_time = ros::Time::now();
  int config_cnt = 0, loop_cnt = 0;
  while (config_cnt != max_samples)
  {
    whole_body_jmg_->getVariableRandomPositions(rng_, whole_body_joint_values_);
    wb_jnt_pos_map_.clear();

    for (int i = 0; i < whole_body_joint_names_.size(); i++)
    {
      wb_jnt_pos_map_.insert(std::pair<std::string, double>(whole_body_joint_names_[i], whole_body_joint_values_[i]));
    }

    rstate.setVariablePositions(wb_jnt_pos_map_);
    rstate.update();

    if (isFeasible(rstate))
    {
      rstate.copyJointGroupPositions(whole_body_jmg_, whole_body_joint_values_);
      if(verbose_)
      {
        robot_state::robotStateToRobotStateMsg(rstate, rstate_msg.state);
        visualization_msgs::Marker com_marker = getCOMMarker();
        visualization_msgs::Marker p_com_marker = getPorjectedCOMMarker();
        geometry_msgs::PolygonStamped foot_polygon = getSupportPolygon();
        support_polygon_pub_.publish(foot_polygon);
        robot_state_publisher_.publish(rstate_msg);
        pcom_pub_.publish(p_com_marker);
        com_pub_.publish(com_marker);

        ros::Duration(5.0).sleep();
      }

      if (write_pose)
      {
        for (std::size_t i = 0; i < whole_body_joint_values_.size(); i++)
        {
          ss_database << whole_body_joint_values_[i];
          ss_database << " ";
        }
        ss_database << "\n";
      }
      config_cnt++;
    }

    loop_cnt++;
    if ((loop_cnt%10000) == 0)
      ROS_INFO_STREAM("current loop count: " << loop_cnt << " config count: " << config_cnt);
  }
  ss_database.close();

  ros::Time end_time = ros::Time::now();
  ros::Duration dura = end_time - start_time;

  ROS_INFO_STREAM("cost " << dura << " seconds to finish whole process");

  return true;
}

bool StableConfigGenerator::computeRobotCoM(const robot_state::RobotState& state)
{
//  robot_state::RobotState robot_state_(ps_->getRobotModel());

//  robot_state_ = ps_->getCurrentStateNonConst();
//  robot_state_.setToDefaultValues();

//  whole_body_jmg_->getVariableRandomPositions(rng_, whole_body_joint_values_);

  std::map<std::string, double> joint_positions_map;
  state.copyJointGroupPositions(whole_body_jmg_, whole_body_joint_values_);
  for (int i = 0; i < whole_body_joint_names_.size(); i++)
  {
    joint_positions_map.insert(std::pair<std::string, double>(whole_body_joint_names_[i], whole_body_joint_values_[i]));
  }

//  robot_state_.setVariablePositions(joint_positions_map);

  //KDL::Vector com;
  KDL::Frame ident = KDL::Frame::Identity();

  computeCOMRecurs(kdl_tree_.getRootSegment(), joint_positions_map, ident);

  if (totoal_mass_ < 0.0)
  {
    ROS_WARN("Total mass less than 0");
    com_.setValue(0.0, 0.0, 0.0);
  }

  com = com * 1.0/totoal_mass_;
  com_.setValue(com.x(), com.y(), com.z());

  p_com_ = com_;
  p_com_.setZ(0.0);

  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(state, robot_state_msg_.state);
  robot_state_publisher_.publish(robot_state_msg_);

  geometry_msgs::PolygonStamped foot_polygon = getSupportPolygon();
  visualization_msgs::Marker com_marker = getCOMMarker();
  visualization_msgs::Marker p_com_marker = getPorjectedCOMMarker();

  support_polygon_pub_.publish(foot_polygon);
  pcom_pub_.publish(p_com_marker);
  com_pub_.publish(com_marker);

  return true;

}

bool StableConfigGenerator::generateConfig()
{
  robot_state::RobotState robot_state_ = getRandConfig();

  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);

  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  robot_state_publisher_.publish(robot_state_msg_);

  bool generate_config = ds_constraint_kin_.enforceDSLeftLeg(robot_state_, l_leg_jmg_);
  if(!generate_config)
  {
    ROS_WARN("stable_config_generator: unable to enforce double support");
    return generate_config;
  }

  generate_config = isFeasible(robot_state_);
  if (!generate_config)
  {
    ROS_WARN("stable_config_generator: random config is unfeasible");
    return generate_config;
  }

  robot_state_.copyJointGroupPositions(whole_body_jmg_, whole_body_joint_values_);
  robot_state_.update();

  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  geometry_msgs::PolygonStamped foot_polygon = getSupportPolygon();
  visualization_msgs::Marker com_marker = getCOMMarker();
  visualization_msgs::Marker p_com_marker = getPorjectedCOMMarker();

  goal_state_publisher_.publish(robot_state_msg_);
  support_polygon_pub_.publish(foot_polygon);
  pcom_pub_.publish(p_com_marker);
  com_pub_.publish(com_marker);

  return generate_config;
}



robot_state::RobotState StableConfigGenerator::getRandConfig()
{
  robot_state::RobotState robot_state_(ps_->getRobotModel());
  robot_state_.setToDefaultValues();

  std::vector<double> rand_config_value;
  whole_body_jmg_->getVariableRandomPositions(rng_, rand_config_value);
  wb_jnt_pos_map_.clear();

  for (int i = 0; i < whole_body_joint_names_.size(); i++)
  {
    wb_jnt_pos_map_.insert(std::pair<std::string, double>(whole_body_joint_names_[i], rand_config_value[i]));
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

  return robot_state_;
}

bool StableConfigGenerator::isFeasible(const robot_state::RobotState& robot_state)
{
  bool statically_stable = false;
  bool collision_free = false;
  bool config_valid = false;

  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collision_res;
  collision_detection::AllowedCollisionMatrix acm = ps_->getAllowedCollisionMatrix();

  collision_req.group_name = group_name_;
  collision_res.clear();

  ps_->checkCollision(collision_req, collision_res, robot_state, acm);

  if(collision_res.collision == 1)
  {
    collision_free = false;
    return false;
  }
  else
  {
    collision_free = true;
  }

  computeCOM();

  if (isStaticallyStable(p_com_, support_polygon_))
  {
     statically_stable = true;
  }

  if (statically_stable == true && collision_free == true)
  {
    config_valid = true;
  }
  else
  {
    config_valid = false;
  }

  return config_valid;

}

bool StableConfigGenerator::initSupportPolygon(const double& sp_scale)
{
  std::string rfoot_mesh_link_name_ = "r_ankle";
  const boost::shared_ptr<const urdf::ModelInterface>& urdf_model_ = ps_->getRobotModel()->getURDF();
  boost::shared_ptr<const urdf::Link> foot_link =  urdf_model_->getLink(rfoot_mesh_link_name_);
  assert(foot_link);
  boost::shared_ptr<const urdf::Geometry> geom;
  urdf::Pose geom_pose;

  if (foot_link->collision && foot_link->collision->geometry)
  {
    geom = foot_link->collision->geometry;
    geom_pose = foot_link->collision->origin;
  }
  else if (foot_link->visual && foot_link->visual->geometry)
  {
    geom = foot_link->visual->geometry;
    geom_pose = foot_link->visual->origin;
  }
  else
  {
    ROS_ERROR_STREAM("No geometry for link "<< rfoot_mesh_link_name_ << " available");
    return false;
  }

  tf::Pose geom_origin = tf::Pose(tf::Transform(tf::Quaternion(geom_pose.rotation.x, geom_pose.rotation.y, geom_pose.rotation.z, geom_pose.rotation.w),
                                              tf::Vector3(geom_pose.position.x, geom_pose.position.y, geom_pose.position.z)));

  if (geom->type != urdf::Geometry::MESH)
  {
    ROS_ERROR_STREAM("Geometry for link "<< rfoot_mesh_link_name_ << " is not a mesh");
    return false;
  }

  boost::shared_ptr<const urdf::Mesh> mesh = boost::dynamic_pointer_cast<const urdf::Mesh>(geom);
  const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
  shapes::Mesh* shape_mesh = shapes::createMeshFromResource(mesh->filename, scale);
  size_t vertex_count = shape_mesh->vertex_count;

  std::vector<tf::Point> right_foot_support_pt;
  for (unsigned int i = 0; i < vertex_count; i++)
  {
    unsigned int i3 = i*3;

    tf::Point p(shape_mesh->vertices[i3], shape_mesh->vertices[i3+1], shape_mesh->vertices[i3+2]);
    tf::Point projectedP = geom_origin * p;
    projectedP.setZ(0.0);

    right_foot_support_pt.push_back(projectedP);
  }

  float sum_x_coord = 0.0;
  float sum_y_coord = 0.0;
  tf::Point r_foot_center;
  for (unsigned int i = 0 ; i < right_foot_support_pt.size(); ++i)
  {
    sum_x_coord = sum_x_coord + right_foot_support_pt[i].x();
    sum_y_coord = sum_y_coord + right_foot_support_pt[i].y();
  }

  //X and Y of right foot center
  r_foot_center.setX(sum_x_coord/right_foot_support_pt.size());
  r_foot_center.setY(sum_y_coord/right_foot_support_pt.size());

  //Vector storing foot points w.r.t foot center
  std::vector<tf::Point> foot_SP_right_center;
  tf::Point foot_point;
  for (unsigned int i = 0 ; i < right_foot_support_pt.size(); ++i)
  {
    //Express point w.r.t foot center and directly apply scaling
    foot_point.setX( (right_foot_support_pt[i].x() - r_foot_center.x()) * sp_scale );
    foot_point.setY( (right_foot_support_pt[i].y() - r_foot_center.y()) * sp_scale );
    foot_SP_right_center.push_back(foot_point);
  }

  //Express new(scaled) coordinates in local frame
  std::vector<tf::Point> scaled_SP_right;
  for (unsigned int i = 0 ; i < foot_SP_right_center.size() ; ++i)
  {
    //Express point w.r.t foot center and directly apply scaling
    foot_point.setX( foot_SP_right_center[i].x() + r_foot_center.x() ) ;
    foot_point.setY( foot_SP_right_center[i].y() + r_foot_center.y() ) ;
    scaled_SP_right.push_back(foot_point);
  }

  foot_support_polygon_right_ = convexHull(scaled_SP_right);

  support_polygon_.clear();
  for (unsigned int i = 0 ; i < foot_support_polygon_right_.size() ; ++i)
  {
    support_polygon_.push_back(foot_support_polygon_right_[i]);
  }

  if (support_mode_ == DOUBLE_SUPPORT)
  {
    foot_support_polygon_left_ = foot_support_polygon_right_;
    for (unsigned i = 0; i < foot_support_polygon_left_.size(); i++)
    {
  //    foot_support_polygon_left_[i] *= tf::Point(1.0, -1.0, 1.0);
      foot_support_polygon_left_[i] += tf::Point(0.0, 0.2477, 0.0);
    }

    // restore order of polygon
    foot_support_polygon_left_ = convexHull(foot_support_polygon_left_);

    for (unsigned int i = 0 ; i < foot_support_polygon_left_.size() ; ++i)
    {
      support_polygon_.push_back(foot_support_polygon_left_[i]);
    }
  }

  ROS_INFO_STREAM("current support mode:" << support_mode_);

  support_polygon_ = convexHull(support_polygon_);

  if(verbose_)
  {
    ROS_INFO("End of initial support polygon");
  }

  return true;
}

bool StableConfigGenerator::isStaticallyStable(const tf::Point &point, const std::vector<tf::Point> &polygon)
{
  int positive_direction = 0;
  for (unsigned i = 0; i < polygon.size(); i++)
  {
    int i2 = (i+1)% (polygon.size());
    double dx = polygon[i2].getX() - polygon[i].getX();
    double dy = polygon[i2].getY() - polygon[i].getY();

    if (dx == 0.0 && dy == 0.0)
    {
      ROS_DEBUG("Skipping polygon connection [%d-%d] (identical points)", i, i2);
      continue;
    }

    double line_test = (point.y() - polygon[i].getY())*dx - (point.x() - polygon[i].getX())*dy;

    if (i == 0)
    {
      positive_direction = (line_test > 0.0);
    }

    ROS_DEBUG("Line test [%d-%d] from (%f,%f) to (%f,%f): %f", i, i2, polygon[i].getX(), polygon[i].getY(),
              polygon[i2].getX(), polygon[i2].getY(), line_test);

    if ((line_test > 0.0) != positive_direction)
    {
      return false;
    }
  }

  return true;
}

void StableConfigGenerator::computeCOM()
{
  KDL::Frame ident = KDL::Frame::Identity();

  computeCOMRecurs(kdl_tree_.getRootSegment(), wb_jnt_pos_map_, ident);

  if (totoal_mass_ < 0.0)
  {
    ROS_WARN("Total mass less than 0");
    com_.setValue(0.0, 0.0, 0.0);
  }

  com = com * 1.0/totoal_mass_;
  com_.setValue(com.x(), com.y(), com.z());

  p_com_ = com_;
  p_com_.setZ(0.0);
  totoal_mass_ = 0.0;
}

void StableConfigGenerator::computeCOMRecurs(const KDL::SegmentMap::const_iterator &current_seg, const std::map<std::string, double>& joint_positions, const KDL::Frame& prev_frame)
{
  double jnt_p = 0.0;

  if (current_seg->second.segment.getJoint().getType() != KDL::Joint::None)
  {
    std::map<std::string, double>::const_iterator jnt = joint_positions.find(current_seg->second.segment.getJoint().getName());

    if (jnt == joint_positions.end())
    {
      ROS_WARN("Could not find joint %s of %s in joint positions. Aborting tree branch.", current_seg->second.segment.getJoint().getName().c_str(), current_seg->first.c_str());
      return;
    }
    jnt_p = jnt->second;
  }

  KDL::Frame current_frame = prev_frame * current_seg->second.segment.pose(jnt_p);

  double mass_current = current_seg->second.segment.getInertia().getMass();
  KDL::Vector COM_current = current_seg->second.segment.getInertia().getCOG();

  com = com + mass_current * (current_frame * COM_current);
  totoal_mass_ += mass_current;

//  ROS_INFO_STREAM("At link: " << current_seg->first.c_str() << " mass: " << mass_current << "\n global frame: "
//                  << com.x() << " " << com.y() << " " << com.z() <<
//                  "\n local frame: " << COM_current.x() << " " << COM_current.y() << " " << COM_current.z());

  std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
  for (child_it = current_seg->second.children.begin(); child_it != current_seg->second.children.end(); ++child_it)
  {
    computeCOMRecurs(*child_it, joint_positions, current_frame);
  }

}

visualization_msgs::Marker StableConfigGenerator::getCOMMarker() const
{
  visualization_msgs::Marker COM_marker;

  COM_marker.header.stamp = ros::Time::now();
  COM_marker.header.frame_id = root_link_name_;
  COM_marker.action = visualization_msgs::Marker::ADD;
  COM_marker.type = visualization_msgs::Marker::SPHERE;

  tf::pointTFToMsg(com_, COM_marker.pose.position);
//  tf::quaternionTFToMsg(tf_to_support_.getRotation(), COM_marker.pose.orientation);

  COM_marker.scale.x = 0.05;
  COM_marker.scale.y = 0.05;
  COM_marker.scale.z = 0.05;
  COM_marker.color.a = 1.0;
  COM_marker.color.r = 1.0;

  return COM_marker;
}

visualization_msgs::Marker StableConfigGenerator::getPorjectedCOMMarker() const
{
  visualization_msgs::Marker p_COM_marker;

  p_COM_marker.header.stamp = ros::Time::now();
  p_COM_marker.header.frame_id = root_link_name_;
  p_COM_marker.action = visualization_msgs::Marker::ADD;
  p_COM_marker.type = visualization_msgs::Marker::SPHERE;

  tf::pointTFToMsg(p_com_, p_COM_marker.pose.position);
//  tf::quaternionTFToMsg(tf_to_support_.getRotation(), COM_marker.pose.orientation);

  p_COM_marker.scale.x = 0.05;
  p_COM_marker.scale.y = 0.05;
  p_COM_marker.scale.z = 0.005;
  p_COM_marker.color.a = 1.0;
  p_COM_marker.color.g = 1.0;
  p_COM_marker.color.r = 0.0;

  return p_COM_marker;
}

geometry_msgs::PolygonStamped StableConfigGenerator::getSupportPolygon()
{
  geometry_msgs::PolygonStamped foot_print_polygon;
  foot_print_polygon.header.frame_id = "r_sole";
  foot_print_polygon.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i < support_polygon_.size(); i++)
  {
    geometry_msgs::Point32 p;
    tf::Point tfP = support_polygon_[i];
    p.x = tfP.x();
    p.y = tfP.y();
    p.z = tfP.z();

    foot_print_polygon.polygon.points.push_back(p);
  }

  return foot_print_polygon;
}

std::vector<tf::Point> StableConfigGenerator::convexHull(const std::vector<tf::Point>& input_pts) const
{
  std::vector<tf::Point> hull;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> chull_points;
  pcl::ConvexHull<pcl::PointXYZ> chull;

  if (input_pts.empty())
  {
    ROS_ERROR("convexHull on empty set of points!");
    return hull;
  }

  for (unsigned i = 0; i < input_pts.size(); i++)
  {
    pcl_points->points.push_back(pcl::PointXYZ(input_pts[i].x(), input_pts[i].y(), 0.0));
  }

  chull.setDimension(2);
  chull.setInputCloud(pcl_points);
  std::vector<pcl::Vertices> polygons;
  chull.reconstruct(chull_points, polygons);

  if (polygons.size() == 0)
  {
    ROS_ERROR("Convex hull polygons are empty");
    return hull;
  }

  for (unsigned int i = 0; i < polygons[0].vertices.size(); i++)
  {
    int idx = polygons[0].vertices[i];
    tf::Point p(chull_points.points[idx].x,
                chull_points.points[idx].y,
                chull_points.points[idx].z);
    hull.push_back(p);
  }

  return hull;
}

bool StableConfigGenerator::test()
{
  if (renbo_urdf_ == NULL)
  {
    renbo_urdf_ = ps_->getRobotModel()->getURDF();
  }

  robot_state::RobotState robot_state_(ps_->getRobotModel());

  robot_state_ = ps_->getCurrentStateNonConst();
  robot_state_.setToDefaultValues();

  whole_body_jmg_->getVariableRandomPositions(rng_, whole_body_joint_values_);

  std::map<std::string, double> joint_positions_map;
  //robot_state_.copyJointGroupPositions(whole_body_jmg_, whole_body_joint_values_);
  for (int i = 0; i < whole_body_joint_names_.size(); i++)
  {
    joint_positions_map.insert(std::pair<std::string, double>(whole_body_joint_names_[i], whole_body_joint_values_[i]));
  }

  robot_state_.setVariablePositions(joint_positions_map);

  //KDL::Vector com;
  KDL::Frame ident = KDL::Frame::Identity();

  computeCOMRecurs(kdl_tree_.getRootSegment(), joint_positions_map, ident);

  if (totoal_mass_ < 0.0)
  {
    ROS_WARN("Total mass less than 0");
    com_.setValue(0.0, 0.0, 0.0);
  }

  com = com * 1.0/totoal_mass_;
  com_.setValue(com.x(), com.y(), com.z());

  p_com_ = com_;
  p_com_.setZ(0.0);

  moveit_msgs::DisplayRobotState robot_state_msg_;
  robot_state::robotStateToRobotStateMsg(robot_state_, robot_state_msg_.state);
  robot_state_publisher_.publish(robot_state_msg_);

  geometry_msgs::PolygonStamped foot_polygon = getSupportPolygon();
  visualization_msgs::Marker com_marker = getCOMMarker();
  visualization_msgs::Marker p_com_marker = getPorjectedCOMMarker();

  support_polygon_pub_.publish(foot_polygon);
  pcom_pub_.publish(p_com_marker);
  com_pub_.publish(com_marker);

  return true;
}

void StableConfigGenerator::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = segment->second.segment.getName();
  const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;

  for (unsigned int i=0; i < children.size(); i++)
  {
    const KDL::Segment& child = children[i]->second.segment;
    robot_state_publisher::SegmentPair s(children[i]->second.segment, root, child.getName());

    if (child.getJoint().getType() == KDL::Joint::None)
    {
      // skip over fixed:
      //      segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Tree initialization: Skipping fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else
    {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Tree initialization: Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

void StableConfigGenerator::writePose()
{
  for (std::size_t i = 0; i < whole_body_joint_values_.size(); i++)
  {
      ds_database_ << whole_body_joint_values_[i];
      ds_database_ << " ";
  }
  ds_database_ << "\n";
}

int StableConfigGenerator::getNumConfig()
{
  return num_config_;
}

void StableConfigGenerator::setSupportMode(FootSupport support_mode)
{
  support_mode_ = support_mode;
}
void StableConfigGenerator::setVerbose(bool verbose)
{
  verbose_ = verbose;
}


}

#include <renbo_whole_body_plan/stability_checker.h>

namespace renbo_planner
{

StabilityChecker::StabilityChecker(const int& support_mode, const double& scale_sp):
  support_mode_(support_mode)
{
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

  renbo_urdf_ = robot_model_loader_->getURDF();

  wb_jnt_names_ = robot_model_loader_->getModel()->getJointModelGroup("whole_body_fixed")->getJointModelNames();

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

StabilityChecker::~StabilityChecker(){}

bool StabilityChecker::isStable(const robot_state::RobotState rstate)
{
  std::vector<double> config;
  rstate.copyJointGroupPositions("whole_body_fixed", config);
  std::map<std::string, double> wb_jp_map;
  for (int i = 0; i < wb_jnt_names_.size(); i++)
  {
    wb_jp_map.insert(std::pair<std::string, double>(wb_jnt_names_[i], config[i]));
  }

  computeCOM(wb_jp_map);
  if (com_.getX() < 0)
  {
    ROS_ERROR("Invalid CoM position");
    return false;
  }

  bool is_stable = false;
  if (isStaticallyStable(p_com_, support_polygon_))
  {
     is_stable = true;
  }

  com_.setValue(0, 0, 0);
  p_com_.setValue(0, 0, 0);
  total_mass_ = 0.0;

  return is_stable;
}

bool StabilityChecker::isStaticallyStable(const tf::Point &point, const std::vector<tf::Point> &polygon)
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

void StabilityChecker::computeCOM(std::map<std::string, double> config)
{
  KDL::Frame ident = KDL::Frame::Identity();

  computeCOMRecurs(kdl_tree_.getRootSegment(), config, ident);

  if (total_mass_ < 0.0)
  {
    ROS_WARN("Total mass less than 0");
    com_.setValue(-1, -1, -1);
  }

  com = com * 1.0/total_mass_;
  com_.setValue(com.x(), com.y(), com.z());

  p_com_ = com_;
  p_com_.setZ(0.0);
}

void StabilityChecker::computeCOMRecurs(const KDL::SegmentMap::const_iterator &current_seg, const std::map<std::string, double> &joint_positions, const KDL::Frame& prev_frame)
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
  total_mass_ += mass_current;

//  ROS_INFO_STREAM("At link: " << current_seg->first.c_str() << " mass: " << mass_current << "\n global frame: "
//                  << com.x() << " " << com.y() << " " << com.z() <<
//                  "\n local frame: " << COM_current.x() << " " << COM_current.y() << " " << COM_current.z());

  std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
  for (child_it = current_seg->second.children.begin(); child_it != current_seg->second.children.end(); ++child_it)
  {
    computeCOMRecurs(*child_it, joint_positions, current_frame);
  }
}

void StabilityChecker::addChildren(const KDL::SegmentMap::const_iterator segment)
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

bool StabilityChecker::initSupportPolygon(const double& sp_scale)
{
  std::string rfoot_mesh_link_name_ = "r_ankle";
//  boost::shared_ptr<const urdf::Link> foot_link =  renbo_urdf_->getLink(rfoot_mesh_link_name_);
  const boost::shared_ptr<const urdf::ModelInterface>& urdf_model_ = robot_model_loader_->getURDF();
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
//    ROS_INFO_STREAM(foot_point.getX() << " " << foot_point.getY() << " " << foot_point.getZ());
  }

  foot_support_polygon_right_ = convexHull(scaled_SP_right);

  support_polygon_.clear();
  for (unsigned int i = 0 ; i < foot_support_polygon_right_.size() ; ++i)
  {
    support_polygon_.push_back(foot_support_polygon_right_[i]);
  }

  if (support_mode_ == 0)
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
  if (support_polygon_.empty())
  {
    return false;
  }

  return true;
}

std::vector<tf::Point> StabilityChecker::convexHull(const std::vector<tf::Point>& input_pts) const
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

}

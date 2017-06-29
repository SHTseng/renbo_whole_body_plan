#ifndef STABLE_CONFIG_GENERATOR_H_
#define STABLE_CONFIG_GENERATOR_H_

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <hrl_kinematics/TestStability.h>
#include <hrl_kinematics/Kinematics.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>

#include <robot_state_publisher/robot_state_publisher.h>

#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/PolygonStamped.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>

#include <renbo_whole_body_plan/double_support_constraint.h>

namespace renbo_constraint_sampler
{

class StableConfigGenerator
{
public:

  enum FootSupport {DOUBLE_SUPPORT, SINGLE_SUPPORT_RIGHT, SINGLE_SUPPORT_LEFT};

  StableConfigGenerator(const std::string &group_name, const double& scale_sp);

  virtual ~StableConfigGenerator();

  /*
   * sample valid double support configuration
  */
  bool sampleDSConfig(int config_count, std::string file_destination, bool write_pose);

  /*
   * sample valid single support configuration
  */
  bool sampleSSConfig(const int& max_samples, const std::string& file_destination, bool write_pose);

  bool computeRobotCoM(const robot_state::RobotState& state);

  int getNumConfig();

  void setSupportMode(FootSupport support_mode);

  void setVerbose(bool verbose);

  bool test();

  bool generateConfig();


private:

  void initialize(double scale_sp);

  robot_state::RobotState getRandConfig();

  bool isFeasible(const moveit::core::RobotState &robot_state);

  bool isStaticallyStable(const tf::Point &point, const std::vector<tf::Point> &polygon);

  bool initSupportPolygon(const double& sp_scale);

  std::vector<tf::Point> convexHull(const std::vector<tf::Point>& input_pts) const;

  void computeCOM();

  void computeCOMRecurs(const KDL::SegmentMap::const_iterator &current_seg, const std::map<std::string, double> &joint_positions, const KDL::Frame& prev_frame);

  void addChildren(const KDL::SegmentMap::const_iterator segment);

  void writePose();

  geometry_msgs::PolygonStamped getSupportPolygon();

  visualization_msgs::Marker getCOMMarker() const;

  visualization_msgs::Marker getPorjectedCOMMarker() const;

  ros::NodeHandle nh_;

  ros::Publisher visualization_pub_, support_polygon_pub_, com_pub_, pcom_pub_;

  ros::Publisher robot_state_publisher_;

  ros::Publisher goal_state_publisher_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  robot_model::RobotModelPtr robot_model_;

  const robot_state::JointModelGroup* whole_body_jmg_;
  const robot_state::JointModelGroup* l_leg_jmg_;

  std::string root_link_name_;

  std::map<std::string, double> whole_body_joint_positions_map_;
  std::vector<double> whole_body_joint_values_;
  std::vector<std::string> whole_body_joint_names_;

  std::map<std::string, double> wb_jnt_pos_map_;

  std::string group_name_;

  planning_scene::PlanningScenePtr ps_;

  std::vector<tf::Point> foot_support_polygon_right_;
  std::vector<tf::Point> foot_support_polygon_left_;
  std::vector<tf::Point> support_polygon_;

  boost::shared_ptr<const urdf::ModelInterface> renbo_urdf_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_right_;
  KDL::Chain kdl_chain_left_;

  std::map<std::string, robot_state_publisher::SegmentPair> segments_;

  KDL::Vector com;
  tf::Point com_;
  tf::Point p_com_;
  tf::Transform tf_to_support_;
  double totoal_mass_  ;

  FootSupport support_mode_;

  DoubleSupportConstraint ds_constraint_kin_;

  random_numbers::RandomNumberGenerator rng_;

  std::ofstream ds_database_;

  int num_config_;

  bool verbose_;

};

}


#endif

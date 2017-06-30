#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

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

#include <memory>


namespace renbo_planner
{

class StabilityChecker
{
public:

  StabilityChecker(const int& support_mode, const double& scale_sp);

  ~StabilityChecker();

  bool isStable(const robot_state::RobotState rstate);

private:

  void initialize(double scale_sp);

  bool isStaticallyStable(const tf::Point &point, const std::vector<tf::Point> &polygon);

  bool initSupportPolygon(const double& sp_scale);

  void addChildren(const KDL::SegmentMap::const_iterator segment);

  std::vector<tf::Point> convexHull(const std::vector<tf::Point>& input_pts) const;

  void computeCOM(std::map<std::string, double> config);

  void computeCOMRecurs(const KDL::SegmentMap::const_iterator &current_seg, const std::map<std::string, double> &joint_positions, const KDL::Frame& prev_frame);

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  std::vector<std::string> wb_jnt_names_;

  int support_mode_;

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

  double total_mass_ ;
};
}

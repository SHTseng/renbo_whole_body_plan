#ifndef RRT_CONNECT_PLANNER_
#define RRT_CONNECT_PLANNER_

#include <ros/ros.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/macros/console_colors.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <renbo_whole_body_plan/double_support_constraint.h>

#include <hrl_kinematics/TestStability.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <fstream>
#include <sstream>
#include <string>

namespace renbo_planner
{

struct node
{
  int index;
  std::vector<double> config;
  int predecessor_index;
};

struct tree
{
  int id;
  std::vector<node> nodes;
  int num_nodes;
  bool is_complete;
};

enum status{ADVANCED, REACHED, TRAPPED};

typedef std::vector< std::vector<double> > trajectory;


class RRTConnectPlanner
{
public:

  RRTConnectPlanner(std::string group_name, std::string database_pth, bool write_file);

  virtual ~RRTConnectPlanner();

  bool setStartGoalConfigs(std::vector<double> start_config , std::vector<double> goal_config);

  moveit_msgs::DisplayTrajectory solveQuery(int max_iter, double max_step_size);

  void updateEnvironment(const planning_scene::PlanningScenePtr &scene);

  void setAttachCollsionObject(const moveit_msgs::AttachedCollisionObject& attach_object);

  void setVisualizationSwtich(bool on_off);

  void setVerbose(bool verbose);

  void getResult();

  bool TEST(int test_flag);

  bool is_grasped = false;

protected:

  bool loadDSDatabase(std::string path);

  // Add a random valid config to the node
  node getRandomStableConfig();

  //Searches for the nearest neighbor of q_rand according to some distance metric
  node findNearestNeighbour(tree input_tree, node q_rand);

  //Generate a q_new along the line connecting q_near to q_rand
  status generate_q_new (node q_near, node q_rand, node &q_new);

  //extend the tree (finds a valid q_new and adds it to the tree)
  status extendTree(tree &input_tree, node q_rand, node &q_near, moveit::core::RobotState &state);

  //connect the trees (tries to connect one tree to the other)
  status connectTree(tree &input_tree, node q_connect, node &q_near, moveit::core::RobotState &state);

  //Add a configuration to the tree
  void addConfigtoTree(tree &input_tree, node q_near, node q_new_modified);

  void swapTree(tree& tree_a, tree& tree_b);

  bool checkCollision(const std::vector<double> config);

  bool checkCollision(const moveit::core::RobotState& state);

  //Write the solution path configurations into a file
  void writePath(tree T_start, tree T_goal, node q_near,
                 int connector, moveit_msgs::DisplayTrajectory&disp_trajectory_msgs,
                 std::string solution_file_path);

  //Interpolate two waypoints of the raw solution path
  moveit_msgs::DisplayTrajectory interpolateWaypoints(std::vector<double> waypoint_start,
                                                      std::vector<double> waypoint_goal,
                                                      int num_intermediate_waypoints);

  //Interpolate the waypoints of the shortcutted path
  void interpolateShortPathWaypoints(std::vector<std::vector<double> > short_path, std::vector<std::vector<double> > &interpolated_path, int num_intermediate_waypoints);

  void linearInterpolation(trajectory short_path, trajectory &interpolated_path, int num_intermediate_waypoints);

  /*
   * Path Shortcutter:
   * Given the raw solution path tries to reduce the number of waypoints/configurations while maintaining validity of the path
   */
  bool pathShortCutter(trajectory raw_path, trajectory &shortcutted_path, int num_intermediate_waypoints);

  //Generate a Trajectory from the array "path"
  void generateTrajectory(std::vector< std::vector<double> > path, int num_configs, moveit_msgs::DisplayTrajectory &disp_traj);

  double computePathLength();

  void resetTrees();

  void writeLogFile(const double& duration, const int& node_num, const int& interation_time);

  bool timeOut(ros::Time current_time, float time_out);

  void PAUSE();

  ros::NodeHandle nh_;

  ros::Publisher robot_state_publisher_;

  ros::Publisher scene_publisher_;

  ros::Publisher trajectory_publisher_;

  planning_scene::PlanningScenePtr ps_;

  std::string base_frame_;

  std::string group_name_;

  std::string eef_name_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  const robot_state::JointModelGroup* wb_jmg_;

  const robot_state::JointModelGroup* rarm_torso_jmg_;

  const robot_state::JointModelGroup* l_leg_jmg_;

  std::vector<std::string> wb_joint_names_;

  std::vector<std::string> wb_link_names_;

  int wb_num_joint_;

  std::map<std::string, double> wb_jnt_pos_map_;

  std::string database_path_;

  std::vector< std::vector<double> > ds_database_configs_;

  int ds_database_config_count_;

  std::string solution_file_path_;

  tree tree_start_;

  tree tree_goal_;

  double step_size_;

  trajectory solution_path_configs_;

  moveit_msgs::AttachedCollisionObject attached_collision_object_;

  renbo_constraint_sampler::DoubleSupportConstraint ds_constraint_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

  random_numbers::RandomNumberGenerator rng_;

  bool write_file_;

  bool visualize_path_;

  bool verbose_;

};

class MultiGoalRRTPlanner: public RRTConnectPlanner
{

public:

  MultiGoalRRTPlanner(std::string group_name, std::string database_pth, bool write_file);

  virtual ~MultiGoalRRTPlanner() = default;

  bool setStartGoalConfigs(std::vector<double> start_config , std::vector< std::vector<double> > goal_configs);

  std::vector<moveit_msgs::DisplayTrajectory> solve(double time_out, double step_size);

private:

  double computePathLength(trajectory path);

  void swapTrees(std::vector<tree> tree_a, std::vector<tree> tree_b);

  void resetMultiTrees();

  std::vector<tree> goal_trees_;

};

}

#endif

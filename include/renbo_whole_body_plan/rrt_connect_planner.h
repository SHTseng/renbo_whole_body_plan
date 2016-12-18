#ifndef RRT_CONNECT_PLANNER_
#define RRT_CONNECT_PLANNER_

#include <ros/ros.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/macros/console_colors.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <renbo_whole_body_plan/stable_config_generator.h>
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
  std::string name;
  std::vector<node> nodes;
  int num_nodes;
};

enum status{ADVANCED, REACHED, TRAPPED};

typedef std::vector< std::vector<double> > Trajectory;


class RRTConnectPlanner
{
public:

  RRTConnectPlanner(std::string group_name, std::__cxx11::string database_pth, std::string solution_path);

  ~RRTConnectPlanner();

  bool setStartGoalConfigs(std::vector<double> start_config , std::vector<double> goal_config);

  moveit_msgs::DisplayTrajectory solveQuery(int max_iter, double max_step_size);

  void updateEnvironment(const planning_scene::PlanningScenePtr &scene);

  void setDatabasePath(const std::string& path);

  void setVisualizationSwtich(bool on_off);

  void setVerbose(bool verbose);

  bool TEST(int test_flag);

private:

  bool loadDSDatabase(std::string path);


  // Add a random valid config to the node
  void getRandomStableConfig(node &sample);


  /*
   * RRT related functions
   */

  //Searches for the nearest neighbor of q_rand according to some distance metric
  void findNearestNeighbour(tree T_current, node q_rand, node &nearest_neighbor);

  //Generate a q_new along the line connecting q_near to q_rand
  status generate_q_new (node q_near, node q_rand, node &q_new, double step_size);

  //extend the tree (finds a valid q_new and adds it to the tree)
  status extendTree(tree &input_tree, node q_rand, node &q_near, double max_step_size);

  //connect the trees (tries to connect one tree to the other)
  status connectTree(tree &input_tree, node q_connect, node &q_near, double max_step_size);

  //Add a configuration to the tree
  void addConfigtoTree(tree &input_tree, node q_near, node q_new_modified);


//  //Modify q_new such that the returned node obeys the constraints (Joint limits + double support)
//  status enforce_DS_Constraints(node q_new, node &q_new_modified);

//  //Modify q_new_modified such that the returned node obeys the manipulation constraints (hand moves along line or circle)
//  status enforce_MA_Constraints(std::string tree_name, node q_near, node q_new_ds, node &q_new_manip);

  /*
   * Check whether the robot configuration is valid or not, check collision and statically stable.
  */

  bool checkCollision(planning_scene::PlanningScenePtr ps_, const std::vector<double> config);

  bool checkCollision(moveit::core::RobotState robot_state);


  //Write the solution path configurations into a file
  void writePath(tree T_start, tree T_goal, node q_near, int connector, moveit_msgs::DisplayTrajectory &trajectory, std::string solution_file_path);

  //Interpolate two waypoints of the raw solution path
  moveit_msgs::DisplayTrajectory interpolateWaypoints(std::vector<double> waypoint_start,std::vector<double> waypoint_goal, int num_intermediate_waypoints);

  //Interpolate the waypoints of the shortcutted path
  void interpolateShortPathWaypoints(std::vector<std::vector<double> > short_path, std::vector<std::vector<double> > &interpolated_path, int num_intermediate_waypoints);

  void linearInterpolation(Trajectory short_path, Trajectory &interpolated_path, int num_intermediate_waypoints);

  /*
   * Path Shortcutter
    Given the raw solution path tries to reduce the number of waypoints/configurations while maintaining validity of the path
   *
   */
  bool pathShortCutter(Trajectory raw_path, Trajectory &shortcutted_path, int num_intermediate_waypoints);

  //Generate a Trajectory from the array "path"
  void generateTrajectory(std::vector< std::vector<double> > path, int num_configs, moveit_msgs::DisplayTrajectory &disp_traj);

  void resetTrees();

  void writeLogFile(const double& duration, const int& node_num, const int& interation_time);

  void PAUSE();

  ros::NodeHandle nh_;

  ros::Publisher robot_state_publisher_;

  ros::Publisher scene_publisher_;

  ros::Publisher trajectory_publisher_;

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  planning_scene::PlanningScenePtr ps_;

  moveit::planning_interface::PlanningSceneInterface pci_;

  std::string base_frame_;

  std::string group_name_;

  std::string eef_name_;

  const robot_state::JointModelGroup* wb_jmg_;

  const robot_state::JointModelGroup* l_leg_jmg_;

  std::vector<std::string> wb_joint_names_;

  std::vector<std::string> wb_link_names_;

  int wb_num_joint_;

  std::map<std::string, double> wb_jnt_pos_map_;

  std::ifstream ds_config_file_;

  std::string database_path_;

  std::vector< std::vector<double> > ds_database_configs_;

  int ds_database_config_count_;

  std::string solution_file_path_;

  tree T_start_;

  tree T_goal_;

  Trajectory solution_path_configs_;

  renbo_constraint_sampler::DoubleSupportConstraint ds_constraint_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

  random_numbers::RandomNumberGenerator rng_;

  bool visualize_path_;

  bool verbose_;

};

}

#endif

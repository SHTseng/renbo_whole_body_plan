#include <ros/ros.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <renbo_msgs/compute_motion_plan.h>
#include <renbo_msgs/generate_whole_body_posture.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "generate_grasping_posture");

  ros::NodeHandle nh_;

  ros::ServiceClient generate_posture = nh_.serviceClient<renbo_msgs::generate_whole_body_posture>("capability_map/generate_wb_posture");
  renbo_msgs::generate_whole_body_posture generate_posture_srv;

  //generate_posture_srv.request.scenario = SCENERIO;
  Eigen::Affine3d eigen_pose;
  eigen_pose = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())* Eigen::AngleAxisd(0.523, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ());
  eigen_pose.translation() << 0.43, -0.18, 0.8;

  geometry_msgs::Pose geo_pose;
  tf::poseEigenToMsg(eigen_pose, geo_pose);
  generate_posture_srv.request.right_eef_pose = geo_pose;

  std::vector<double> goal_config;
  if (generate_posture.call(generate_posture_srv))
  {
    ROS_INFO_STREAM("generate whole-body posture return state: " << (int)generate_posture_srv.response.success);
    goal_config = generate_posture_srv.response.solved_config;
  }
  else
  {
    ROS_INFO_STREAM("generate whole-body posture return state: " << (int)generate_posture_srv.response.success);
    exit(1);
  }

  ros::shutdown();

  return 0;

}

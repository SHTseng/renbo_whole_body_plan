#include <ros/ros.h>

#include <rrt_planner_msgs/Generate_DS_Configs.h>

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "compute_com");

  ros::NodeHandle nh_;

  ros::ServiceClient compute_com_client = nh_.serviceClient<rrt_planner_msgs::Generate_DS_Configs>("compute_robot_com");
  rrt_planner_msgs::Generate_DS_Configs compute_com_srv;

  if(!compute_com_client.call(compute_com_srv))
  {
    ROS_ERROR("Fail to generate statically stable data");
    exit(1);
  }

  return 0;
}

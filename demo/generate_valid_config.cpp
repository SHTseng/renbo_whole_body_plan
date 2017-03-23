#include <ros/ros.h>

#include <renbo_msgs/generate_ss_config.h>

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "generate_valid_config");

  ros::NodeHandle nh_;

  ros::ServiceClient generate_valid_config = nh_.serviceClient<renbo_msgs::generate_ss_config>("generate_valid_config");
  renbo_msgs::generate_ss_config generate_valid_config_srv;

  int success = 0;
  while(success == 0)
  {
    bool call = generate_valid_config.call(generate_valid_config_srv);
    success = (int)generate_valid_config_srv.response.result;
  }

  return 0;
}

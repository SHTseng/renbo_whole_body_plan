#include <ros/ros.h>
#include <ros/package.h>

#include <renbo_whole_body_plan/stable_config_generator.h>
#include <memory>

using namespace renbo_constraint_sampler;

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "generate_ss_database");
  ros::NodeHandle nh_("~");

  std::unique_ptr<StableConfigGenerator> scg;
  scg = std::unique_ptr<StableConfigGenerator>(new StableConfigGenerator("whole_body_fixed", 0.8));

  StableConfigGenerator::FootSupport sm =  StableConfigGenerator::FootSupport::SINGLE_SUPPORT_RIGHT;
  scg->setVerbose(false);
  scg->setSupportMode(sm);

  std::string file_path = ros::package::getPath("renbo_whole_body_plan");
  file_path.append("/database/ss_dataset.dat");

  if (!scg->sampleSSConfig(100000, file_path, true))
  {
    ROS_ERROR("Could not generate stable config");
    exit(1);
  }

  return 0;
}

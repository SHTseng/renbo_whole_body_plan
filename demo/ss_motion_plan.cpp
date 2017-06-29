#include <ros/ros.h>
#include <ros/package.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <renbo_msgs/compute_motion_plan.h>
#include <renbo_msgs/generate_whole_body_posture.h>

#include <fstream>

std::vector<double> wholeBodyJointMapping(std::vector<double> data);

std::vector<double> read_config_from_pose_file(const std::string& file_name)
{
  std::string package_path = ros::package::getPath("renbo_whole_body_plan")+"/database/";
  std::string f_path = package_path.append(file_name);
  std::ifstream input_file(f_path.c_str());
  if (!input_file)
  {
    ROS_ERROR("unable to open config file");
    exit(1);
  }

  std::vector<double> config;
  double v;
  while (input_file >> v)
  {
    v *= M_PI/180.0;
    config.push_back(v);
  }

  input_file.close();

  std::vector<double> ret = wholeBodyJointMapping(config);
  return ret;
}

std::vector<double> read_config_from_traj_file(const std::string& file_name, const int& pt)
{
  std::string package_path = ros::package::getPath("renbo_whole_body_plan")+"/database/";
  std::string f_path = package_path.append(file_name);
  std::ifstream input_file(f_path.c_str());
  if (!input_file)
  {
    ROS_ERROR("unable to open config file");
    exit(1);
  }

  int cnt = 0;
  std::vector<double> config;
  std::string str;
  while (1)
  {
    std::getline(input_file, str);
    if (cnt == pt)
    {
      std::istringstream in(str);
      std::copy(std::istream_iterator<double>(in), std::istream_iterator<double>(), std::back_inserter(config));
      break;
    }
    cnt++;
  }

  input_file.close();

  std::vector<double> ret = wholeBodyJointMapping(config);
  return ret;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ss_motion_plan");

  ros::NodeHandle nh_;

  ros::ServiceClient ss_motion_plan = nh_.serviceClient<renbo_msgs::compute_motion_plan>("BiRRT_motion_planner");
  renbo_msgs::compute_motion_plan ss_motion_plan_srv;

  ss_motion_plan_srv.request.scenario = 6; // empty space
  ss_motion_plan_srv.request.support_mode = 1;

  std::vector<double> initial_config(29, 0);
//  ss_motion_plan_srv.request.initial_config = read_config_from_traj_file("goal_1.txt", 950);
  ss_motion_plan_srv.request.initial_config = initial_config;
  ss_motion_plan_srv.request.goal_config = read_config_from_pose_file("goal_2.txt");

  if (ss_motion_plan.call(ss_motion_plan_srv))
  {
    ROS_INFO_STREAM("BiRRT planner return state: " << (int)ss_motion_plan_srv.response.success);
  }
  else
  {
    ROS_INFO_STREAM("BiRRT planner return state: " << (int)ss_motion_plan_srv.response.success);
    exit(1);
  }

  ros::shutdown();
  return 0;
}

std::vector<double> wholeBodyJointMapping(std::vector<double> data)
{
  std::vector<double> wb_joints(29, 0.0);

  // right leg
//  wb_joints[0] = data[32]*(-1);
//  wb_joints[1] = data[31];
//  wb_joints[2] = data[30];
//  wb_joints[3] = data[29]*(-1);
//  wb_joints[4] = data[28]*(-1);
//  wb_joints[5] = data[27];

//  // left leg
//  wb_joints[6] = data[21];
//  wb_joints[7] = data[22]*(-1);
//  wb_joints[8] = data[23]*(-1);
//  wb_joints[9] = data[24]*(-1);
//  wb_joints[10] = data[25]*(-1);
//  wb_joints[11] = data[26]*(-1);

  // swap right and left leg config
  // right leg
  wb_joints[0] = data[26];
  wb_joints[1] = data[25];
  wb_joints[2] = data[24];
  wb_joints[3] = data[23]*(-1);
  wb_joints[4] = data[22];
  wb_joints[5] = data[21];

  // left leg
  wb_joints[6] = data[27];
  wb_joints[7] = data[28];
  wb_joints[8] = data[29]*(-1);
  wb_joints[9] = data[30]*(-1);
  wb_joints[10] = data[31]*(-1);
  wb_joints[11] = data[32]*(-1);

  // torso
  wb_joints[12] = data[19]*(-1);
  wb_joints[13] = data[10];
  wb_joints[14] = data[4]*(-1);

  // right arm
  wb_joints[23] = data[8]*(-1);
  wb_joints[24] = data[11]*(-1);
  wb_joints[25] = data[15];
  wb_joints[26] = data[16]*(-1);
  wb_joints[27] = data[18];
  wb_joints[28] = data[17];

  // left arm
  wb_joints[17] = data[5];
  wb_joints[18] = data[3];
  wb_joints[19] = data[14];
  wb_joints[20] = data[13];
  wb_joints[21] = 0.0;
  wb_joints[22] = 0.0;

  return wb_joints;
}

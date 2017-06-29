#include <ros/ros.h>
#include <ros/package.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <string>
#include <sstream>
#include <fstream>

std::vector< std::vector<double>> read_from_rtx(const std::string& file_path, int scenario);

std::vector<double> wholeBodyJointMapping(std::vector<double> data);

//moveit_msgs::DisplayTrajectory buildTrajectory(std::vector< std::vector<double>> raw_data);

int main (int argc, char** argv)
{
  ros::init(argc, argv, "visulize_trajectory");

  if (argc != 2)
  {
    ROS_ERROR_STREAM("need 2 input arg");
    exit(1);
  }

  int scenario = std::atoi(argv[1]);

  std::string path = ros::package::getPath("renbo_whole_body_plan");
  path = path.append("/trajectory/WriteTrajectory");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  std::vector<std::string> wb_jnt_names = robot_model_loader.getModel()->getJointModelGroup("whole_body_fixed")->getActiveJointModelNames();

  //  for (auto i : wb_jnt_names)
  //    std::cout << i << std::endl;

  robot_state::RobotState rstate(robot_model_loader.getModel());
  rstate.setToDefaultValues();

  rviz_visual_tools::RvizVisualToolsPtr rvt;
  rvt.reset(new rviz_visual_tools::RvizVisualTools("r_sole", "planned_path_marker"));
  rvt->enableBatchPublishing(true);
  rvt->deleteAllMarkers();


  auto trajectory = read_from_rtx(path, 4);
  EigenSTL::vector_Affine3d path_pts;
  for (int i = 0; i < trajectory[0].size(); i++)
  {
    std::vector<double> current_config;
    for (int j = 0; j < trajectory.size(); j++)
      current_config.push_back(trajectory[j][i]);

    std::vector<double> mapped_config = wholeBodyJointMapping(current_config);
    rstate.setVariablePositions(wb_jnt_names, mapped_config);

    Eigen::Affine3d eef_pose = rstate.getGlobalLinkTransform("r_gripper");
    path_pts.push_back(eef_pose);
  }

  rvt->publishPath(path_pts, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);

  path_pts.clear();
  trajectory = read_from_rtx(path, 5);
  for (int i = 0; i < trajectory[0].size(); i+=10)
  {
    std::vector<double> current_config;
    for (int j = 0; j < trajectory.size(); j++)
      current_config.push_back(trajectory[j][i]);

    std::vector<double> mapped_config = wholeBodyJointMapping(current_config);
    rstate.setVariablePositions(wb_jnt_names, mapped_config);

    Eigen::Affine3d eef_pose = rstate.getGlobalLinkTransform("r_gripper");
    path_pts.push_back(eef_pose);
  }

  rvt->publishPath(path_pts, rviz_visual_tools::MAGENTA, rviz_visual_tools::LARGE);

  rvt->trigger();

  ros::Duration(1.0).sleep();


  //moveit_msgs::DisplayTrajectory disp_traj = buildTrajectory(trajectory);

  ros::shutdown();

  return 0;
}

std::vector< std::vector<double>> read_from_rtx(const std::string& file_path, int scenario)
{
  std::string file_name;
  std::string path = file_path;
  switch (scenario)
  {
    case 0:
    {
      file_name = path.append("/table/command.txt");
      break;
    }
    case 1:
    {
      file_name = path.append("/table/actual.txt");
      break;
    }
    case 2:
    {
      file_name = path.append("/closet/command.txt");
      break;
    }
    case 3:
    {
      file_name = path.append("/closet/actual.txt");
      break;
    }
    case 4:
    {
      file_name = path.append("/mug/command_1.txt");
      break;
    }
    case 5:
    {
      file_name = path.append("/mug/actual_1.txt");
      break;
    }
    case 6:
    {
      file_name = path.append("/mug/command_2.txt");
      break;
    }
    case 7:
    {
      file_name = path.append("/mug/actual_2.txt");
      break;
    }
  }

  std::ifstream traj_file(file_name.c_str());
  if (!traj_file)
  {
    ROS_ERROR("Can't open trajectory file");
  }
  std::vector< std::vector<double>> traj;
  std::string str;

  while (std::getline(traj_file, str))
  {
    std::vector<double> temp;
    std::istringstream in(str);
    std::copy(std::istream_iterator<double>(in), std::istream_iterator<double>(), std::back_inserter(temp));
    traj.push_back(temp);
  }

  traj_file.close();

  return traj;
}

std::vector<double> wholeBodyJointMapping(std::vector<double> data)
{
  std::vector<double> wb_joints(29, 0.0);

  // right leg
  wb_joints[0] = data[32]*(-1);
  wb_joints[1] = data[31];
  wb_joints[2] = data[30];
  wb_joints[3] = data[29]*(-1);
  wb_joints[4] = data[28]*(-1);
  wb_joints[5] = data[27];

  // left leg
  wb_joints[6] = data[21];
  wb_joints[7] = data[22]*(-1);
  wb_joints[8] = data[23]*(-1);
  wb_joints[9] = data[24]*(-1);
  wb_joints[10] = data[25]*(-1);
  wb_joints[11] = data[26]*(-1);

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
  wb_joints[17] = -0.53;
  wb_joints[20] = 1.04;

  return wb_joints;
}

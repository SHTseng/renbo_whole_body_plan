#include <renbo_whole_body_plan/nlp_ik_solver.h>

NLPIKSolver::NLPIKSolver()
{
  package_path_ = ros::package::getPath("renbo_description");

  urdf_path_ = package_path_.append("/robots/renbo_root_rfoot.urdf");
}

NLPIKSolver::~NLPIKSolver(){}

bool NLPIKSolver::solve(const Eigen::Affine3d& eef_pose, std::vector<double>& solved_pose)
{
  std::string file_path = drake::parsers::GetFullPath(urdf_path_);
  ROS_INFO_STREAM(file_path);

//  auto tree = std::make_unique<RigidBodyTree<double>>();
//  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf_path_,
//                                                            drake::multibody::joints::kRollPitchYaw,
//                                                            tree.get());
//
//  std::cout << "drake get urdf joint count" << tree->get_num_positions() << std::endl;
//
//  double inf = std::numeric_limits<double>::infinity();
//  Eigen::Vector2d t_span;
//  t_span << 0.0, inf;

//  Eigen::VectorXd reach_state(tree->get_num_positions());


  return true;
}

std::vector<int> NLPIKSolver::GetJointPositionVectorIndices(const RigidBodyTreed* tree,
                                               const std::string& name)
{
  RigidBody<double>* joint_child_body = tree->FindChildBodyOfJoint(name);
  int num_positions = joint_child_body->getJoint().get_num_positions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // Since the joint position states are located in a contiguous region of the
  // the rigid body tree's state vector, fill the return vector with
  // sequentially increasing indices starting at
  // `joint_child_body->get_position_start_index()`.
  std::iota(ret.begin(), ret.end(), joint_child_body->get_position_start_index());

  return ret;
}


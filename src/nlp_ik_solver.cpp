#include <renbo_whole_body_plan/nlp_ik_solver.h>

NLPIKSolver::NLPIKSolver()
{

  urdf_path_ = drake::GetDrakePath() + "/examples/RENBO/urdf/urdf/renbo_root_rfoot.urdf";
}

NLPIKSolver::~NLPIKSolver(){}

bool NLPIKSolver::solve(const Eigen::Affine3d& eef_pose, std::vector<double>& solved_pose)
{
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf_path_,
                                                            drake::multibody::joints::kRollPitchYaw,
                                                            tree.get());

  std::cout << "drake get urdf joint count: " << tree->get_num_positions() << std::endl;

  double inf = std::numeric_limits<double>::infinity();
  Eigen::Vector2d t_span;
  t_span << 0.0, inf;

  Eigen::VectorXd reach_start(tree->get_num_positions());
  reach_start << 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, // floating base joint
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;

  KinematicsCache<double> cache = tree->doKinematics(reach_start);

  PostureConstraint kc_posture_neck(tree.get(), t_span);
  std::vector<int> neck_idx;
  FindJointAndInsert(tree.get(), "head_yaw_joint", &neck_idx);
  FindJointAndInsert(tree.get(), "head_pitch_joint", &neck_idx);
  Eigen::VectorXd neck_lb = Eigen::VectorXd::Zero(neck_idx.size());
  Eigen::VectorXd neck_ub = Eigen::VectorXd::Zero(neck_idx.size());
  kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                 neck_ub);

  const Eigen::Vector3d origin(0, 0, 0);

  int l_foot = tree->FindBodyIndex("left_sole");
  Eigen::Vector4d lfoot_quat(1, 0, 0, 0);
  auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
  Eigen::Vector3d lfoot_pos_lb = lfoot_pos0;
  // Position and quaternion constraints are relaxed to make the problem
  // solvable by IPOPT.
  lfoot_pos_lb -= 0.0001*Eigen::Vector3d::Ones();
  Eigen::Vector3d lfoot_pos_ub = lfoot_pos0;
  lfoot_pos_ub += 0.0001*Eigen::Vector3d::Ones();
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                       lfoot_pos_ub, t_span);
  double tol = 0.5 / 180 * M_PI;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, t_span);

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

void NLPIKSolver::FindJointAndInsert(const RigidBodyTreed* model,
                        const std::string& name,
                        std::vector<int>* const position_list)
{
  auto position_indices = GetJointPositionVectorIndices(model, name);
  position_list->insert(position_list->end(), position_list->begin(), position_list->end());
}

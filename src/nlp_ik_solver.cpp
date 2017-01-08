#include <renbo_whole_body_plan/nlp_ik_solver.h>

NLPIKSolver::NLPIKSolver()
{
  urdf_path_ = drake::GetDrakePath() + "/examples/RENBO/urdf/urdf/renbo_root_rfoot.urdf";
}

NLPIKSolver::~NLPIKSolver(){}

bool NLPIKSolver::solve(const Eigen::Affine3d& desired_eef_pose, std::vector<double>& solved_pose)
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

  int l_foot = tree->FindBodyIndex("l_sole");
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

  // 3 Right foot position and orientation constraint
   int r_foot = tree->FindBodyIndex("r_sole");
   auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
   Eigen::Vector4d rfoot_quat(1, 0, 0, 0);
   Eigen::Vector3d rfoot_pos_lb = rfoot_pos0;
   rfoot_pos_lb -= 0.0001 * Eigen::Vector3d::Ones();
   Eigen::Vector3d rfoot_pos_ub = rfoot_pos0;
   rfoot_pos_ub += 0.0001 * Eigen::Vector3d::Ones();

   WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                        rfoot_pos_ub, t_span);
   WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, t_span);

//   // 4 Torso posture constraint
//   PostureConstraint kc_posture_torso(tree.get(), t_span);
//   std::vector<int> torso_idx;
//   FindJointAndInsert(tree.get(), "trunk_yaw_joint", &torso_idx);
//   FindJointAndInsert(tree.get(), "trunk_pitch_joint", &torso_idx);
//   FindJointAndInsert(tree.get(), "trunk_roll_joint", &torso_idx);
//   Eigen::Vector3d torso_nominal = Eigen::Vector3d::Zero();
//   Eigen::Vector3d torso_half_range(15.0 / 180 * M_PI, 25.0 / 180 * M_PI, inf);
//   Eigen::Vector3d torso_lb = torso_nominal - torso_half_range;
//   Eigen::Vector3d torso_ub = torso_nominal + torso_half_range;
//   torso_lb(1) = -5.0 / 180 * M_PI;
//   kc_posture_torso.setJointLimits(3, torso_idx.data(), torso_lb, torso_ub);

//   // 5 knee posture constraint
//   PostureConstraint kc_posture_knee(tree.get(), t_span);
//   std::vector<int> knee_idx;
//   FindJointAndInsert(tree.get(), "l_knee_joint", &knee_idx);
//   FindJointAndInsert(tree.get(), "r_knee_joint", &knee_idx);
//   Eigen::Vector2d knee_lb(-4.0/180*M_PI, -4.0/180*M_PI);
//   Eigen::Vector2d knee_ub(115.0/180*M_PI, 115.0/180*M_PI);
//   kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

//   // 6 Left arm posture constraint
//   PostureConstraint kc_posture_larm(tree.get(), t_span);
//   std::vector<int> larm_idx;
//   FindJointAndInsert(tree.get(), "l_shoulder_pitch_joint", &larm_idx);
//   FindJointAndInsert(tree.get(), "l_shoulder_roll_joint", &larm_idx);
//   FindJointAndInsert(tree.get(), "l_shoulder_yaw_joint", &larm_idx);
//   FindJointAndInsert(tree.get(), "l_elbow_joint", &larm_idx);
//   FindJointAndInsert(tree.get(), "l_wrist_yaw_joint", &larm_idx);
//   FindJointAndInsert(tree.get(), "l_wrist_pitch_joint", &larm_idx);
//   Eigen::Matrix<double, 6, 1> larm_lb;
//   larm_lb.setZero();
//   for (int i = 0; i < 6; i++) larm_lb(i) = reach_start(larm_idx[i]);
//   Eigen::Matrix<double, 6, 1> larm_ub = larm_lb;
//   kc_posture_larm.setJointLimits(6, larm_idx.data(), larm_lb, larm_ub);

//   // 7 Right arm posture constraint
//   PostureConstraint kc_posture_rarm(tree.get(), t_span);
//   std::vector<int> rarm_idx;
//   FindJointAndInsert(tree.get(), "r_shoulder_pitch_joint", &rarm_idx);
//   FindJointAndInsert(tree.get(), "r_shoulder_roll_joint", &rarm_idx);
//   FindJointAndInsert(tree.get(), "r_shoulder_yaw_joint", &rarm_idx);
//   FindJointAndInsert(tree.get(), "r_elbow_joint", &rarm_idx);
//   FindJointAndInsert(tree.get(), "r_wrist_yaw_joint", &rarm_idx);
//   FindJointAndInsert(tree.get(), "r_wrist_pitch_joint", &rarm_idx);
//   Eigen::Matrix<double, 6, 1> rarm_lb;
//   rarm_lb.setZero();
//   for (int i = 0; i < 6; i++) rarm_lb(i) = reach_start(rarm_idx[i]);
//   Eigen::Matrix<double, 6, 1> rarm_ub = rarm_lb;
//   kc_posture_rarm.setJointLimits(6, rarm_idx.data(), rarm_lb, rarm_ub);

//   // 8 Quasistatic constraint
//   QuasiStaticConstraint kc_quasi(tree.get(), t_span);
//   kc_quasi.setShrinkFactor(0.4);
//   kc_quasi.setActive(true);

//   auto leftFootPtr = tree->FindBody("L_sole");
//   Eigen::Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
//   Eigen::Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
//   kc_quasi.addContact(1, &l_foot, &l_foot_pts);

//   auto rightFootPtr = tree->FindBody("r_sole");
//   Eigen::Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
//   Eigen::Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
//   kc_quasi.addContact(1, &r_foot, &r_foot_pts);

   // -----------------solve-----------------------------------------------------
   std::vector<RigidBodyConstraint*> constraint_array;
   constraint_array.push_back(&kc_posture_neck);
   constraint_array.push_back(&kc_lfoot_pos);
   constraint_array.push_back(&kc_lfoot_quat);
   constraint_array.push_back(&kc_rfoot_pos);
   constraint_array.push_back(&kc_rfoot_quat);
//   constraint_array.push_back(&kc_posture_torso);
//   constraint_array.push_back(&kc_posture_knee);
//   constraint_array.push_back(&kc_posture_larm);
//   constraint_array.push_back(&kc_posture_rarm);
//   constraint_array.push_back(&kc_quasi);

//   IKoptions ikoptions(tree.get());
//   Eigen::VectorXd q_sol(tree->get_num_positions());
//   Eigen::VectorXd q_nom = reach_start;
//   int info;
//   std::vector<std::string> infeasible_constraint;
//   inverseKin(tree.get(), q_nom, q_nom, constraint_array.size(),
//              constraint_array.data(), ikoptions, &q_sol, &info,
//              &infeasible_constraint);

//   for (int i = 6; i < tree->get_num_positions(); i++)
//   {
//     solved_pose.push_back(q_sol(i));
//   }

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

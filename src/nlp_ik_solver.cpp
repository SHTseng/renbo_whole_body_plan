#include <renbo_whole_body_plan/nlp_ik_solver.h>

NLPIKSolver::NLPIKSolver()
{
  urdf_path_ = drake::GetDrakePath() + "/examples/RENBO/urdf/urdf/renbo_no_waist.urdf";
}

NLPIKSolver::~NLPIKSolver()
{

}

void NLPIKSolver::initialization()
{

}

bool NLPIKSolver::solve(const Eigen::Affine3d& desired_eef_pose, std::map<std::string, double>& jnt_pos_map)
{
  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf_path_,
                                                            drake::multibody::joints::kRollPitchYaw,
                                                            tree.get());
ROS_INFO_STREAM(tree->get_num_positions());

  double inf = std::numeric_limits<double>::infinity();
  Eigen::Vector2d t_span;
  t_span << 0, 1;

  Eigen::VectorXd reach_start(tree->get_num_positions());
  reach_start << 0.0572, 0.0, 0.8, 0.0, 0.0, 0.0, // 5, floating base joint
          0.0, 0.0, 0.0, 0.0, -0.316, // 10
          0.0, 0.0, -0.316, 0.0, 0.0, // 15
          0.63, 0.63, 0.0, 0.0, 0.0, // 20
          0.0, 0.0, 0.0, -0.3923, 0.0, // 25
          0.0, 0.3923, 0.785, -0.785, -0.3923, // 30
          0.0, 0.3923, 0.0, 0.0, 0.0; // 35

  KinematicsCache<double> cache = tree->doKinematics(reach_start);

  PostureConstraint kc_posture_neck(tree.get(), t_span);
  std::vector<int> neck_idx;
  FindJointAndInsert(tree.get(), "head_yaw_joint", neck_idx);
  FindJointAndInsert(tree.get(), "head_pitch_joint", neck_idx);
  Eigen::VectorXd neck_lb = Eigen::VectorXd::Zero(neck_idx.size());
  Eigen::VectorXd neck_ub = Eigen::VectorXd::Zero(neck_idx.size());
  kc_posture_neck.setJointLimits(neck_idx.size(), neck_idx.data(), neck_lb,
                                 neck_ub);

  const Eigen::Vector3d origin(0, 0, 0);

  int l_foot = tree->FindBodyIndex("l_sole");
  Eigen::Vector4d lfoot_quat(1, 0, 0, 0);
//  Eigen::Vector4d lfoot_quat(0.5, 0.5, 0.5, 0.5);

  auto lfoot_pos0 = tree->transformPoints(cache, origin, l_foot, 0);
  Eigen::Vector3d lfoot_pos_lb = lfoot_pos0;

  std::cout << lfoot_pos_lb << std::endl;

  // Position and quaternion constraints are relaxed to make the problem
  // solvable by IPOPT.
  lfoot_pos_lb -= 0.01*Eigen::Vector3d::Ones();
//  lfoot_pos_lb(2) += 0.08;
  Eigen::Vector3d lfoot_pos_ub = lfoot_pos0;
  lfoot_pos_ub += 0.01*Eigen::Vector3d::Ones();
//  lfoot_pos_ub(2) += 0.08;
  WorldPositionConstraint kc_lfoot_pos(tree.get(), l_foot, origin, lfoot_pos_lb,
                                       lfoot_pos_ub, t_span);
  double tol = 0.5 / 180 * M_PI;
  WorldQuatConstraint kc_lfoot_quat(tree.get(), l_foot, lfoot_quat, tol, t_span);

  // 3 Right foot position and orientation constraint
  int r_foot = tree->FindBodyIndex("r_sole");
  auto rfoot_pos0 = tree->transformPoints(cache, origin, r_foot, 0);
  Eigen::Vector4d rfoot_quat(1, 0, 0, 0);
  Eigen::Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb -= 0.01 * Eigen::Vector3d::Ones();
  Eigen::Vector3d rfoot_pos_ub = rfoot_pos0;
  rfoot_pos_ub += 0.01 * Eigen::Vector3d::Ones();

  WorldPositionConstraint kc_rfoot_pos(tree.get(), r_foot, origin, rfoot_pos_lb,
                                      rfoot_pos_ub, t_span);
  WorldQuatConstraint kc_rfoot_quat(tree.get(), r_foot, rfoot_quat, tol, t_span);

  // 4 Torso posture constraint
  PostureConstraint kc_posture_torso(tree.get(), t_span);
  std::vector<int> torso_idx;
  FindJointAndInsert(tree.get(), "trunk_yaw_joint", torso_idx); // -60~60
  FindJointAndInsert(tree.get(), "trunk_pitch_joint", torso_idx); // lower="-0.576" upper="0.7156"
  FindJointAndInsert(tree.get(), "trunk_roll_joint", torso_idx); // lower="-0.7155" upper="0.7155"

  Eigen::VectorXd torso_nominal = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd torso_half_range(3);
  torso_half_range(0) = 15.0 / 180 * M_PI;
  torso_half_range(1) = 25.0 / 180 * M_PI;
  torso_half_range(2) = 45.0 / 180 * M_PI;
  Eigen::VectorXd torso_lb = torso_nominal - torso_half_range;
  Eigen::VectorXd torso_ub = torso_nominal + torso_half_range;
//  torso_lb(1) = -5.0 / 180 * M_PI;

  kc_posture_torso.setJointLimits(torso_idx.size(), torso_idx.data(), torso_lb, torso_ub);

  // 5 knee posture constraint
  PostureConstraint kc_posture_knee(tree.get(), t_span);
  std::vector<int> knee_idx;
  FindJointAndInsert(tree.get(), "l_knee_joint", knee_idx);
  FindJointAndInsert(tree.get(), "r_knee_joint", knee_idx);

  Eigen::VectorXd knee_lb = Eigen::VectorXd::Zero(2);
  knee_lb(0) = 0.0/180*M_PI;
//  knee_lb(0) = -75.0/180*M_PI;

  knee_lb(1) = 0.0/180*M_PI;

  Eigen::VectorXd knee_ub = Eigen::VectorXd::Zero(2);
  knee_ub(0) = 9.0/180*M_PI;
//  knee_ub(0) = 0.0/180*M_PI;
  knee_ub(1) = 90.0/180*M_PI;

  kc_posture_knee.setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // 6 Left arm posture constraint
  PostureConstraint kc_posture_larm(tree.get(), t_span);
  std::vector<int> larm_idx;
  FindJointAndInsert(tree.get(), "l_shoulder_pitch_joint", larm_idx);
  FindJointAndInsert(tree.get(), "l_shoulder_roll_joint", larm_idx);
  FindJointAndInsert(tree.get(), "l_shoulder_yaw_joint", larm_idx);
  FindJointAndInsert(tree.get(), "l_elbow_joint", larm_idx);
  FindJointAndInsert(tree.get(), "l_wrist_yaw_joint", larm_idx);
  FindJointAndInsert(tree.get(), "l_wrist_pitch_joint", larm_idx);
  Eigen::Matrix<double, 6, 1> larm_lb;
  larm_lb.setZero();
  for (int i = 0; i < 6; i++)
  {
   larm_lb(i) = reach_start(larm_idx[i]);
  }
  Eigen::Matrix<double, 6, 1> larm_ub = larm_lb;
  kc_posture_larm.setJointLimits(6, larm_idx.data(), larm_lb, larm_ub);

  // 7 Right arm posture constraint
  PostureConstraint kc_posture_rarm(tree.get(), t_span);
  std::vector<int> rarm_idx;
  FindJointAndInsert(tree.get(), "r_shoulder_pitch_joint", rarm_idx);
  FindJointAndInsert(tree.get(), "r_shoulder_roll_joint", rarm_idx);
  FindJointAndInsert(tree.get(), "r_shoulder_yaw_joint", rarm_idx);
  FindJointAndInsert(tree.get(), "r_elbow_joint", rarm_idx);
  FindJointAndInsert(tree.get(), "r_wrist_yaw_joint", rarm_idx);
  FindJointAndInsert(tree.get(), "r_wrist_pitch_joint", rarm_idx);

  Eigen::Matrix<double, 6, 1> rarm_lb;
  rarm_lb.setZero();
  rarm_lb(0) = -2.193;
  rarm_lb(1) = 0.0;
  rarm_lb(2) = -1.57;
  rarm_lb(3) = -0.2618;
  rarm_lb(4) = -1.57;
  rarm_lb(5) = -1.8;
//  for (int i = 0; i < 6; i++)
//  {
//   rarm_lb(i) = reach_start(rarm_idx[i]);
//  }
//  Eigen::Matrix<double, 6, 1> rarm_ub = rarm_lb;
  Eigen::Matrix<double, 6, 1> rarm_ub;
  rarm_ub(0) = 2.193;
  rarm_ub(0) = 1.57;
  rarm_ub(0) = 1.57;
  rarm_ub(0) = 2.0;
  rarm_ub(0) = 1.57;
  rarm_ub(0) = 1.8;

  kc_posture_rarm.setJointLimits(6, rarm_idx.data(), rarm_lb, rarm_ub);

  // 8 Quasistatic constraint
  QuasiStaticConstraint kc_quasi(tree.get(), t_span);
  kc_quasi.setShrinkFactor(0.4);
  kc_quasi.setActive(true);

//  auto leftFootPtr = tree->FindBody("l_sole");
//  Eigen::Matrix3Xd leftFootContactPts = leftFootPtr->get_contact_points();
//  Eigen::Matrix3Xd l_foot_pts = leftFootContactPts.rightCols(8);
//  kc_quasi.addContact(1, &l_foot, &l_foot_pts);

//  auto rightFootPtr = tree->FindBody("r_sole");
//  Eigen::Matrix3Xd rightFootContactPts = rightFootPtr->get_contact_points();
//  Eigen::Matrix3Xd r_foot_pts = rightFootContactPts.rightCols(8);
//  kc_quasi.addContact(1, &r_foot, &r_foot_pts);

  // 9 End-effector constraint
  int r_gripper_idx = tree->FindBodyIndex("r_gripper");
  Eigen::VectorXd r_gripper_pos = Eigen::VectorXd::Zero(3);


  auto origin_r_gripper_pose = tree->transformPoints(cache, origin, r_gripper_idx, 0);
  std::cout << "Original eef position \n" << origin_r_gripper_pose << std::endl;

//  r_gripper_pos(0) = desired_eef_pose.translation().x();
//  r_gripper_pos(1) = desired_eef_pose.translation().y();
//  r_gripper_pos(2) = desired_eef_pose.translation().z();

  r_gripper_pos(0) = origin_r_gripper_pose(0)+0.2; // 0.052
  r_gripper_pos(1) = origin_r_gripper_pose(1)-0.05; // -0.153
  r_gripper_pos(2) = origin_r_gripper_pose(2)+0.1;  // 0.793

  Eigen::VectorXd r_gripper_pos_lb = r_gripper_pos;
  r_gripper_pos_lb(0) -= 0.001;
  r_gripper_pos_lb(1) -= 0.001;
  r_gripper_pos_lb(2) -= 0.001;

  Eigen::VectorXd r_gripper_pos_ub = r_gripper_pos;
  r_gripper_pos_ub(0) += 0.001;
  r_gripper_pos_ub(1) += 0.001;
  r_gripper_pos_ub(2) += 0.001;

  WorldPositionConstraint kc_r_gripper_pos(tree.get(), r_gripper_idx, origin,
                                       r_gripper_pos_lb, r_gripper_pos_ub, t_span);

 tol = 0.1 / 180.0 * M_PI;
  Eigen::Vector4d r_gripper_quat(0.7071, 0.707, 0.0, 0.0);
  WorldQuatConstraint kc_r_gripper_quat(tree.get(), r_gripper_idx, r_gripper_quat, tol, t_span);

  // -----------------solve-----------------------------------------------------
  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&kc_posture_neck);
  constraint_array.push_back(&kc_lfoot_pos);
//  constraint_array.push_back(&kc_lfoot_quat);
  constraint_array.push_back(&kc_rfoot_pos);
//  constraint_array.push_back(&kc_rfoot_quat);
//  constraint_array.push_back(&kc_posture_torso);
//  constraint_array.push_back(&kc_posture_knee);
//  constraint_array.push_back(&kc_posture_larm);
//  constraint_array.push_back(&kc_posture_rarm);
//  constraint_array.push_back(&kc_quasi);
  constraint_array.push_back(&kc_r_gripper_pos);
//  constraint_array.push_back(&kc_r_gripper_quat);

  IKoptions ikoptions(tree.get());
  Eigen::VectorXd q_sol(tree->get_num_positions());
  Eigen::VectorXd q_nom = reach_start;
  int info;
  std::vector<std::string> infeasible_constraint;
  inverseKin(tree.get(), q_nom, q_nom, constraint_array.size(),
            constraint_array.data(), ikoptions, &q_sol, &info,
            &infeasible_constraint);
  ROS_INFO("info = %d\n", info);
  for (auto i : infeasible_constraint)
    std::cout << i << std::endl;

  Eigen::Vector3d com_orig = tree->centerOfMass(cache);

  ROS_INFO("com position \n %5.6f\n%5.6f\n%5.6f\n", com_orig(0), com_orig(1), com_orig(2));

  cache = tree->doKinematics(q_sol);
  Eigen::Vector3d com = tree->centerOfMass(cache);

  ROS_INFO("com position \n %5.6f\n%5.6f\n%5.6f\n", com(0), com(1), com(2));

  auto solved_r_gripper_pose = tree->transformPoints(cache, origin, r_gripper_idx, 0);
  auto solved_r_sole_pose = tree->transformPoints(cache, origin, r_foot, 0);
  auto solved_l_sole_pose = tree->transformPoints(cache, origin, l_foot, 0);

  std::cout << "r_gripper:\n" << solved_r_gripper_pose << std::endl;
  std::cout << "l_sole:\n" << solved_r_sole_pose << std::endl;
  std::cout << "r_sole:\n" << solved_l_sole_pose << std::endl;

  for (int i = 6; i < tree->get_num_positions(); i++)
  {
   ROS_INFO_STREAM(i << " " << tree->get_position_name(i) << ": " << q_sol(i));
   jnt_pos_map.insert(std::pair<std::string, double>(tree->get_position_name(i), q_sol(i)));
  }

  jnt_pos_map.erase("waist_joint");
//  jnt_pos_map["r_ankle_roll_joint"] *= (-1);
//  jnt_pos_map["r_ankle_pitch_joint"] *= (-1);
//  jnt_pos_map["r_knee_joint"] *= (-1);
 jnt_pos_map["r_hip_pitch_joint"] *= (-1);
//  jnt_pos_map["r_hip_roll_joint"] *= (-1);
//  jnt_pos_map["r_hip_yaw_joint"] *= (-1);


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
                        std::vector<int>& position_list)
{
  auto position_indices = GetJointPositionVectorIndices(model, name);

  position_list.insert(position_list.end(),
                        position_indices.begin(), position_indices.end());
}










#include <renbo_whole_body_plan/double_support_constraint.h>

namespace renbo_constraint_sampler
{

DoubleSupportConstraint::DoubleSupportConstraint():
  verbose_(false)
{

}

DoubleSupportConstraint::~DoubleSupportConstraint()
{

}

bool DoubleSupportConstraint::enforceDSLeftLeg(robot_state::RobotState& state, const robot_state::JointModelGroup *jmg_)
{
  // finding the current random configuraion of hips and soles
  r_foot_config_ = state.getGlobalLinkTransform("r_sole");

  Eigen::Matrix3d rot_x, rot_z;
  rot_x = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX());
  rot_z = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ());

  l_foot_fixed_config_.linear() = rot_z*rot_x*r_foot_config_.linear();
  l_foot_fixed_config_.translation() << 0.0, 0.2477, 0.15005;

  bool ik_successed = false;

  if(state.setFromIK(jmg_, l_foot_fixed_config_, 1, 0))
  {
    state.update();

    ik_successed = true;
  }

  return ik_successed;
}

}

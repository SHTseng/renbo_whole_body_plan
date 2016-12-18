#ifndef DOUBLE_SUPPORT_CONSTRAINT_H_
#define DOUBLE_SUPPORT_CONSTRAINT_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <random_numbers/random_numbers.h>

#include <moveit/robot_state/robot_state.h>

namespace renbo_constraint_sampler
{

class DoubleSupportConstraint
{

public:

  DoubleSupportConstraint();

  ~DoubleSupportConstraint();

  bool enforceDSLeftLeg(moveit::core::RobotState &state, const robot_state::JointModelGroup* jmg_);

private:

  Eigen::Affine3d r_hip_config_;
  Eigen::Affine3d r_foot_config_;

  Eigen::Affine3d l_hip_mimic_config_;
  Eigen::Affine3d l_foot_fixed_config_;

  bool verbose_;

};

}

#endif

#ifndef NLP_IK_SOLVER_
#define NLP_IK_SOLVER_

#include <iostream>
#include <memory>
#include <numeric>

#include <ros/ros.h>
#include <ros/package.h>

#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsers/package_map.h"

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"


class NLPIKSolver
{
public:

  NLPIKSolver();

  ~NLPIKSolver();

  bool solve(const Eigen::Affine3d& eef_pose, std::vector<double>& solved_pose);

private:

  std::vector<int> GetJointPositionVectorIndices(const RigidBodyTreed* tree,
                                                   const std::string& name);

  std::string package_path_;

  std::string urdf_path_;

};





#endif

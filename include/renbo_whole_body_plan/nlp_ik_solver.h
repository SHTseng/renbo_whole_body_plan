#ifndef NLP_IK_SOLVER_
#define NLP_IK_SOLVER_

#include <ros/ros.h>

#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsers/package_map.h"

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include <iostream>
#include <numeric>
#include <fstream>

class NLPIKSolver
{
public:

  NLPIKSolver();

  ~NLPIKSolver();

  bool solve(const Eigen::Affine3d& desired_eef_pose, std::map<std::string, double>& jnt_pos_map);

private:

  void initialization();

  bool readConfigFromFile(const std::string& path, std::vector<double> &read_conifg);

  std::vector<int> GetJointPositionVectorIndices(const RigidBodyTreed* tree,
                                                   const std::string& name);

  void FindJointAndInsert(const RigidBodyTreed* model,
                          const std::string& name,
                          std::vector<int>& position_list);

  std::string urdf_path_;

};





#endif

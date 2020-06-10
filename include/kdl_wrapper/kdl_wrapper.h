// Copyright 2020 Markus Bjønnes and Marius Nilsen.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <vector>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <angles/angles.h>
#include <kdl/frames_io.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class KdlWrapper
{
public:
  KdlWrapper(urdf::Model);

  /* Setup up solvers and limits */
  bool init();

  /* Inverse kinematics for right arm. */
  KDL::JntArray inverse_kinematics_right(KDL::Frame &frame_dest, KDL::JntArray &q_seed);

  /* Inverse kinematics for left arm. */
  KDL::JntArray inverse_kinematics_left(KDL::Frame &frame_dest, KDL::JntArray &q_seed);

  /* Forward kinematics for right arm. */
  KDL::Frame forward_kinematics_right(KDL::JntArray joint_config);

  /* Forward kinematics for right arm. */
  KDL::Frame forward_kinematics_left(KDL::JntArray joint_config);

  /* Fetch functions. */
  KDL::Chain get_right_arm();
  KDL::Chain get_left_arm();

  /* Set function.s */
  void set_iterations(double iterations) { iterations_ = iterations; }
  void set_allowed_error(double allowed_error) { allowed_error_ = allowed_error_; }

  void print_info();

  /* Dynamics functions. */
  KDL::JntSpaceInertiaMatrix dynamics_inertia(std::string mech_unit, KDL::JntArray &q);
  KDL::JntArray dynamics_coriolis(std::string mech_unit, KDL::JntArray &q, KDL::JntArray &q_dot);
  KDL::JntArray dynamics_gravity(std::string mech_unit, KDL::JntArray &q);
  KDL::Jacobian calculate_jacobian(std::string mech_unit, KDL::JntArray &q);

  //overloads using std::vectors
  KDL::JntSpaceInertiaMatrix dynamics_inertia(std::string mech_unit, std::vector<float> &q);
  KDL::JntArray dynamics_coriolis(std::string mech_unit, std::vector<float> &q, std::vector<float> &q_dot);
  KDL::JntArray dynamics_gravity(std::string mech_unit, std::vector<float> &q);
  KDL::Jacobian calculate_jacobian(std::string mech_unit, std::vector<float> &q);
  
  // converts std::vector<float> to KDL::JntArray
  KDL::JntArray stdvec_to_jntarray(std::vector<double> &vec);
  KDL::JntArray stdvec_to_jntarray(std::vector<float> &vec);
  std::vector<double> jntarray_to_stdvec(KDL::JntArray& jntarray);

private:
  std::shared_ptr<rclcpp::Node> node_;
  urdf::Model robot_urdf_;
  KDL::Tree robot_tree_;
  KDL::Chain left_arm_;
  KDL::Chain right_arm_;
  double allowed_error_;
  double iterations_;

  /* Right arm solvers */
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_r_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_pinv_r_;
  std::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_vel_wdls_r_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos_nrjl_r_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver_pos_nr_r_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_pos_lma_r_;

  /* Left arm solvers */
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_pos_l_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_pinv_l_;
  std::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_vel_wdls_l_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos_nrjl_l_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver_pos_nr_l_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_pos_lma_l_;

  KDL::JntArray joint_min_r_;
  KDL::JntArray joint_max_r_;
  KDL::JntArray joint_min_l_;
  KDL::JntArray joint_max_l_;

  /*dynamics solvers */
  std::shared_ptr<KDL::ChainDynParam> dynamics_solver_right_;
  std::shared_ptr<KDL::ChainDynParam> dynamics_solver_left_;

  /*jacobian solver*/
  std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_right_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_left_;

  /*dynamics matrices*/
  KDL::JntSpaceInertiaMatrix inertia_right_;
  KDL::JntArray coriolis_right_;
  KDL::JntArray gravity_right_;
  KDL::JntSpaceInertiaMatrix inertia_left_;
  KDL::JntArray coriolis_left_;
  KDL::JntArray gravity_left_;
  /*gravity vector*/
  KDL::Vector grav_{0.0, 0.0, -9.81};

  /*jacobian*/
  KDL::Jacobian jacobian_right_;
  KDL::Jacobian jacobian_left_;

  /* Adjust seed */
  void adjust_q_seed(KDL::JntArray &q_seed);
  // Helper funcitons
  bool store_tree_from_urdf(urdf::Model urdf_model);

}; //end KdlWrapper



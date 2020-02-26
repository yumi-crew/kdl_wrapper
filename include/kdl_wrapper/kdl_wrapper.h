#ifndef KDL_WRAPPER_H_
#define KDL_WRAPPER_H_

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


class KdlWrapper
{
public:

  KdlWrapper(urdf::Model);

  /* Setup up solvers and limits */
  bool init();

  /* Inverse kinematics for right arm */
  KDL::JntArray inverse_kinematics_right(KDL::Frame& frame_dest, KDL::JntArray& q_seed);

  /* Inverse kinematics for left arm */
  KDL::JntArray inverse_kinematics_left(KDL::Frame& frame_dest, KDL::JntArray& q_seed);

  /* Forward kinematics for right arm */
  KDL::Frame forward_kinematics_right(KDL::JntArray joint_config);

  /* Forward kinematics for right arm */
  KDL::Frame forward_kinematics_left(KDL::JntArray joint_config);

  /* Fetch functions */
  KDL::Chain get_right_arm();
  KDL::Chain get_left_arm();
  
  /* set functions */
  void set_iterations(double iterations) {iterations_ = iterations;}
  void set_allowed_error(double allowed_error) {allowed_error_ = allowed_error_;}

  void print_info();

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

  // Helper funcitons
  bool store_tree_from_urdf(urdf::Model urdf_model);

}; //end KdlWrapper



#endif //KDL_WRAPPER_H_
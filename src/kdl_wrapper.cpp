#include "kdl_wrapper/kdl_wrapper.h"

KdlWrapper::KdlWrapper(urdf::Model urdf_model)
    : robot_urdf_{urdf_model}
{
}

bool KdlWrapper::init()
{
  if (!store_tree_from_urdf(robot_urdf_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to store tree from urdf file");
    return false;
  }
  if (!robot_tree_.getChain("yumi_body", "gripper_l_center", left_arm_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to load left arm chain from tree");
    return false;
  }
  if (!robot_tree_.getChain("yumi_body", "gripper_r_center", right_arm_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to load right arm chain from tree");
    return false;
  }

  // Joint position limits
  joint_min_r_.resize(right_arm_.getNrOfJoints());
  joint_max_r_.resize(right_arm_.getNrOfJoints());

  // Allowed jointwise error
  allowed_error_ = 1e-6; //angles::from_degrees(0.001);
  // Allowed iterations
  iterations_ = 1000;

  // Right arm joint limits
  joint_min_r_(0) = -2.94087978961;
  joint_max_r_(0) = 2.94087978961;
  joint_min_r_(1) = -2.50454747661;
  joint_max_r_(1) = 0.759218224618;
  joint_min_r_(2) = -2.94087978961;
  joint_max_r_(2) = 2.94087978961;
  joint_min_r_(3) = -2.15548162621;
  joint_max_r_(3) = 1.3962634016;
  joint_min_r_(4) = -5.06145483078;
  joint_max_r_(4) = 5.06145483078;
  joint_min_r_(5) = -1.53588974176;
  joint_max_r_(5) = 2.40855436775;
  joint_min_r_(6) = -3.99680398707;
  joint_max_r_(6) = 3.99680398707;

  // Left arm joint limits
  joint_min_l_ = joint_min_r_;
  joint_max_l_ = joint_max_r_;

  // Right arm solvers
  fk_solver_pos_r_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_arm_);
  ik_solver_vel_pinv_r_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(right_arm_);
  ik_solver_vel_wdls_r_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(right_arm_);
  ik_solver_pos_nrjl_r_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(right_arm_, joint_min_r_, joint_max_r_,
                                                                        *fk_solver_pos_r_.get(),
                                                                        *ik_solver_vel_wdls_r_.get(),
                                                                        iterations_, allowed_error_);

  ik_solver_pos_nr_r_ = std::make_shared<KDL::ChainIkSolverPos_NR>(right_arm_, *fk_solver_pos_r_.get(),
                                                                   *ik_solver_vel_wdls_r_.get(),
                                                                   iterations_, allowed_error_);

  ik_solver_pos_lma_r_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(right_arm_);

  // Left arm solvers
  fk_solver_pos_l_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_arm_);
  ik_solver_vel_pinv_l_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(left_arm_);
  ik_solver_vel_wdls_l_ = std::make_shared<KDL::ChainIkSolverVel_wdls>(left_arm_);
  ik_solver_pos_nrjl_l_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(left_arm_, joint_min_l_, joint_max_l_,
                                                                        *fk_solver_pos_l_.get(),
                                                                        *ik_solver_vel_wdls_l_.get(),
                                                                        iterations_, allowed_error_);

  ik_solver_pos_nr_l_ = std::make_shared<KDL::ChainIkSolverPos_NR>(left_arm_, *fk_solver_pos_l_.get(),
                                                                   *ik_solver_vel_wdls_l_.get(),
                                                                   iterations_, allowed_error_);

  ik_solver_pos_lma_l_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(left_arm_);

  // initialize dynamics solvers for right and left arm
  dynamics_solver_right_ = std::make_shared<KDL::ChainDynParam>(right_arm_, grav_);
  dynamics_solver_left_ = std::make_shared<KDL::ChainDynParam>(left_arm_, grav_);

  jacobian_solver_right_ = std::make_shared<KDL::ChainJntToJacSolver>(right_arm_);
  jacobian_solver_left_ = std::make_shared<KDL::ChainJntToJacSolver>(left_arm_);


  inertia_right_ = KDL::JntSpaceInertiaMatrix(right_arm_.getNrOfJoints());
  inertia_left_ = KDL::JntSpaceInertiaMatrix(left_arm_.getNrOfJoints());
  coriolis_right_ = KDL::JntArray(right_arm_.getNrOfJoints());
  coriolis_left_ = KDL::JntArray(left_arm_.getNrOfJoints());
  gravity_right_ = KDL::JntArray(right_arm_.getNrOfJoints());
  gravity_left_ = KDL::JntArray(left_arm_.getNrOfJoints());

  jacobian_right_ = KDL::Jacobian(right_arm_.getNrOfJoints());
  jacobian_left_ = KDL::Jacobian(left_arm_.getNrOfJoints());
  return true;
}

bool KdlWrapper::store_tree_from_urdf(urdf::Model urdf_model)
{
  if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_tree_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree");
    return false;
  }
  return true;
}

KDL::JntArray KdlWrapper::inverse_kinematics_right(KDL::Frame &frame_cart_pose, KDL::JntArray &q_seed)
{
  KDL::JntArray q(right_arm_.getNrOfJoints());
  int rc{-10};
  while (rc != 0)
  {
    rc = ik_solver_pos_nrjl_r_->CartToJnt(q_seed, frame_cart_pose, q);
    if (rc != 0)
    {
      adjust_q_seed(q_seed);
    }
  }
  if (rc != 0)
    throw std::runtime_error("Inverse solver for right arm failed, rc: " + std::to_string(rc));
  else
    return q;
}

KDL::JntArray KdlWrapper::inverse_kinematics_left(KDL::Frame &frame_cart_pose, KDL::JntArray &q_seed)
{
  KDL::JntArray q(left_arm_.getNrOfJoints());
  int rc{-10};
  while (rc != 0)
  {
    rc = ik_solver_pos_nrjl_l_->CartToJnt(q_seed, frame_cart_pose, q);
    if (rc != 0)
    {
      adjust_q_seed(q_seed);
    }
  }
  if (rc != 0)
    throw std::runtime_error("Inverse solver for left arm failed, rc: " + std::to_string(rc));
  else
    return q;
}

void KdlWrapper::adjust_q_seed(KDL::JntArray &q_seed)
{
  for (int i = 0; i < 7; ++i)
  {
    q_seed(i) += 0.01;
  }
}

KDL::Frame KdlWrapper::forward_kinematics_right(KDL::JntArray joint_config)
{
  KDL::Frame frame;
  int rc = fk_solver_pos_r_->JntToCart(joint_config, frame);
  if (rc != 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Something went wrong. rc value: " << rc);
  }
  return frame;
}

KDL::Frame KdlWrapper::forward_kinematics_left(KDL::JntArray joint_config)
{
  KDL::Frame frame;
  int rc = fk_solver_pos_l_->JntToCart(joint_config, frame);
  if (rc != 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Something went wrong. rc value: " << rc);
  }
  return frame;
}

KDL::JntSpaceInertiaMatrix KdlWrapper::dynamics_inertia(std::string mech_unit, KDL::JntArray &q)
{
  if (!mech_unit.compare("right_arm"))
  {
    dynamics_solver_right_->JntToMass(q, inertia_right_);
    return inertia_right_;
  }
  else if (!mech_unit.compare("left_arm"))
  {
    dynamics_solver_left_->JntToMass(q, inertia_left_);
    return inertia_left_;
  }
}

KDL::JntSpaceInertiaMatrix KdlWrapper::dynamics_inertia(std::string mech_unit, std::vector<float> &q)
{
  KDL::JntArray q_kdl = stdvec_to_jntarray(q);
  return dynamics_inertia(mech_unit, q_kdl);
}

KDL::JntArray KdlWrapper::dynamics_coriolis(std::string mech_unit, KDL::JntArray &q, KDL::JntArray &q_dot)
{
  if (!mech_unit.compare("right_arm"))
  {
    dynamics_solver_right_->JntToCoriolis(q, q_dot, coriolis_right_);
    return coriolis_right_;
  }
  else if (!mech_unit.compare("left_arm"))
  {
    dynamics_solver_left_->JntToCoriolis(q, q_dot, coriolis_left_);
    return coriolis_left_;
  }
}

KDL::JntArray KdlWrapper::dynamics_coriolis(std::string mech_unit, std::vector<float> &q, std::vector<float> &q_dot)
{
  KDL::JntArray q_kdl = stdvec_to_jntarray(q);
  KDL::JntArray q_dot_kdl = stdvec_to_jntarray(q_dot);
  return dynamics_coriolis(mech_unit, q_kdl, q_dot_kdl);
}

KDL::JntArray KdlWrapper::dynamics_gravity(std::string mech_unit, KDL::JntArray &q)
{
  if (!mech_unit.compare("right_arm"))
  {
    dynamics_solver_right_->JntToGravity(q, gravity_right_);
    return gravity_right_;
  }
  else if (!mech_unit.compare("left_arm"))
  {
    dynamics_solver_left_->JntToGravity(q, gravity_left_);
    return gravity_left_;
  }
}

KDL::JntArray KdlWrapper::dynamics_gravity(std::string mech_unit, std::vector<float> &q)
{
  KDL::JntArray q_kdl = stdvec_to_jntarray(q);
  return dynamics_gravity(mech_unit, q_kdl);
}

KDL::Jacobian KdlWrapper::calculate_jacobian(std::string mech_unit, KDL::JntArray &q)
{
  if(!mech_unit.compare("right_arm"))
  {
    jacobian_solver_right_->JntToJac(q, jacobian_right_);
    return jacobian_right_;
  }
  else if(!mech_unit.compare("left_arm"))
  {
    jacobian_solver_left_->JntToJac(q, jacobian_left_);
    return jacobian_left_;
  }
}

KDL::Jacobian KdlWrapper::calculate_jacobian(std::string mech_unit, std::vector<float> &q)
{
  KDL::JntArray q_kdl = stdvec_to_jntarray(q);
  return calculate_jacobian(mech_unit, q);
}

KDL::Chain KdlWrapper::get_right_arm()
{
  return right_arm_;
}

KDL::Chain KdlWrapper::get_left_arm()
{
  return left_arm_;
}

KDL::JntArray KdlWrapper::stdvec_to_jntarray(std::vector<float> &vec)
{
  KDL::JntArray vec_kdl(vec.size());
  for (int i = 0; i < vec_kdl.rows(); ++i)
    vec_kdl(i) = vec[i];
  return vec_kdl;
}

void KdlWrapper::print_info()
{
  std::cout << "Number of joints on right arm: " << right_arm_.getNrOfJoints() << std::endl;
  std::cout << "Number of segments on right arm: " << right_arm_.getNrOfSegments() << std::endl;

  std::cout << "Number of joints on left arm: " << left_arm_.getNrOfJoints() << std::endl;
  std::cout << "Number of segments on left arm: " << left_arm_.getNrOfSegments() << std::endl;
}

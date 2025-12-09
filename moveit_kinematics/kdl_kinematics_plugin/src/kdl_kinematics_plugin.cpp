/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp>
#include <moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp>
#include <moveit/utils/logger.hpp>

#include <tf2_kdl/tf2_kdl.hpp>
// TODO: Remove conditional include when released to all active distros.
#if __has_include(<tf2/transform_datatypes.hpp>)
#include <tf2/transform_datatypes.hpp>
#else
#include <tf2/transform_datatypes.h>
#endif

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <algorithm>
#include <cfloat>

namespace kdl_kinematics_plugin
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.kinematics.kdl_kinematics_plugin");
}
}  // namespace

static rclcpp::Clock steady_clock = rclcpp::Clock(RCL_ROS_TIME);

KDLKinematicsPlugin::KDLKinematicsPlugin() : initialized_(false)
{
}

void KDLKinematicsPlugin::getRandomConfiguration(Eigen::VectorXd& jnt_array) const
{
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array[0]);
}

void KDLKinematicsPlugin::getRandomDeltaQ(Eigen::VectorXd& delta_q) const
{
  // **the code below has bug**: it can violate mimic joint constraints
  // auto delta_q_active = Eigen::VectorXd::Random(active_dimension_);
  // **Correct implementation** :
  auto& rng = state_->getRandomNumberGenerator();
  Eigen::VectorXd delta_q_active(active_dimension_);
  for (auto& dq : delta_q_active)
    dq = rng.uniformReal(-1.0, 1.0);

  for (std::size_t i = 0; i < mimic_joints_.size(); i++)
  {
    // for delta_q, mimic offset is cancelled out
    delta_q(i) = mimic_joints_[i].multiplier * delta_q_active(mimic_joints_[i].map_index);
  }
}

void KDLKinematicsPlugin::getRandomConfiguration(const Eigen::VectorXd& seed_state,
                                                 const std::vector<double>& consistency_limits,
                                                 Eigen::VectorXd& jnt_array) const
{
  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), &jnt_array[0],
                                                       &seed_state[0], consistency_limits);
}

bool KDLKinematicsPlugin::checkConsistency(const Eigen::VectorXd& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const Eigen::VectorXd& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
      return false;
  }
  return true;
}

void KDLKinematicsPlugin::getJointWeights()
{
  const std::vector<std::string> joint_names = joint_model_group_->getActiveJointModelNames();
  // Default all joint weights to 1.0
  joint_weights_ = std::vector<double>(joint_names.size(), 1.0);

  // Check if joint weight is assigned in kinematics YAML
  // Loop through map (key: joint name and value: Struct with a weight member variable)
  for (const auto& joint_weight : params_.joints_map)
  {
    // Check if joint is an active joint in the group
    const auto joint_name = joint_weight.first;
    auto it = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (it == joint_names.cend())
    {
      RCLCPP_WARN(getLogger(), "Joint '%s' is not an active joint in group '%s'", joint_name.c_str(),
                  joint_model_group_->getName().c_str());
      continue;
    }

    // Find index of the joint name and assign weight to the coressponding index
    joint_weights_.at(it - joint_names.begin()) = joint_weight.second.weight;
  }

  RCLCPP_INFO_STREAM(
      getLogger(), "Joint weights for group '"
                       << getGroupName() << "': "
                       << Eigen::Map<const Eigen::VectorXd>(joint_weights_.data(), joint_weights_.size()).transpose());
}

bool KDLKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                     const std::string& group_name, const std::string& base_frame,
                                     const std::vector<std::string>& tip_frames, double search_discretization)
{
  node_ = node;

  // Get Solver Parameters
  std::string kinematics_param_prefix = "robot_description_kinematics." + group_name;
  param_listener_ = std::make_shared<kdl_kinematics::ParamListener>(node, kinematics_param_prefix);
  params_ = param_listener_->get_params();

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (!joint_model_group_->isChain())
  {
    RCLCPP_ERROR(getLogger(), "Group '%s' is not a chain", group_name.c_str());
    return false;
  }
  if (!joint_model_group_->isSingleDOFJoints())
  {
    RCLCPP_ERROR(getLogger(), "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  KDL::Tree kdl_tree;

  if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
  {
    RCLCPP_ERROR(getLogger(), "Could not initialize tree object");
    return false;
  }
  if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
  {
    RCLCPP_ERROR(getLogger(), "Could not initialize chain object");
    return false;
  }

  *const_cast<unsigned int*>(&active_dimension_) = joint_model_group_->getActiveJointModels().size();
  *const_cast<unsigned int*>(&dimension_) = active_dimension_ + joint_model_group_->getMimicJointModels().size();
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    const moveit::core::JointModel* jm = joint_model_group_->getJointModels()[i]; 
    if (jm->getType() == moveit::core::JointModel::REVOLUTE ||
        jm->getType() == moveit::core::JointModel::PRISMATIC)
    {
      solver_info_.joint_names.push_back(jm->getName());
      /** if the joint is mimic, enforce joint limits that are consistent with the parent's limits
       * otherwise, `clipToJointLimits` can produce joint values that violate the mimic joint constraints
       */
      if (jm->getMimic())
      {
        double mult = jm->getMimicFactor();
        double offset = jm->getMimicOffset();
        // 1-DoF joints (revolute or prismatic) so getVariableBoundsMsg().size() == 1
        moveit_msgs::msg::JointLimits limits = jm->getMimic()->getVariableBoundsMsg()[0];
        limits.min_position = mult * limits.min_position + offset; 
        limits.max_position = mult * limits.max_position + offset;
        if (limits.min_position > limits.max_position)
          std::swap(limits.min_position, limits.max_position);
        limits.max_velocity = std::fabs(mult) * limits.max_velocity;
        limits.max_acceleration = std::fabs(mult) * limits.max_acceleration;
        limits.max_jerk = std::fabs(mult) * limits.max_jerk;
        solver_info_.limits.emplace_back(limits);
      }
      else // the joint is active so get the limits directly
      {
        solver_info_.limits.emplace_back(jm->getVariableBoundsMsg()[0]);
      }
    }
  }

  if (!joint_model_group_->hasLinkModel(getTipFrame()))
  {
    RCLCPP_ERROR(getLogger(), "Could not find tip name in joint group '%s'", group_name.c_str());
    return false;
  }
  solver_info_.link_names.push_back(getTipFrame());

  joint_min_.resize(solver_info_.limits.size());
  joint_max_.resize(solver_info_.limits.size());

  for (unsigned int i = 0; i < solver_info_.limits.size(); ++i)
  {
    joint_min_(i) = solver_info_.limits[i].min_position;
    joint_max_(i) = solver_info_.limits[i].max_position;
  }

  getJointWeights();

  // Check for mimic joints
  unsigned int active_joint_idx = -1; // the index of an active joint in the active joints vector
  unsigned int joint_idx = -1; // the index of a joint in the (active + mimic) joints vector
  for (std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
  {
    const std::string joint_name = kdl_chain_.segments[i].getJoint().getName();
    const moveit::core::JointModel* jm = robot_model_->getJointModel(joint_name);
    
    // skip fixed joints
    if (jm->getVariableCount() == 0) continue;

    const moveit::core::JointModel* mimic_parent_jm = jm->getMimic();
  
    ++joint_idx;

    // check if it is an active joint
    if (mimic_parent_jm == nullptr)
    {
      ++active_joint_idx;
      JointMimic mimic_joint;
      mimic_joint.reset(active_joint_idx, joint_idx);
      mimic_joint.joint_name = joint_name;
      mimic_joint.active = true;
      mimic_joints_.push_back(mimic_joint);
      continue;
    }
    // check that the joint belongs to the joint model group
    if (joint_model_group_->hasJointModel(joint_name))
    {
      // check that the joint has a parent that is mimicking, and that the parent is part of the joint model group
      if (mimic_parent_jm && joint_model_group_->hasJointModel(mimic_parent_jm->getName()))
      {
        JointMimic mimic_joint;
        mimic_joint.joint_name = joint_name;
        mimic_joint.offset = jm->getMimicOffset();
        mimic_joint.multiplier = jm->getMimicFactor();
        mimic_joints_.push_back(mimic_joint);
        continue;
      }
    }
  }

  // for each mimic joint find the index of the parent joint that is being mimicked
  for (JointMimic& mimic_joint : mimic_joints_)
  {
    if (!mimic_joint.active)
    {
      const std::string mimic_parent_joint_name = 
        joint_model_group_->getJointModel(mimic_joint.joint_name)->getMimic()->getName();
      // find the parent joint
      for (JointMimic& mimic_joint_recall : mimic_joints_)
      {
        if (mimic_joint_recall.joint_name == mimic_parent_joint_name)
        {
          // set the index of this mimic joint to be the parent index
          mimic_joint.map_index = mimic_joint_recall.map_index;
          mimic_joint.index = mimic_joint_recall.index;
        }
      }
    }
  }

  // Setup the joint state groups that we need
  state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  initialized_ = true;
  RCLCPP_DEBUG(getLogger(), "KDL solver initialized");
  return true;
}

bool KDLKinematicsPlugin::timedOut(const rclcpp::Time& start_time, double duration) const
{
  return ((steady_clock.now() - start_time).seconds() >= duration);
}

bool KDLKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  // limit search to a single attempt by setting a timeout of zero
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool KDLKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  const rclcpp::Time start_time = steady_clock.now();
  if (!initialized_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics solver not initialized");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR(getLogger(), "Seed state must have size %d instead of size %zu\n", dimension_, ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Resize consistency limits to remove mimic joints
  std::vector<double> consistency_limits_active;
  if (!consistency_limits.empty())
  {
    if (consistency_limits.size() != dimension_)
    {
      RCLCPP_ERROR(getLogger(), "Consistency limits must be empty or have size %d instead of size %zu\n", dimension_,
                   consistency_limits.size());
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    for (std::size_t i = 0; i < dimension_; ++i)
    {
      if (mimic_joints_[i].active)
        consistency_limits_active.push_back(consistency_limits[i]);
    }
  }

  auto orientation_vs_position_weight = params_.position_only_ik ? 0.0 : params_.orientation_vs_position;
  if (orientation_vs_position_weight == 0.0)
    RCLCPP_INFO(getLogger(), "Using position only ik");
  
  Eigen::Matrix<double, 6, 1> cartesian_weights;
  cartesian_weights.topRows<3>().setConstant(1.0);
  cartesian_weights.bottomRows<3>().setConstant(orientation_vs_position_weight);

  KDL::JntArray jnt_seed_state(dimension_);
  KDL::JntArray jnt_pos_in(dimension_);
  KDL::JntArray jnt_pos_out(dimension_);
  jnt_seed_state.data = Eigen::Map<const Eigen::VectorXd>(ik_seed_state.data(), ik_seed_state.size());
  jnt_pos_in = jnt_seed_state;

  KDL::ChainIkSolverVelMimicSVD ik_solver_vel(kdl_chain_, mimic_joints_, orientation_vs_position_weight == 0.0);
  solution.resize(dimension_);

  KDL::Frame pose_desired;
  tf2::fromMsg(ik_pose, pose_desired);

  RCLCPP_DEBUG_STREAM(getLogger(), "searchPositionIK: Position request pose is "
                                       << ik_pose.position.x << ' ' << ik_pose.position.y << ' ' << ik_pose.position.z
                                       << ' ' << ik_pose.orientation.x << ' ' << ik_pose.orientation.y << ' '
                                       << ik_pose.orientation.z << ' ' << ik_pose.orientation.w);

  auto calc_solution_cost = [this, &pose_desired](const KDL::JntArray& jnt_pos) -> double
  {
    KDL::Frame f;
    fk_solver_->JntToCart(jnt_pos, f);
    KDL::Twist delta_twist = KDL::diff(pose_desired, f);
    return delta_twist.vel.Norm() + delta_twist.rot.Norm(); 
  };

  KDL::JntArray best_jnt_pos_out(jnt_seed_state);
  double best_cost = calc_solution_cost(best_jnt_pos_out);

  unsigned int attempt = 0;
  bool found_any_solution = false;
  bool found_accurate_solution = false;
  do
  {
    ++attempt;
    if (attempt > 1)  // randomly re-seed after first attempt
    {
      if (!consistency_limits_active.empty())
      {
        getRandomConfiguration(jnt_seed_state.data, consistency_limits_active, jnt_pos_in.data);
      }
      else
      {
        getRandomConfiguration(jnt_pos_in.data);
      }
      RCLCPP_DEBUG_STREAM(getLogger(), "New random configuration (" << attempt << "): " << jnt_pos_in);
    }

    int ik_valid =
        CartToJnt(ik_solver_vel, jnt_pos_in, pose_desired, jnt_pos_out, params_.max_solver_iterations,
      Eigen::Map<const Eigen::VectorXd>(joint_weights_.data(), joint_weights_.size()), cartesian_weights);
    if (ik_valid == 0 || options.return_approximate_solution)  // found acceptable solution
    {
      if (!consistency_limits_active.empty() &&
          !checkConsistency(jnt_seed_state.data, consistency_limits_active, jnt_pos_out.data))
        continue;
      
      found_any_solution = true;

      RCLCPP_DEBUG_STREAM(getLogger(), 
        "IK finished: elapse time = " << (steady_clock.now() - start_time).seconds() << " (timeout = " << timeout << ") sec"
        << ", attempt = " << attempt
        << ", is_valid = " << (ik_valid == 0 ? "true" : "false")
        << ", approximate_solution = " << (ik_valid != 0 && options.return_approximate_solution ? "true" : "false")
      );

      // get the best solution so far
      double cost = calc_solution_cost(jnt_pos_out);
      if (cost < best_cost)
      {
        best_jnt_pos_out = jnt_pos_out;
        cost = best_cost;
      }

      // found accurate solution, so exit
      if (ik_valid == 0)
      {
        found_accurate_solution = true;
        break;
      }
    }

  } while (!timedOut(start_time, timeout));

  if (found_any_solution)
  {
    Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size()) = best_jnt_pos_out.data;
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      // if (error_code.val != error_code.SUCCESS)
      //   continue;
    }
  }

  RCLCPP_DEBUG_STREAM(getLogger(), "IK finished after " 
    << (steady_clock.now() - start_time).seconds() << "sec and " << attempt << " attempts");

  if (found_accurate_solution)
  {
    error_code.val = error_code.SUCCESS;
    RCLCPP_DEBUG_STREAM(getLogger(), "Found accurate solution!");
    return true;
  }
  else if (found_any_solution)
  {
    error_code.val = error_code.SUCCESS;
    RCLCPP_DEBUG_STREAM(getLogger(), "Found approximate solution.");
    return true;
  }
  
  error_code.val = error_code.TIMED_OUT;
  RCLCPP_DEBUG_STREAM(getLogger(), "Timeout, no solution found.");

  return false;
}

// NOLINTNEXTLINE(readability-identifier-naming)
int KDLKinematicsPlugin::CartToJnt(KDL::ChainIkSolverVelMimicSVD& ik_solver, const KDL::JntArray& q_init,
                                   const KDL::Frame& p_in, KDL::JntArray& q_out, const unsigned int max_iter,
                                   const Eigen::VectorXd& joint_weights, const Twist& cartesian_weights) const
{
  auto t0 = std::chrono::steady_clock::now();

  q_out = q_init;

  KDL::Frame f; // current pose solution (corresponding to `q_out`)
  KDL::Twist delta_twist; // the error between the current pose solution `f` and the the goal pose `p_in`
  double last_delta_twist_norm = DBL_MAX;
  KDL::JntArray delta_q(q_out.rows()); // joint diff to apply to the current joint solution `q_out`
  double step_size = 1.0; // step size applied to delta_q to get new q_out
  KDL::JntArray q_backup(q_out.rows()); // joint values of last successful step
  Eigen::ArrayXd extra_joint_weights(joint_weights.rows()); // used to weight down joints that at the current solution step are close to their limits
  extra_joint_weights.setOnes();

  RCLCPP_DEBUG_STREAM(getLogger(), "Input: " << q_init);

  unsigned int iter;
  bool success = false;
  bool stuck_in_singularity = false;
  for (iter = 0; iter < max_iter; ++iter)
  {
    fk_solver_->JntToCart(q_out, f);
    delta_twist = KDL::diff(f, p_in);
    RCLCPP_DEBUG_STREAM(getLogger(), "[" << std::setw(3) << iter << "] delta_twist: " << delta_twist);

    // check norms of position and orientation errors
    const double position_error = delta_twist.vel.Norm();
    const double orientation_error = ik_solver.isPositionOnly() ? 0 : delta_twist.rot.Norm();
    const double delta_twist_norm = std::max(position_error, orientation_error);
    if (delta_twist_norm <= params_.epsilon)
    {
      success = true;
      break;
    }

    if (delta_twist_norm >= last_delta_twist_norm)
    {
      // if the error increased, we are close to a singularity -> reduce step size
      double old_step_size = step_size;
      step_size *= std::min(0.2, last_delta_twist_norm / delta_twist_norm);  // reduce scale;
      // delta_q does not have mimic offset, so we can multiply directly
      KDL::Multiply(delta_q, step_size / old_step_size, delta_q);
      RCLCPP_DEBUG(getLogger(), "      error increased: %f -> %f, scale: %f", last_delta_twist_norm, delta_twist_norm,
                   step_size);
      q_out = q_backup;  // restore previous unclipped joint values
    }
    else
    {
      q_backup = q_out;  // remember joint values of last successful step
      step_size = 1.0;   // reset step size
      last_delta_twist_norm = delta_twist_norm;

      ik_solver.CartToJnt(q_out, delta_twist, delta_q, extra_joint_weights * joint_weights.array(), cartesian_weights);
    }

    Eigen::VectorXd delta_q0 = delta_q.data; // unclipped values for debug-print purposes

    clipToJointLimits(q_out, delta_q, extra_joint_weights);

    const double delta_q_norm = delta_q.data.lpNorm<1>();
    RCLCPP_DEBUG(getLogger(), "[%3d] pos err: %f  rot err: %f  delta_q: %f", iter, position_error, orientation_error,
                 delta_q_norm);
    if (delta_q_norm < params_.epsilon)  // stuck in singularity
    {
      if (step_size < params_.epsilon)  // cannot reach target
      {
        stuck_in_singularity = true;
        break;
      }
      // wiggle joints
      last_delta_twist_norm = DBL_MAX;
      getRandomDeltaQ(delta_q.data);
      delta_q.data *= std::min(0.1, delta_twist_norm);
      clipToJointLimits(q_out, delta_q, extra_joint_weights);
      extra_joint_weights.setOnes();
    }

    KDL::Add(q_out, delta_q, q_out);

    RCLCPP_DEBUG_STREAM(getLogger(), "      delta_q: " << delta_q);
    RCLCPP_DEBUG_STREAM(getLogger(), "      q: " << q_out);
  }

  double elaps_time = 1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  int result;
  std::string result_msg;
  if (stuck_in_singularity)
  {
    result = -4;
    result_msg = "STUCK_IN_SINGULARITY";
  }
  else if (iter == max_iter)
  {
    result = -3;
    result_msg = "MAX_ITERATIONS";
  } 
  else
  {
    result = success ? 0 : -2;
    result_msg = success ? "SUCCESS" : "FAIL";
  }

  RCLCPP_DEBUG_STREAM(getLogger(), "Result: "<< result_msg 
    << ", elapsed_time: " << elaps_time << " ms"
    << ", iterations:" << iter
    << ", solution: " << q_out);

  return result;
}

void KDLKinematicsPlugin::clipToJointLimits(const KDL::JntArray& q, KDL::JntArray& q_delta,
                                            Eigen::ArrayXd& weighting) const
{
  weighting.setOnes();
  for (std::size_t i = 0; i < q.rows(); ++i)
  {
    const double delta_max = joint_max_(i) - q(i);
    const double delta_min = joint_min_(i) - q(i);
    if (q_delta(i) > delta_max)
    {
      q_delta(i) = delta_max;
    }
    else if (q_delta(i) < delta_min)
    {
      q_delta(i) = delta_min;
    }
    else
    {
      continue;
    }

    weighting[mimic_joints_[i].map_index] = 0.01;
  }
}

bool KDLKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!initialized_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics solver not initialized");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(getLogger(), "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in(dimension_);
  jnt_pos_in.data = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); ++i)
  {
    if (fk_solver_->JntToCart(jnt_pos_in, p_out) >= 0)
    {
      poses[i] = tf2::toMsg(p_out);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& KDLKinematicsPlugin::getJointNames() const
{
  return solver_info_.joint_names;
}

const std::vector<std::string>& KDLKinematicsPlugin::getLinkNames() const
{
  return solver_info_.link_names;
}

}  // namespace kdl_kinematics_plugin

// register KDLKinematics as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(kdl_kinematics_plugin::KDLKinematicsPlugin, kinematics::KinematicsBase)

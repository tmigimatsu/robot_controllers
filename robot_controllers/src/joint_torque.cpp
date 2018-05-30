/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Stanford Robotics Lab
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Unbounded Robotics nor the names of its
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

/*
 * Derived from fetch_robotcs/robot_controllers/cartesian_wrench_controller.cpp
 * Author: Toki Migimatsu
 */

#include <pluginlib/class_list_macros.h>
#include <robot_controllers/joint_torque.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <time.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::JointTorqueController, robot_controllers::Controller)

namespace robot_controllers
{

JointTorqueController::JointTorqueController() :
    initialized_(false)
{
}

int JointTorqueController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Initialize KDL structures
  std::string tip_link;
  nh.param<std::string>("root_name", root_link_, "torso_lift_link");
  nh.param<std::string>("tip_name", tip_link, "gripper_link");

  // Load URDF
  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Could not construct tree from URDF");
    return -1;
  }

  // Populate the chain
  if(!kdl_tree.getChain(root_link_, tip_link, kdl_chain_))
  {
    ROS_ERROR("Could not construct chain from URDF");
    return -1;
  }

  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));
  dof_ = kdl_chain_.getNrOfJoints();
  desired_torque_.resize(dof_);
  gravity_torque_.resize(dof_);
  kdl_q_.resize(dof_);
  kdl_dq_.resize(dof_);
  inertia_.resize(dof_);
  clock_gettime(CLOCK_MONOTONIC, &t_start_);

  q_ = Eigen::VectorXd::Zero(dof_);
  dq_ = Eigen::VectorXd::Zero(dof_);
  ddq_ = Eigen::VectorXd::Zero(dof_);
  dt_ = 0.001;

  // Init joint handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

  // Subscribe to command
  command_sub_ = nh.subscribe<std_msgs::Float64MultiArray>("command", 1,
					boost::bind(&JointTorqueController::command, this, _1));
  last_command_ = ros::Time(0);

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool JointTorqueController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("JointTorqueController",
                    "Unable to start, not initialized.");
    return false;
  }

  if (ros::Time::now() - last_command_ > ros::Duration(3.0))
  {
    ROS_ERROR_NAMED("JointTorqueController",
                    "Unable to start, no goal.");
    return false;
  }

  return true;
}

bool JointTorqueController::stop(bool force)
{
  return true;
}

bool JointTorqueController::reset()
{
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

void JointTorqueController::update(const ros::Time& now, const ros::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_) {
    SetToZero(desired_torque_);
    return;
  }

  if (ros::Time::now() - last_command_ > ros::Duration(0.1))
  {
    // Command has timed out, shutdown
    SetToZero(desired_torque_);
    // manager_->requestStop(getName());
    // return;
  } 

  // Get current positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
    kdl_q_(i) = joints_[i]->getPosition();
    kdl_dq_(i) = joints_[i]->getVelocity();
  }
  q_ = kdl_q_.data;

  // Do the gravity compensation
  kdl_chain_dynamics_->JntToGravity(kdl_q_, gravity_torque_);

  // Do forward dynamics
  //Eigen::VectorXd G(joints_.size());
  //for (size_t i = 0; i < joints_.size(); ++i) {
    //G(i) = gravity_torque_(i);
  //}

  kdl_chain_dynamics_->JntToMass(kdl_q_, inertia_);
  //for (size_t i = 0; i < inertia_.rows(); ++i) {
    //for (size_t j = 0; j < inertia_.cols(); ++j) {
      //inertia_(i,j) = 0;
      //A(i,j) = inertia_(i,j);
    //}
  //}

  //Eigen::VectorXd tau(joints_.size());
  //for (size_t i = 0; i < joints_.size(); ++i) {
    //tau(i) = desired_torque_(i);
  //}
  for (size_t i = 0; i * dt_ < 0.005; i++) {
  Eigen::VectorXd q = q_ + dq_ * dt_;
  Eigen::VectorXd dq = dq_ + ddq_ * dt_;

  Eigen::VectorXd ddq = inertia_.data.llt().solve(desired_torque_.data);
  q_ += 0.5 * (dq_ + dq) * dt_;
  dq_ += 0.5 * (ddq_ + ddq) * dt_;

  kdl_q_.data = q_;
  kdl_dq_.data = dq_;
  kdl_chain_dynamics_->JntToMass(kdl_q_, inertia_);
  ddq_ = inertia_.data.llt().solve(desired_torque_.data);
  }
  
  ROS_WARN("%g %g %g %g %g %g %g; %g", q_(0), q_(1), q_(2), q_(3), q_(4), q_(5), q_(6), dt.toSec());
  //Eigen::VectorXd dq(joints_.size());
  //Eigen::VectorXd q(joints_.size());
  //for (size_t i = 0; i < joints_.size(); ++i) {
    //dq(i) = dq_(i) + ddq(i) * dt.toSec();
    //q(i) = q_(i) + 0.5 * (dq_(i) + dq(i)) * dt.toSec();
  //}
  //ROS_WARN("%d %d %d %d", inertia_.rows(), inertia_.columns(), desired_torque_.rows(), desired_torque_.columns());
//std::cout << q.transpose() << std::endl;
  //Eigen::VectorXd q = q_ + dq_ * dt; 
  //q = q_ + 0.5 * (dq_ + dq) * dt;

  //q = q_ + 0.5 * (dq_ + dq) * dt;
  //dq = dq_ + 0.5 * (ddq_ + ddq) * dt;

  //kdl_chain_dynamics_->JntToMass(q, inertia_);
  //for (size_t i = 0; i < joints_.size(); ++i) {
    //for (size_t j = 0; j < joints_.size(); ++j) {
      //A(i,j) = inertia_(i,j);
    //}
  //}
  //ddq_ = A.ldlt().solve(desired_torque_(j));

  // Actually update joints
  for (size_t j = 0; j < joints_.size(); ++j) {
    // Add dither for friction compensation
    //float t_curr = ros::Time::now().toSec();
    //joints_[j]->setVelocity(dq_(j) + 0.1 * sin(2*M_PI/5*t_curr), gravity_torque_(j) + desired_torque_(j));
    //joints_[j]->setVelocity(desired_torque_(j), gravity_torque_(j));

    // Set torque including gravity compensation
    //if (j == 7) {
    //    timespec t_now;
    //    clock_gettime(CLOCK_MONOTONIC, &t_now);
    //    timespec dt;
    //    if (t_now.tv_nsec - t_start_.tv_nsec < 0) {
    //        dt.tv_sec = t_now.tv_sec - t_start_.tv_sec - 1;
    //        dt.tv_nsec = t_now.tv_nsec - t_start_.tv_nsec + 1e9;
    //    } else {
    //        dt.tv_sec = t_now.tv_sec - t_start_.tv_sec;
    //        dt.tv_nsec = t_now.tv_nsec - t_start_.tv_nsec;
    //    }
    //    double t_curr = dt.tv_sec + 1e-9 * double(dt.tv_nsec);
    //    joints_[j]->setEffort(gravity_torque_(j) + desired_torque_(j) + 1. * sin(2*M_PI*1000*t_curr));
	//ROS_WARN("%f %f %f %f", gravity_torque_(j), desired_torque_(j), 1. * sin(2*M_PI*20*t_curr), t_curr);
//}
    //joints_[j]->setEffort(gravity_torque_(j) + desired_torque_(j));
    if (desired_torque_(j) == 0) {
      joints_[j]->setEffort(gravity_torque_(j));
    } else {
      joints_[j]->setPosition(q_(j), 0, gravity_torque_(j));//, dq(j), gravity_torque_(j) + desired_torque_(j));
    }
    // joints_[j]->setEffort(desired_torque_(j));
  }
}

void JointTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr& desired_torque)
{
  // Update command
  if (desired_torque->data.size() != joints_.size()) {
    SetToZero(desired_torque_);
    ROS_ERROR("JointTorqueController: Invalid joint torque dim. Sending zero torques.");
    return;
  }

  for (size_t i = 0; i < joints_.size(); i++) {
	desired_torque_(i) = desired_torque->data[i];
  }

  // Update last command time before trying to start controller
  last_command_ = ros::Time::now();

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    SetToZero(desired_torque_);
    ROS_ERROR("JointTorqueController: Cannot start, blocked by another controller.");
    return;
  }
}

std::vector<std::string> JointTorqueController::getCommandedNames()
{
  std::vector<std::string> names;
  if (initialized_)
  {
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
  }
  return names;
}

std::vector<std::string> JointTorqueController::getClaimedNames()
{
  return getCommandedNames();
}

}  // namespace robot_controllers

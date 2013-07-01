/*********************************************************************
 * Software License Agreement (LGPL License)
 *
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *   
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *
 *********************************************************************/

#include "joint_trajectory_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace reflexxes_effort_controllers {

  JointTrajectoryController::JointTrajectoryController()
    : loop_count_(0),
    decimation_(10),
    max_pos_tolerance_(0.0),
    max_acceleration_(0.5),
    new_reference_(false),
    sampling_resolution_(0.001)
  {}

  JointTrajectoryController::~JointTrajectoryController()
  {
    sub_command_.shutdown();
  }

  bool JointTrajectoryController::init(
      hardware_interface::EffortJointInterface *robot,
      ros::NodeHandle &n)
  {
    // Store nodehandle
    nh_ = n;

    // Initialize joint name from rosparam
    std::string joint_name;
    if (!n.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    // Initialize PID from rosparam
    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(n, "pid")))
      return false;

    // Create state publisher
    controller_state_publisher_.reset(
        new realtime_tools::RealtimePublisher<controllers_msgs::JointControllerState>(n, "state", 1));

    // Create command subscriber
    sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointTrajectoryController::setCommandCB, this);

    return init(robot, joint_name, pid);
  }

  bool JointTrajectoryController::init(
      hardware_interface::EffortJointInterface
      *robot, const std::string &joint_name, const
      control_toolbox::Pid &pid)
  {
    joint_ = robot->getHandle(joint_name);
    pid_controller_ = pid;

    // get urdf info about joint
    urdf::Model urdf;
    // TODO: change this to initString and get urdf param locally
    if (!urdf.initParam("robot_description")){
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }
    joint_urdf_ = urdf.getJoint(joint_name);
    if (!joint_urdf_){
      ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
      return false;
    }

    // Get position tolerance
    if (!nh_.getParam("position_tolerance", max_pos_tolerance_)) {
      ROS_INFO("No position_tolerance specified (namespace: %s), using default.", nh_.getNamespace().c_str());
    }

    if (!nh_.getParam("max_acceleration", max_acceleration_)) {
      ROS_INFO("No max_acceleration specified (namespace: %s), using default.", nh_.getNamespace().c_str());
    }

    if (!nh_.getParam("sampling_resolution", sampling_resolution_)) {
      ROS_INFO("No sampling_resolution specified (namespace: %s), using default.", nh_.getNamespace().c_str());
    }

    // Create trajectory generator
    // TODO: Scale this to multiple joints
    rml_.reset(new ReflexxesAPI(1, sampling_resolution_));
    rml_in_.reset(new RMLPositionInputParameters(1));
    rml_out_.reset(new RMLPositionOutputParameters(1));

    // Get RML parameters from URDF
    rml_in_->MaxVelocityVector->VecData[0] = joint_urdf_->limits->velocity;
    rml_in_->MaxAccelerationVector->VecData[0] = max_acceleration_;
    rml_in_->MaxJerkVector->VecData[0] = 1000;

    rml_flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;

    return true;
  }


  void JointTrajectoryController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
  {
    pid_controller_.setGains(p,i,d,i_max,i_min);
  }

  void JointTrajectoryController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_.getGains(p,i,d,i_max,i_min);
  }

  std::string JointTrajectoryController::getJointName()
  {
    return joint_.getName();
  }

  // Set the joint position command
  void JointTrajectoryController::setCommand(double cmd)
  {
    // the writeFromNonRT can be used in RT, if you have the guarantee that 
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    command_.writeFromNonRT(cmd);
    pid_controller_.reset();
    new_reference_ = true;
  }


  void JointTrajectoryController::starting(const ros::Time& time) 
  {
    command_.initRT(joint_.getPosition());
    pid_controller_.reset();
    new_reference_ = true;
  }


  int JointTrajectoryController::computeTrajectory(
      const ros::Time& time, 
      const ros::Duration& period,
      const double pos_actual,
      const double vel_actual,
      const double pos_ref)
  {
    ROS_DEBUG("Recomputing trajectory...");

    // Store the traj start time
    traj_start_time_ = time;

    // Update RML input parameters
    rml_in_->CurrentPositionVector->VecData[0] = pos_actual;
    rml_in_->CurrentVelocityVector->VecData[0] = vel_actual;
    rml_in_->CurrentAccelerationVector->VecData[0] = 0.0;

    rml_in_->TargetPositionVector->VecData[0] = pos_ref;
    rml_in_->TargetVelocityVector->VecData[0] = 0.0;

    rml_in_->SelectionVector->VecData[0] = true;

    // Compute trajectory
    int rml_result = 0;
    rml_result = rml_->RMLPosition(
        *rml_in_.get(), 
        rml_out_.get(), 
        rml_flags_);

    new_reference_ = false;

    return rml_result;
  }

  void JointTrajectoryController::update(
      const ros::Time& time, 
      const ros::Duration& period)
  {

    // Use this to change the desired point
    double pos_ref = *(command_.readFromRT());

    // Convenience variables
    double pos_actual = joint_.getPosition(),
           vel_actual = joint_.getVelocity();

    int rml_result = 0;

    // Compute RML traj if the error is too large
    if(new_reference_ || 
        std::abs(rml_out_->NewPositionVector->VecData[0] - pos_actual) > max_pos_tolerance_) {
      rml_result = this->computeTrajectory(
          time, period, pos_actual, vel_actual, pos_ref);
    } else {
      // Sample the computed trajectory
      rml_result = rml_->RMLPositionAtAGivenSampleTime(
        (time - traj_start_time_).toSec(),
        rml_out_.get());
    }

    // Convenience variables
    double pos_target = rml_out_->NewPositionVector->VecData[0],
           vel_target = rml_out_->NewVelocityVector->VecData[0];
    double pos_error,
           vel_error;

    // Compute position error
    switch(joint_urdf_->type) {

      // Revolute joint with limits
      case urdf::Joint::REVOLUTE:
        angles::shortest_angular_distance_with_limits(
            pos_actual,
            pos_target, 
            joint_urdf_->limits->lower, 
            joint_urdf_->limits->upper,
            pos_error);
        break;

        // Continuous joint with no limits
      case urdf::Joint::CONTINUOUS:
        pos_error = angles::shortest_angular_distance(
            pos_actual,
            pos_target);
        break;

        // Prismatic joint types
      default:
        pos_error = pos_target - pos_actual;
        break;
    };

    // Compute velocity error 
    vel_error = vel_target - vel_actual;

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error. 
    double commanded_effort = pid_controller_.computeCommand(pos_error, vel_error, period); 

    // Only set a non-zero effort command if the 
    if(rml_result < 0) {
      ROS_ERROR("Reflexxes error code: %d. Setting command to zero.", rml_result);
      commanded_effort = 0.0;
    }

    // Set the command
    joint_.setCommand(commanded_effort);

    // publish state
    if (loop_count_ % decimation_ == 0) {
      if(controller_state_publisher_ && controller_state_publisher_->trylock()) {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = pos_target;
        controller_state_publisher_->msg_.process_value = pos_actual;
        controller_state_publisher_->msg_.process_value_dot = vel_actual;
        controller_state_publisher_->msg_.error = pos_error;
        controller_state_publisher_->msg_.time_step = period.toSec();
        controller_state_publisher_->msg_.command = commanded_effort;

        double dummy;
        getGains(controller_state_publisher_->msg_.p,
            controller_state_publisher_->msg_.i,
            controller_state_publisher_->msg_.d,
            controller_state_publisher_->msg_.i_clamp,
            dummy);
        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;
  }

  void JointTrajectoryController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( 
    reflexxes_effort_controllers::JointTrajectoryController,
    controller_interface::ControllerBase)

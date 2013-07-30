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
#include <trajectory_msgs/JointTrajectory.h>
#include <sstream>

namespace reflexxes_effort_controllers {

  JointTrajectoryController::JointTrajectoryController()
    : loop_count_(0),
      decimation_(10),
      sampling_resolution_(0.001),
      new_reference_traj_(false),
      compute_trajectory_point_(false),
      is_action_(false),
      final_state_reached_(false),
      point_index_(0)
  {}

  JointTrajectoryController::~JointTrajectoryController()
  {
    trajectory_command_sub_.shutdown();
  }


  template<class T>
  std::ostream& operator<< (std::ostream& stream, const RMLVector<T>& rml_vec) {
    stream<<"[ ";
    for(int i=0; i<rml_vec.VectorDimension; i++) { stream<<(rml_vec.VecData[i])<<", "; }
    stream<<"]";
    return stream;
  }

  void JointTrajectoryController::rml_debug(const ros::console::levels::Level level) {
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT NumberOfDOFs: "<<rml_in_->NumberOfDOFs);
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT MinimumSynchronizationTime: "<<rml_in_->MinimumSynchronizationTime);
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT SelectionVector: "<<(*rml_in_->SelectionVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT CurrentPositionVector: "<<(*rml_in_->CurrentPositionVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT CurrentVelocityVector: "<<(*rml_in_->CurrentVelocityVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT CurrentAccelerationVector: "<<(*rml_in_->CurrentAccelerationVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT MaxAccelerationVector: "<<(*rml_in_->MaxAccelerationVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT MaxJerkVector: "<<(*rml_in_->MaxJerkVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT TargetVelocityVector: "<<(*rml_in_->TargetVelocityVector));

    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT MaxVelocityVector: "<<(*rml_in_->MaxVelocityVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT TargetPositionVector: "<<(*rml_in_->TargetPositionVector));
    ROS_LOG_STREAM(level,ROSCONSOLE_DEFAULT_NAME,"RML INPUT AlternativeTargetVelocityVector: "<<(*rml_in_->AlternativeTargetVelocityVector));
  }

  bool JointTrajectoryController::checkRMLValidity()
  {
    if(rml_in_->CheckForValidity()) {
      ROS_INFO_STREAM("RML INPUT Configuration Valid.");
      this->rml_debug(ros::console::levels::Debug);
    } else {
      ROS_ERROR_STREAM("RML INPUT Configuration Invalid!");
      this->rml_debug(ros::console::levels::Warn);
      return false;
    }
    return true;
  }

  bool JointTrajectoryController::init(
      hardware_interface::EffortJointInterface *robot,
      ros::NodeHandle &n)
  {
    // Store nodehandle
    nh_ = n;

    // Get joint names
    XmlRpc::XmlRpcValue xml_array;
    if( !nh_.getParam("joint_names", xml_array) ) {
      ROS_ERROR("No 'joint_names' parameter in controller (namespace '%s')", nh_.getNamespace().c_str());
      return false;
    }
    // Make sure it's an array type
    if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("The 'joint_names' parameter is not an array (namespace '%s')",nh_.getNamespace().c_str());
      return false;
    }

    // Get number of joints
    n_joints_ = xml_array.size();

    ROS_INFO_STREAM("Initializing JointTrajectoryController with "<<n_joints_<<" joints.");

    // Get trajectory sampling resolution (cycle times in seconds)
    if (!nh_.hasParam("sampling_resolution")) {
      ROS_INFO("No sampling_resolution specified (namespace: %s), using default.", nh_.getNamespace().c_str());
    }
    nh_.param("sampling_resolution", sampling_resolution_, 0.001);

    // Create trajectory generator
    rml_.reset(new ReflexxesAPI(n_joints_, sampling_resolution_));
    rml_in_.reset(new RMLPositionInputParameters(n_joints_));
    rml_out_.reset(new RMLPositionOutputParameters(n_joints_));

    // Get urdf
    urdf::Model urdf;
    std::string urdf_str;
    ros::NodeHandle nh;
    nh.getParam("robot_description", urdf_str);

    if (!urdf.initString(urdf_str)){
      ROS_ERROR("Failed to parse urdf from 'robot_description' parameter (namespace: %s)",nh.getNamespace().c_str());
      return false;
    }

    // Get individual joint properties from urdf and parameter server
    joint_names_.resize(n_joints_);
    joints_.resize(n_joints_);
    urdf_joints_.resize(n_joints_);
    position_tolerances_.resize(n_joints_);
    max_accelerations_.resize(n_joints_);
    max_jerks_.resize(n_joints_);
    position_controllers_.resize(n_joints_);

    for(int i=0; i<n_joints_; i++) 
    {
      // Get joint name
      if(xml_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("The 'joint_names' parameter contains a non-string element (namespace '%s')",nh_.getNamespace().c_str());
        return false;
      }
      joint_names_[i] = static_cast<std::string>(xml_array[i]);

      // Get the joint-namespace nodehandle
      {
        ros::NodeHandle joint_nh(nh_, "joints/"+joint_names_[i]);
        ROS_INFO("Loading joint info for '%s', Namespace: %s", joint_names_[i].c_str(), joint_nh.getNamespace().c_str());

        position_controllers_[i].reset(new effort_controllers::JointPositionController());
        position_controllers_[i]->init(robot, joint_nh);

        // Get position tolerance
        if (!joint_nh.hasParam("position_tolerance")) {
          ROS_INFO("No position_tolerance specified (namespace: %s), using default.",
              joint_nh.getNamespace().c_str());
        }
        joint_nh.param("position_tolerance", position_tolerances_[i], 0.1);

        // Get maximum acceleration
        if (!joint_nh.hasParam("max_acceleration")) {
          ROS_INFO("No max_acceleration specified (namespace: %s), using default.",
              joint_nh.getNamespace().c_str());
        }
        joint_nh.param("max_acceleration", max_accelerations_[i], 1.0);
        
        // Get maximum acceleration
        if (!joint_nh.hasParam("max_jerk")) {
          ROS_INFO("No max_jerk specified (namespace: %s), using default.",
              joint_nh.getNamespace().c_str());
        }
        joint_nh.param("max_jerk", max_jerks_[i], 1000.0);

      } // end of joint-namespaces

      // Get ros_control joint handle
      joints_[i] = robot->getHandle(joint_names_[i]);

      // Get urdf joint
      urdf_joints_[i] = urdf.getJoint(joint_names_[i]);
      if (!urdf_joints_[i]) {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return false;
      }
      
      // Get RML parameters from URDF
      rml_in_->MaxVelocityVector->VecData[i] = urdf_joints_[i]->limits->velocity;
      rml_in_->MaxAccelerationVector->VecData[i] = max_accelerations_[i];
      rml_in_->MaxJerkVector->VecData[i] = max_jerks_[i];

    }
    
    for(int i=0; i<n_joints_; i++) {
      rml_in_->SelectionVector->VecData[i] = true;
    }


    if(rml_in_->CheckForValidity()) {
      ROS_INFO_STREAM("RML INPUT Configuration Valid.");
      this->rml_debug(ros::console::levels::Debug);
    } else {
      ROS_ERROR_STREAM("RML INPUT Configuration Invalid!");
      this->rml_debug(ros::console::levels::Warn);
      return false;
    }

    // Create command subscriber
    trajectory_command_sub_ = nh_.subscribe<trajectory_msgs::JointTrajectory>(
      "trajectory_command", 1, &JointTrajectoryController::trajectoryMsgCommandCB, this);

    trajectory_test_command_sub_ = nh_.subscribe<std_msgs::Empty>(
      "test_command", 1, &JointTrajectoryController::trajectoryTestCommandCB, this);

    // Create action server
    action_server_.reset(new FJTAS(nh_, "follow_joint_trajectory",
        boost::bind(&JointTrajectoryController::trajectoryActionCommandCB, this, _1), false));
    action_server_->start();


    return true;
  }



  void JointTrajectoryController::starting(const ros::Time& time) 
  {
    trajectory_msgs::JointTrajectory initial_command;
    trajectory_msgs::JointTrajectoryPoint initial_point;
    for(int i=0; i<n_joints_; i++) {
      initial_point.positions.push_back(joints_[i].getPosition());
      //initial_point.velocities.push_back(joints_[i].getVelocity());
      //initial_point.positions.push_back(0);
      initial_point.velocities.push_back(0);
      initial_point.accelerations.push_back(0.0);
    }
    initial_point.time_from_start = ros::Duration(1.0);
    initial_command.points.push_back(initial_point);
    trajectory_command_buffer_.initRT(initial_command);
    new_reference_traj_ = true;

    ROS_INFO_STREAM_NAMED("temp","STARTING TRAJECTORY CONTROLLERS ----------------------------");
    ROS_INFO_STREAM_NAMED("temp","STARTING TRAJECTORY CONTROLLERS ----------------------------");
    ROS_INFO_STREAM_NAMED("temp","STARTING TRAJECTORY CONTROLLERS ----------------------------");

    const std_msgs::EmptyConstPtr thing;
    trajectoryTestCommandCB(thing);
  }

  void JointTrajectoryController::stopping(const ros::Time& time)
  {
    action_server_->shutdown();
  }

  void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Debug info
    static size_t counter = 0;
    verbose_ = false;
    counter ++;
    if( counter % 100 == 0 )
      verbose_ = true;


    // Check if there is a new trajectory to update
    bool success = updateTrajectory(time,period);

    // Apply joint commands
    for(size_t i=0; i<n_joints_; i++) 
    {
      if( success )
      {
        position_controllers_[i]->setCommand(
          rml_out_->NewPositionVector->VecData[i], 
          rml_out_->NewVelocityVector->VecData[i]);

        if( verbose_ )
          ROS_INFO_STREAM_NAMED("temp","setting command " << 
            rml_out_->NewPositionVector->VecData[i] << " and " <<
            rml_out_->NewVelocityVector->VecData[i] );
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("temp","DOING NOTHING BC ERROR ---------------------------");

        // Freeze the controllers at current position and 0 velocity
        position_controllers_[i]->setCommand(joints_[i].getPosition(), 0.0);          
      }

      // Update the individual joint controllers
      position_controllers_[i]->update(time, period);
    }
  }

  bool JointTrajectoryController::checkNewTrajectory(
    const ros::Time& time, const ros::Duration& period,
    const trajectory_msgs::JointTrajectory &commanded_trajectory)
  {
    // Choose new trajectory start time
    if(commanded_trajectory.header.stamp.isZero()) 
    {
      // Start trajectory immediately if stamp is zero
      commanded_start_time_ = time;
      ROS_WARN_STREAM_NAMED("temp","commanded_start_time = " << commanded_start_time_.toSec() << " (NOW) " );
    } 
    else 
    {
      ROS_WARN_STREAM_NAMED("temp","commanded_start_time = " << commanded_trajectory.header.stamp.toSec() << " from header" );

      // Check provided trajectory start time - expired
      if( commanded_trajectory.header.stamp < time )
      {
        ROS_ERROR_STREAM_NAMED("temp","Trajectory scheduled to start in the past");

        // Set this new trajectory as old so that it is ignored
        new_reference_traj_ = false;

        return false; // the new trajectory is bad, ignore it
      }

      // Check provided trajectory start time - future
      if( commanded_trajectory.header.stamp > time + period )
      {
        ROS_WARN_STREAM_NAMED("temp","Trajectory scheduled to start in the future, waiting...");
        return false; // pretend like we didn't see it
      }

      // Start time is good, save it
      commanded_start_time_ = commanded_trajectory.header.stamp;
    }

    // Test that all trajectory points have increasing time stamps
    // \todo

    // Test that all trajectory points are within joint limits
    // \todo

    // New trajectory passes    
    return true;
  }

  bool JointTrajectoryController::updateTrajectory(const ros::Time& time, const ros::Duration& period)
  {
    // Read the latest commanded trajectory message
    const trajectory_msgs::JointTrajectory &commanded_trajectory = *(trajectory_command_buffer_.readFromRT());

    // Check for a new incoming trajectory message
    if(new_reference_traj_) 
    {
      if(checkNewTrajectory(time, period, commanded_trajectory))
      {
        // Reset point index so that the first point in the new trajectory is used
        point_index_ = 0;

        // Signal a fresh trajectory to be calculated
        compute_trajectory_point_ = true;

        // Set this new trajectory as old
        new_reference_traj_ = false;
      }
    }

    // Check if there are more points in the trajectory to process
    bool trajectory_incomplete = point_index_ < commanded_trajectory.points.size();
    
    // Initialize RML return value
    int rml_result = 0;

    // Compute RML trajectory if there are still points in the queue and we finished the last one
    if(compute_trajectory_point_ && trajectory_incomplete) 
    {
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");
      ROS_WARN_STREAM_NAMED("temp","Computing sub-trajectory, on point " << point_index_ << " of " 
        << commanded_trajectory.points.size() );
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");

      // Get reference to the active trajectory point
      const trajectory_msgs::JointTrajectoryPoint &active_traj_point = commanded_trajectory.points[point_index_];

      //ROS_INFO_STREAM_NAMED("temp","point: \n" << active_traj_point);

      // Update RML input parameters
      for(size_t i=0; i<n_joints_; i++) 
      {
        rml_in_->CurrentPositionVector->VecData[i] = joints_[i].getPosition();
        rml_in_->CurrentVelocityVector->VecData[i] = joints_[i].getVelocity();
        rml_in_->CurrentAccelerationVector->VecData[i] = 0.0;
        rml_in_->TargetPositionVector->VecData[i] = active_traj_point.positions[i];
        rml_in_->TargetVelocityVector->VecData[i] = active_traj_point.velocities[i];
      }

      // Store the traj start time
      traj_point_start_time_ = time;

      // Set desired execution time for this trajectory (definitely > 0)
      double min_sync_time = (active_traj_point.time_from_start - (traj_point_start_time_ - commanded_start_time_)).toSec();
      //double min_sync_time = std::max(0.0,(active_traj_point.time_from_start - (traj_point_start_time_ - commanded_start_time_)).toSec()));

      ROS_DEBUG_STREAM("RML IN: MinimumSynchronizatonTime: " << min_sync_time
        << "\n Traj point time_from_start: " << active_traj_point.time_from_start.toSec()  // the delay until starting this point
        << "\n traj_point_start_time_ (now): " << traj_point_start_time_.toSec()   // now
        << "\n commanded_start_time_: " << commanded_start_time_.toSec() ); // start time of the whole trajectory msg

      if( min_sync_time < 0 )
      {
        ROS_ERROR_STREAM_NAMED("update","Minimum synchronization time was calculated to be less than zero: " << min_sync_time );        

        min_sync_time = 0.0;
      }
      rml_in_->SetMinimumSynchronizationTime(min_sync_time);
                
      // Hold fixed at final point once trajectory is complete
      rml_flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
      rml_flags_.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

      // Ensure that the RML input is valid
      /*
      if(!checkRMLValidity())
        return false;
      */

      // Have Reflexxes Compute trajectory
      rml_result = rml_->RMLPosition( *rml_in_.get(), rml_out_.get(), rml_flags_);

      /*
      ROS_ERROR_STREAM_NAMED("temp","NEW TRAJECTORY --------------------------------------");
      for(size_t i=0; i<n_joints_; i++)
        std::cout << "Joint " << i << " = " << rml_out_->NewPositionVector->VecData[i] << std::endl;
      ROS_ERROR_STREAM_NAMED("temp","NEW TRAJECTORY --------------------------------------");
      */

      // Disable recompute flag
      compute_trajectory_point_ = false;
    } 
    else  // We are still working on a previous trajectory point plan
    {
      
      if( verbose_ )
        ROS_DEBUG_STREAM_NAMED("precomputed", "Sample: "
          << " compute_trajectory_point_="<<compute_trajectory_point_
          << " trajectory_incomplete="<<trajectory_incomplete
          << " Sample time = " << (time - traj_point_start_time_).toSec() );

      // Have Reflexxes Sample the already computed trajectory
      rml_result = rml_->RMLPositionAtAGivenSampleTime(
        (time - traj_point_start_time_).toSec(), rml_out_.get());
    }

    // Check tolerances -------------------------------------------------------------
    for(size_t i=0; i<n_joints_; i++) 
    {
      // Check this *after* first iteration of RML
      if(std::abs(rml_out_->NewPositionVector->VecData[i] - joints_[i].getPosition()) > position_tolerances_[i]) 
      {
        compute_trajectory_point_ = true;

        /*
        if(!verbose_)
          break;
        */

        if(verbose_)
          ROS_WARN_STREAM_NAMED("update","TOLERANCE: " << joint_names_[i] << " target: " 
            << rml_out_->NewPositionVector->VecData[i] 
            << " actual: " << joints_[i].getPosition() << " tolerance limit: " 
            << position_tolerances_[i] );                  
      }
    }

    // Error Check -------------------------------------------------------------
    std::string result_msg;

    // Only set a non-zero effort command if the 
    switch(rml_result) {
      case ReflexxesAPI::RML_WORKING:
        // all good.

        final_state_reached_ = false;
        break;
      case ReflexxesAPI::RML_FINAL_STATE_REACHED:

        if(!final_state_reached_)
        {
          ROS_INFO_STREAM_NAMED("temp","RML_FINAL_STATE_REACHED on point " << point_index_);
        }
        final_state_reached_ = true;

        // Pop the active point off the trajectory
        point_index_++;

        compute_trajectory_point_ = true;
        break;

      case ReflexxesAPI::RML_ERROR:
        result_msg = "ERROR: This is the initialization value of TypeIVRMLPosition::ReturnValue and TypeIVRMLVelocity::ReturnValue. In practice, this value. cannot be returned";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        result_msg = "INVALID_INPUT_VALUES: The applied input values are invalid (cf. RMLPositionInputParameters::CheckForValidity() RMLVelocityInputParameters::CheckForValidity()).";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        result_msg = "EXECUTION_TIME_CALCULATION: An error occurred during the first step of the algorithm (i.e., during the calculation of the synchronization time).";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        result_msg = "SYNCHRONIZATION: An error occurred during the second step of the algorithm (i.e., during the synchronization of the trajectory).";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        result_msg = "NUMBER_OF_DOFS: The number of degree of freedom of th input parameters, the output parameters, and the On-Line Trajectory Generation algorithm do not match.";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;

      case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        result_msg = "NULL_POINTER: If one of the pointers to objects of the classes RMLPositionInputParameters / RMLVelocityInputParameters, RMLPositionOutputParameters / RMLVelocityOutputParameters, RMLPositionFlags / RMLVelocityFlags is NULL, this error value will be returned.";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        result_msg = "EXECUTION_TIME_TOO_BIG: To ensure numerical stability, the value of the minimum trajectory execution time is limited to a value of case ReflexxesAPI::MAX_EXECUTION_TIME ( $ 10^{10} $ seconds).";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;

      case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        result_msg = "USER_TIME_OUT_OF_RANGE: If either the method ReflexxesAPI::RMLPositionAtAGivenSampleTime() or the method ReflexxesAPI::RMLVelocityAtAGivenSampleTime() was used, the value of the parameter is negative or larger than the value of RML_MAX_EXECUTION_TIME (see docs)";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP;

      case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        result_msg = "NO PHASE SYNCHRONIZATION: If the input flag RMLFlags::ONLY_PHASE_SYNCHRONIZATION is set and it is not possible to calculate a physically (and mathematically) correct phase-synchronized (i.e., homothetic) trajectory, this error value will be returned. Please note: Even if this error message is returned, feasible, steady, and continuous output values will be computed in any case.";
        trajectory_result_.error_code = control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP;

      default:

        ROS_ERROR_STREAM_NAMED("update","Reflexxes error code: " << rml_result << ". Setting effort commands to zero.");
        ROS_ERROR_STREAM_NAMED("update",result_msg);

        final_state_reached_ = false;
        return false;

        break;
    };

    return true;
  }

  void JointTrajectoryController::update2(const ros::Time& time, const ros::Duration& period)
  {
    updateMulti(time, period);

    // Update the sub-controllers
    for(size_t i=0; i<n_joints_; i++) 
    {
      position_controllers_[i]->update(time, period);
    }
  }

  void JointTrajectoryController::updateMulti(const ros::Time& time, const ros::Duration& period)
  {
    // Control how fast we process traj points
    static size_t counter = 0;
    counter ++;
    
    static const size_t time_gap = 3; // seconds
    size_t frequency = time_gap / period.toSec();

    // check if we should update the PID controllers this loop
    if( counter % frequency != 0 )
      return; // nothing to update yet

    bool has_error = false; // flags the PID controllers not to over write the commanded efforts of zero

    // Read the latest commanded trajectory message
    const trajectory_msgs::JointTrajectory &commanded_trajectory = *(trajectory_command_buffer_.readFromRT());

    if(new_reference_traj_) 
    {
      // Reset point index
      point_index_ = 0;

      new_reference_traj_ = false;
    }

    // check if we have processed all the points
    if( commanded_trajectory.points.size() <= point_index_)
    {
      ROS_INFO_STREAM_NAMED("temp","out of points");
      return;
    }

    ROS_INFO_STREAM_NAMED("temp","processing point " << point_index_ << " of " << commanded_trajectory.points.size() );

    // Get reference to the active trajectory point
    const trajectory_msgs::JointTrajectoryPoint &active_traj_point = commanded_trajectory.points[point_index_];


    ROS_ERROR_STREAM_NAMED("temp","NEW TRAJECTORY --------------------------------------");
    for(size_t i=0; i<n_joints_; i++)
      std::cout << "Joint " << i << " = " << active_traj_point.positions[i] << std::endl;

    ROS_ERROR_STREAM_NAMED("temp","DIFFS --------------------------------------");
    if(point_index_ > 0)
      for(size_t i=0; i<n_joints_; i++)
        std::cout << "Joint " << i << " Diff " << commanded_trajectory.points[point_index_].positions[i]
                  - commanded_trajectory.points[point_index_-1].positions[i] << std::endl;

    // Apply joint commands ------------------------------------------------
    for(size_t i=0; i<n_joints_; i++) 
    {
      double pos_target = active_traj_point.positions[i];
      double vel_target = 0; //active_traj_point.velocities[i];

      if( !has_error ) // do not over write error commands of 0
        position_controllers_[i]->setCommand(pos_target); //, vel_target);
      else
        position_controllers_[i]->setCommand(0, 0);

    } // for joints loop

    point_index_ ++;
  }

  void JointTrajectoryController::trajectoryActionCommandCB(
    const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    ROS_DEBUG("Received new joint trajectory command from action server");

    // Command recieved from action server (such as from MoveIt!)
    setTrajectoryCommand(goal->trajectory, true);

    // Temporarily initilize the num remaining points until the update() loop sets this value correctly
    //remaining_num_points_ = 1;

    // Wait until trajectory is finished
    while( ros::ok() && !action_server_->isPreemptRequested() && true ) // remaining_num_points_ > 0)
    {
      //ROS_DEBUG_STREAM_NAMED("temp","trajectoryActionCommandCB sleeping, remaining_num_points_= ?");
        //        remaining_num_points_);
      ros::Duration(0.1).sleep();
    }

    if(true)
    {
      // set the action state to succeeded
      control_msgs::FollowJointTrajectoryResult traj_result;
      traj_result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      std::string error_msg = "Trajectory execution successfully completed";

      ROS_INFO_STREAM_NAMED("trajectoryActionCommandCB",error_msg);
      action_server_->setSucceeded(traj_result, error_msg);
    }
  }

  void JointTrajectoryController::trajectoryTestCommandCB(const std_msgs::EmptyConstPtr& thing)
  {
    trajectory_msgs::JointTrajectory msg;

    //msg.header.stamp = ros::Time::now();

    msg.joint_names.push_back(std::string("right_e0"));
    msg.joint_names.push_back(std::string("right_e1"));
    msg.joint_names.push_back(std::string("right_s0"));
    msg.joint_names.push_back(std::string("right_s1"));
    msg.joint_names.push_back(std::string("right_w0"));
    msg.joint_names.push_back(std::string("right_w1"));
    msg.joint_names.push_back(std::string("right_w2"));

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(7);

    double time_from_start = 0;

    // point
    static const double a_positions1[] = {-0.00010465668109027604, 0.031050410994437172, -1.0404484585002649e-05, 0.06556338867550338, -0.0010021154586947745, 0.01804246958448985, -5.525417278207101e-05};    
    std::vector<double> v_positions1 (a_positions1, a_positions1 + sizeof(a_positions1) / sizeof(a_positions1[0]) );
    point.positions = v_positions1;
    point.velocities.resize(7);
    point.accelerations.resize(7);
    point.time_from_start = ros::Duration(time_from_start += 10.0);
    msg.points.push_back(point);
  
    // point
    static const double a_positions2[] = {0.05286104652745683, 0.0379352831414734, -0.012786901829920542, 0.05876995400033727, 0.021117809612794813, -0.023066806412942058, 0.10387393401780202};
    std::vector<double> v_positions2 (a_positions2, a_positions2 + sizeof(a_positions2) / sizeof(a_positions2[0]) );
    point.positions = v_positions2;
    static const double a_velocities2[] = {0.22619108605247856, 0.029401983055693352, -0.05456228531714662, -0.02901149751853807, 0.09446357873546157, -0.1755579784918196, 0.44383165945745756};
    std::vector<double> v_velocities2 (a_velocities2, a_velocities2 + sizeof(a_velocities2) / sizeof(a_velocities2[0]) );
    point.velocities = v_velocities2;
    static const double a_accelerations2[] = {0.49424873089266885, 0.06424608973154959, -0.11922370922409703, -0.06339284222061026, 0.2064117765222163, -0.3836106434695352, 0.969813789947236};
    std::vector<double> v_accelerations2 (a_accelerations2, a_accelerations2 + sizeof(a_accelerations2) / sizeof(a_accelerations2[0]) );
    point.accelerations = v_accelerations2;
    point.time_from_start = ros::Duration(time_from_start += 10.0);
    msg.points.push_back(point);

    // point
    static const double a_positions3[] = {0.10582674973600394, 0.04482015528850963, -0.02556339917525608, 0.05197651932517117, 0.0432377346842844, -0.06417608241037397, 0.20780312220838612};
    std::vector<double> v_positions3 (a_positions3, a_positions3 + sizeof(a_positions3) / sizeof(a_positions3[0]) );
    point.positions = v_positions3;
    static const double a_velocities3[] = {0.32933654800224255, 0.0428095897719678, -0.07944324867556203, -0.042241038813148474, 0.1375399423364414, -0.2556142225575305, 0.6462234616350577};
    std::vector<double> v_velocities3 (a_velocities3, a_velocities3 + sizeof(a_velocities3) / sizeof(a_velocities3[0]) );
    point.velocities = v_velocities3;
    static const double a_accelerations3[] = {0.4956135904600064, 0.06442350422905523, -0.11955294349424676, -0.0635679004894484, 0.20698177917551286, -0.38466997781709344, 0.9724919144360015};
    std::vector<double> v_accelerations3 (a_accelerations3, a_accelerations3 + sizeof(a_accelerations3) / sizeof(a_accelerations3[0]) );
    point.accelerations = v_accelerations3;
    point.time_from_start = ros::Duration(time_from_start += 10.0);
    msg.points.push_back(point);

    // point
    static const double a_positions4[] = {0.15879245294455105, 0.051705027435545864, -0.03833989652059162, 0.04518308465000506, 0.06535765975577398, -0.10528535840780587, 0.3117323103989702};
    std::vector<double> v_positions4 (a_positions4, a_positions4 + sizeof(a_positions4) / sizeof(a_positions4[0]) );
    point.positions = v_positions4;
    static const double a_velocities4[] = {0.4000616766712626, 0.05200296282229126, -0.09650371165346944, -0.05131231536382344, 0.167076688798022, -0.3105074583361778, 0.7850001560235828};
    std::vector<double> v_velocities4 (a_velocities4, a_velocities4 + sizeof(a_velocities4) / sizeof(a_velocities4[0]) );
    point.velocities = v_velocities4;
    static const double a_accelerations4[] = {0.4545149583733463, 0.05908120137657295, -0.10963904578416486, -0.05829654836144768, 0.18981786729997147, -0.3527713168493544, 0.8918482675142966};
    std::vector<double> v_accelerations4 (a_accelerations4, a_accelerations4 + sizeof(a_accelerations4) / sizeof(a_accelerations4[0]) );
    point.accelerations = v_accelerations4;
    point.time_from_start = ros::Duration(time_from_start += 10.0);
    msg.points.push_back(point);

    ROS_INFO_STREAM_NAMED("temp","msg: " << msg);

    this->setTrajectoryCommand(msg, false);
  }


  void JointTrajectoryController::trajectoryMsgCommandCB(
    const trajectory_msgs::JointTrajectoryConstPtr& msg)
  {
    this->setTrajectoryCommand(*msg, false);
  }
  
  void JointTrajectoryController::setTrajectoryCommand(
      const trajectory_msgs::JointTrajectory& traj_msg, bool is_action)
  {
    ROS_DEBUG("Received new command");
    // the writeFromNonRT can be used in RT, if you have the guarantee that 
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    trajectory_command_buffer_.writeFromNonRT(traj_msg);
    new_reference_traj_ = true;
  }


} // namespace

PLUGINLIB_EXPORT_CLASS( 
    reflexxes_effort_controllers::JointTrajectoryController,
    controller_interface::ControllerBase)

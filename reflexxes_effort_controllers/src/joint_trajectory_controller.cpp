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
      new_reference_(false),
      recompute_trajectory_(false),
      is_action_(false),
      final_state_reached_(false)
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
        ROS_INFO("Loading joint information for joint '%s' (namespace: %s)", joint_names_[i].c_str(), joint_nh.getNamespace().c_str());

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
      //      initial_point.positions.push_back(joints_[i].getPosition());
      //      initial_point.velocities.push_back(joints_[i].getVelocity());
      initial_point.positions.push_back(0);
      initial_point.velocities.push_back(0);
      initial_point.accelerations.push_back(0.0);
    }
    initial_point.time_from_start = ros::Duration(1.0);
    initial_command.points.push_back(initial_point);
    trajectory_command_buffer_.initRT(initial_command);
    new_reference_ = true;
  }

  void JointTrajectoryController::stopping(const ros::Time& time)
  {
    action_server_->shutdown();
  }

  void JointTrajectoryController::update(
      const ros::Time& time, 
      const ros::Duration& period)
  {
    //ROS_INFO_STREAM_NAMED("temp","Time: " << time << " Period: " << period.toSec() );

    // Read the latest commanded trajectory message
    const trajectory_msgs::JointTrajectory &commanded_trajectory = *(trajectory_command_buffer_.readFromRT());

    // Check for a new reference
    if(new_reference_) {
      // Start trajectory immediately if stamp is zero
      if(commanded_trajectory.header.stamp.isZero()) {
        ROS_WARN_STREAM_NAMED("temp","set from time=" << time << " period="<<period.toSec() );
        commanded_start_time_ = time + period;
      } else {
        commanded_start_time_ = commanded_trajectory.header.stamp;
      }
      // Reset point index
      point_index_ = 0;
      recompute_trajectory_ = true;
      new_reference_ = false;
    }

    // Initialize RML result
    int rml_result = 0;

    //bool trajectory_running = commanded_start_time_ >= time;
    bool trajectory_running = true; // true
    //    bool trajectory_running = commanded_start_time_ <= time;
    bool trajectory_incomplete = point_index_ < commanded_trajectory.points.size();

    bool tolerance_violated = false;
    for(int i=0; i<n_joints_; i++) {
      if(std::abs(rml_out_->NewPositionVector->VecData[i] - joints_[i].getPosition()) > position_tolerances_[i]) {
        //recompute_trajectory_ = true;

        /*
          ROS_WARN_STREAM_NAMED("update","(Re)Computing Trajectory: Joint found outside the tolerance bounds of current trajectory:\n"
            << "Joint " << joint_names_[i] << " is desired to be at position " << rml_out_->NewPositionVector->VecData[i] 
            << " but is currently at " << joints_[i].getPosition()
            << " which is outside the tolerance of " << position_tolerances_[i] );
          break;
        */
        //std::cout << " BAD ";
        break;
      }
    }

    // Compute RML traj after the start time and if there are still points in the queue
    if(recompute_trajectory_ && trajectory_running && trajectory_incomplete) {
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");
      ROS_WARN_STREAM_NAMED("temp","Recomputing trajectory, on point " << point_index_ << " of " 
        << commanded_trajectory.points.size() );
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");
      ROS_WARN_STREAM_NAMED("temp","----------------------------------------------------------------");
      // Get reference to the active trajectory point
      const trajectory_msgs::JointTrajectoryPoint &active_traj_point_const = commanded_trajectory.points[point_index_];
      trajectory_msgs::JointTrajectoryPoint active_traj_point = active_traj_point_const;

      //static double time_from_start_hack = 0;
      active_traj_point.time_from_start = ros::Duration(point_index_*2);

      // Compute the trajectory
      ROS_DEBUG("RML Recomputing trajectory...");

      // Update RML input parameters
      for(int i=0; i<n_joints_; i++) {
        rml_in_->CurrentPositionVector->VecData[i] = joints_[i].getPosition();
        rml_in_->CurrentVelocityVector->VecData[i] = joints_[i].getVelocity();
        rml_in_->CurrentAccelerationVector->VecData[i] = 0.0;

        rml_in_->TargetPositionVector->VecData[i] = active_traj_point.positions[i];
        rml_in_->TargetVelocityVector->VecData[i] = active_traj_point.velocities[i];
      }

      // Store the traj start time
      traj_start_time_ = time;

      // Set desired execution time for this trajectory (definitely > 0)
      double min_sync_time = (active_traj_point.time_from_start - (traj_start_time_ - commanded_start_time_)).toSec();

      ROS_DEBUG_STREAM("RML IN: MinimumSynchronizatonTime: " << min_sync_time
        << "\n Traj point time from start: " << active_traj_point.time_from_start.toSec()  // the delay until starting this point
        << "\n traj_start_time_: " << traj_start_time_.toSec()   // the time at which we send the command to reflexxes
        << "\n commanded_start_time_: " << commanded_start_time_.toSec() // when to start execution in the future 
      );

      if( min_sync_time < 0 )
      {
        ROS_ERROR_STREAM_NAMED("update","Minimum synchronization time was calculated to be less than zero: " << min_sync_time );        

        min_sync_time = 0.1;
      }
      rml_in_->SetMinimumSynchronizationTime(min_sync_time);
                
      // Hold fixed at final point once trajectory is complete
      rml_flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
      rml_flags_.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

      // Compute trajectory
      rml_result = rml_->RMLPosition(
          *rml_in_.get(), 
          rml_out_.get(), 
          rml_flags_);

      ROS_ERROR_STREAM_NAMED("temp","NEW TRAJECTORY --------------------------------------");
      for(int i=0; i<n_joints_; i++)
        std::cout << "Joint " << i << " = " << rml_out_->NewPositionVector->VecData[i] << std::endl;
      ROS_ERROR_STREAM_NAMED("temp","NEW TRAJECTORY --------------------------------------");
      
      // Disable recompute flag
      recompute_trajectory_ = false;
    } else {
      /*
      ROS_DEBUG_STREAM_NAMED("temp","Sample pre-computed traj: "
        << " recompute_trajectory_="<<recompute_trajectory_
        << " trajectory_running="<<trajectory_running
        << " trajectory_incomplete="<<trajectory_incomplete);
      */

      //std::cout << " [" << (time - traj_start_time_).toSec() << "s] ";

      // Sample the already computed trajectory
      rml_result = rml_->RMLPositionAtAGivenSampleTime(
        (time - traj_start_time_).toSec(),
        rml_out_.get());
    }

    // Error Check -------------------------------------------------------------
    std::string result_msg;
    bool has_error = false; // flags the PID controllers not to over write the commanded efforts of zero

    // Only set a non-zero effort command if the 
    switch(rml_result) {
      case ReflexxesAPI::RML_WORKING:
        // S'all good.
        //ROS_INFO_STREAM_NAMED("temp","RML_WORKING on point " << point_index_);
        //std::cout << "WORK " << point_index_ << " \t";

        final_state_reached_ = false;
        break;
      case ReflexxesAPI::RML_FINAL_STATE_REACHED:

        //        std::cout << "FINAL " << point_index_ << " \t";

        if(!final_state_reached_)
          ROS_INFO_STREAM_NAMED("temp","RML_FINAL_STATE_REACHED on point " << point_index_);
        final_state_reached_ = true;

        // Pop the active point off the trajectory
        point_index_++;

        recompute_trajectory_ = true;
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
        if (loop_count_ % decimation_ == 0) {
          ROS_ERROR_STREAM_NAMED("update","Reflexxes error code: " << rml_result << ". Setting effort commands to zero.");
          ROS_ERROR_STREAM_NAMED("update",result_msg);
        }
        has_error = true;
        final_state_reached_ = false;
        break;
    };
    
    // Apply joint commands ------------------------------------------------
    for(int i=0; i<n_joints_; i++) 
    {
      double pos_target = rml_out_->NewPositionVector->VecData[i];
      double vel_target = rml_out_->NewVelocityVector->VecData[i];

      if( !has_error ) // do not over write error commands of 0
        position_controllers_[i]->setCommand(pos_target, vel_target);
      else
        position_controllers_[i]->setCommand(0, 0);

      // Update the controller
      position_controllers_[i]->update(time, period);

    } // for joints loop

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

    /*
  void JointTrajectoryController::trajectoryTestCommandCB()
  {
    trajectory_msgs::JointTrajectory msg;


    msg.header.
  header: 
  seq: 0
      stamp: 
    secs: 65
      nsecs: 691000000
      frame_id: ''
      goal_id: 
    stamp: 
    secs: 65
      nsecs: 691000000
      id: /move_group-1-65.691000000
      goal: 
    trajectory: 
    header: 
    seq: 0
      stamp: 
    secs: 0
      nsecs: 0
      frame_id: /base
      joint_names: ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
      points: 
      - 
      positions: [-0.00010465668109027604, 0.031050410994437172, -1.0404484585002649e-05, 0.06556338867550338, -0.0010021154586947745, 0.01804246958448985, -5.525417278207101e-05]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: 
    secs: 0
      nsecs: 0
      - 
      positions: [0.05286104652745683, 0.0379352831414734, -0.012786901829920542, 0.05876995400033727, 0.021117809612794813, -0.023066806412942058, 0.10387393401780202]
      velocities: [0.22619108605247856, 0.029401983055693352, -0.05456228531714662, -0.02901149751853807, 0.09446357873546157, -0.1755579784918196, 0.44383165945745756]
      accelerations: [0.49424873089266885, 0.06424608973154959, -0.11922370922409703, -0.06339284222061026, 0.2064117765222163, -0.3836106434695352, 0.969813789947236]
      time_from_start: 
    secs: 0
      nsecs: 323929188


    this->setTrajectoryCommand(msg, false);
  }
    */


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
    new_reference_ = true;
  }


} // namespace

PLUGINLIB_EXPORT_CLASS( 
    reflexxes_effort_controllers::JointTrajectoryController,
    controller_interface::ControllerBase)

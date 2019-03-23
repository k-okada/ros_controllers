/*********************************************************************
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
 *********************************************************************/

/*
 Author: Vijay Pradeep
 Contributors: Jonathan Bohren, Wim Meeussen, Dave Coleman
 Desc: Effort(force)-based position controller using basic PID loop
*/

#include <effort_controllers/joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {

JointPositionController::JointPositionController()
  : loop_count_(0)
{}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  // Setup for mimic joints
  using namespace XmlRpc;
  XmlRpcValue xml_array;
  if (n.getParam("mimic_joints", xml_array))
  {
    if (xml_array.getType() != XmlRpcValue::TypeStruct) {
      ROS_ERROR("mimic_joints must be the struct of joint/type/pid");
      return false;
    }
    for(XmlRpcValue::ValueStruct::const_iterator it = xml_array.begin(); it != xml_array.end(); ++it) {
      std::string mimic_joint_name = xml_array[it->first]["joint"];
      mimic_joint_urdfs_.push_back(urdf.getJoint(mimic_joint_name));
      if (!mimic_joint_urdfs_.back())
      {
        ROS_ERROR("Could not find joint '%s' in urdf", mimic_joint_name.c_str());
        return false;
      }
      // Get joint handle from hardware interface
      mimic_joints_.push_back(robot->getHandle(mimic_joint_name));
      // create controller andpublisher
      mimic_pid_controllers_.push_back(control_toolbox::Pid());
      if(!mimic_pid_controllers_.back().init(ros::NodeHandle(n, "mimic_joints/"+(std::string)(it->first)+"/pid")))
      {
        ROS_WARN_STREAM("failed to load pid for " << mimic_joint_name << ". use default settings");
        mimic_pid_controllers_.back().setGains(pid_controller_.getGains());
      }
      mimic_controller_state_publishers_.push_back(boost::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> >(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, (std::string)(it->first)+"/state", 1)));
      ROS_INFO_STREAM("Loading mimic joint " << (std::string)(it->first));
    }
  }

  return true;
}

void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void JointPositionController::printDebug()
{
  pid_controller_.printValues();
}

std::string JointPositionController::getJointName()
{
  return joint_.getName();
}

double JointPositionController::getPosition()
{
  return joint_.getPosition();
}

// Set the joint position command
void JointPositionController::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void JointPositionController::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

void JointPositionController::starting(const ros::Time& time)
{
  double pos_command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  command_.initRT(command_struct_);

  pid_controller_.reset();
  for(std::vector<control_toolbox::Pid>::iterator it = mimic_pid_controllers_.begin(); it != mimic_pid_controllers_.end(); ++it)
  {
    (*it).reset();
  }
}

void JointPositionController::update(double command_position, double command_velocity, bool has_velocity, const ros::Time& time, const ros::Duration& period, hardware_interface::JointHandle joint, urdf::JointConstSharedPtr joint_urdf, control_toolbox::Pid &pid_controller, realtime_tools::RealtimePublisher< control_msgs::JointControllerState> *controller_state_publisher)
{
  double error, vel_error;
  double commanded_effort;

  double current_position = joint.getPosition();

  // Compute position error
  if (joint_urdf->type == urdf::Joint::REVOLUTE)
  {
   angles::shortest_angular_distance_with_limits(
      current_position,
      command_position,
      joint_urdf->limits->lower,
      joint_urdf->limits->upper,
      error);
  }
  else if (joint_urdf->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else //prismatic
  {
    error = command_position - current_position;
  }

  // Decide which of the two PID computeCommand() methods to call
  if (has_velocity)
  {
    // Compute velocity error if a non-zero velocity command was given
    vel_error = command_velocity - joint.getVelocity();

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error.
    commanded_effort = pid_controller.computeCommand(error, vel_error, period);
  }
  else
  {
    // Set the PID error and compute the PID command with nonuniform
    // time step size.
    commanded_effort = pid_controller.computeCommand(error, period);
  }

  joint.setCommand(commanded_effort);

  // publish state
  if (loop_count_ % 10 == 0)
  {
    if(controller_state_publisher && controller_state_publisher->trylock())
    {
      controller_state_publisher->msg_.header.stamp = time;
      controller_state_publisher->msg_.set_point = command_position;
      controller_state_publisher->msg_.process_value = current_position;
      controller_state_publisher->msg_.process_value_dot = joint.getVelocity();
      controller_state_publisher->msg_.error = error;
      controller_state_publisher->msg_.time_step = period.toSec();
      controller_state_publisher->msg_.command = commanded_effort;

      double dummy;
      bool antiwindup;
      pid_controller.getGains(controller_state_publisher->msg_.p,
                              controller_state_publisher->msg_.i,
                              controller_state_publisher->msg_.d,
                              controller_state_publisher->msg_.i_clamp,
                              dummy,
                              antiwindup);
      controller_state_publisher->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher->unlockAndPublish();
    }
  }
}

void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  double command_velocity = command_struct_.velocity_;
  bool has_velocity_ =  command_struct_.has_velocity_;

  // Make sure joint is within limits if applicable
  enforceJointLimits(command_position);

  update(command_position, command_velocity, has_velocity_, time, period, joint_, joint_urdf_, pid_controller_, controller_state_publisher_.get());
  for(unsigned int i = 0; i < mimic_joints_.size(); i++ )
  {
    urdf::JointMimicSharedPtr mimic = mimic_joint_urdfs_[i]->mimic;
    double mimic_command_position = command_position * mimic->multiplier + mimic->offset;
    enforceJointLimits(mimic_joint_urdfs_[i], mimic_command_position);
    update(mimic_command_position, command_velocity, has_velocity_, time, period, mimic_joints_[i], mimic_joint_urdfs_[i], mimic_pid_controllers_[i], mimic_controller_state_publishers_[i].get());
  }

  loop_count_++;
}

void JointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointPositionController::enforceJointLimits(double &command)
{
  enforceJointLimits(joint_urdf_, command);
}
void JointPositionController::enforceJointLimits(urdf::JointConstSharedPtr joint_urdf, double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf->limits->upper ) // above upper limnit
    {
      command = joint_urdf->limits->upper;
    }
    else if( command < joint_urdf->limits->lower ) // below lower limit
    {
      command = joint_urdf->limits->lower;
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( effort_controllers::JointPositionController, controller_interface::ControllerBase)

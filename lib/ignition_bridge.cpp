/*!*******************************************************************************************
 *  \file       ignition_bridge.cpp
 *  \brief      Implementation of an Ignition Gazebo bridge to ROS
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "ignition_bridge.hpp"

namespace ignition_platform
{

  std::string IgnitionBridge::name_space_ = "";
  odometryCallbackType IgnitionBridge::odometryCallback_ = [](nav_msgs::msg::Odometry &msg) {};
  groundTruthCallbackType IgnitionBridge::groundTruthCallback_ = [](geometry_msgs::msg::Pose &msg) {};

  IgnitionBridge::IgnitionBridge(std::string name_space, std::string world_name)
  {
    name_space_ = name_space;

    // Initialize the ignition node
    ign_node_ptr_ = std::make_shared<ignition::transport::Node>();

    // Initialize subscribers
    ign_node_ptr_->Subscribe("/model" + name_space + "/odometry",
                             IgnitionBridge::ignitionOdometryCallback);

    ign_node_ptr_->Subscribe("/world/" + world_name + "/pose/info",
                             IgnitionBridge::ignitionGroundTruthCallback);

    // Initialize publishers
    ign_thrust_left_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        "model" + name_space + ign_topic_command_thrust_left_);

    ign_thrust_right_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        "model" + name_space + ign_topic_command_thrust_right_);

    ign_rot_left_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        name_space + ign_topic_command_rot_left_);

    ign_rot_right_pub_ = ign_node_ptr_->Advertise<ignition::msgs::Double>(
        name_space + ign_topic_command_rot_right_);

    return;
  };

  void IgnitionBridge::sendThrustMsg(const std_msgs::msg::Float64 &left_thrust, const std_msgs::msg::Float64 &right_thrust)
  {
      ignition::msgs::Double ign_left_msg;
      ignition::msgs::Double ign_right_msg;
      ros_ign_bridge::convert_ros_to_ign(left_thrust, ign_left_msg);
      ros_ign_bridge::convert_ros_to_ign(right_thrust, ign_right_msg);
      ign_thrust_left_pub_.Publish(ign_left_msg);
      ign_thrust_right_pub_.Publish(ign_right_msg);
      return;
  };

  void IgnitionBridge::sendRotationMsg(const std_msgs::msg::Float64 &left_rotation, const std_msgs::msg::Float64 &right_rotation)
  {
      ignition::msgs::Double ign_left_msg;
      ignition::msgs::Double ign_right_msg;
      ros_ign_bridge::convert_ros_to_ign(left_rotation, ign_left_msg);
      ros_ign_bridge::convert_ros_to_ign(right_rotation, ign_right_msg);
      ign_rot_left_pub_.Publish(ign_left_msg);
      ign_rot_right_pub_.Publish(ign_right_msg);
      return;
  };

  void IgnitionBridge::setOdometryCallback(odometryCallbackType callback)
  {
    odometryCallback_ = callback;
    return;
  };

  void IgnitionBridge::ignitionOdometryCallback(const ignition::msgs::Odometry &msg)
  {
    nav_msgs::msg::Odometry odom_msg;
    ros_ign_bridge::convert_ign_to_ros(msg, odom_msg);
    odometryCallback_(odom_msg);
    return;
  };

  void IgnitionBridge::setGroundTruthCallback(groundTruthCallbackType callback)
  {
    groundTruthCallback_ = callback;
    return;
  };

  void IgnitionBridge::ignitionGroundTruthCallback(const ignition::msgs::Pose_V &msg)
  {
    // Remove firts element of name_space_
    std::string name_space_2 = name_space_.substr(1);
    for (auto const &p : msg.pose())
    {
      if (p.name() == name_space_2)
      {
        geometry_msgs::msg::Pose pose;
        ros_ign_bridge::convert_ign_to_ros(p, pose);
        groundTruthCallback_(pose);
        return;
      }
    }
    return;
  };

} // namespace ignition_platform

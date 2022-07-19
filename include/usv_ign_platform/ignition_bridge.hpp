/*!*******************************************************************************************
 *  \file       ignition_bridge.hpp
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

#ifndef IGNITION_BRIDGE_HPP_
#define IGNITION_BRIDGE_HPP_

#include <memory>
#include <string>
#include <iostream>

#include <unordered_map>
#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ros_ign_bridge/convert.hpp>

namespace ignition_platform
{
    typedef void (*poseCallbackType)(const geometry_msgs::msg::PoseStamped &msg);
    typedef void (*odometryCallbackType)(nav_msgs::msg::Odometry &msg);

    typedef void (*cameraCallbackType)(const sensor_msgs::msg::Image &msg, const std::string &sensor_name);
    typedef void (*cameraInfoCallbackType)(const sensor_msgs::msg::CameraInfo &msg, const std::string &sensor_name);

    typedef void (*laserScanCallbackType)(const sensor_msgs::msg::LaserScan &msg, const std::string &sensor_name);
    typedef void (*pointCloudCallbackType)(const sensor_msgs::msg::PointCloud2 &msg, const std::string &sensor_name);

    class IgnitionBridge
    {
    public:
        IgnitionBridge(std::string name_space = "/");
        ~IgnitionBridge(){};

    public:
        std::shared_ptr<ignition::transport::Node> ign_node_ptr_;
        ignition::transport::v11::Node::Publisher ign_thrust_left_pub_;
        ignition::transport::v11::Node::Publisher ign_thrust_right_pub_;
        ignition::transport::v11::Node::Publisher ign_rot_left_pub_;
        ignition::transport::v11::Node::Publisher ign_rot_right_pub_;

    private:
        std::string name_space_;

        const std::string &ign_topic_command_thrust_left_ = "/joint/left_engine_propeller_joint/cmd_thrust";
        const std::string &ign_topic_command_thrust_right_ = "/joint/right_engine_propeller_joint/cmd_thrust";
        const std::string &ign_topic_command_rot_left_ = "/left/thruster/joint/cmd_pos";
        const std::string &ign_topic_command_rot_right_ = "/right/thruster/joint/cmd_pos";
        const std::string &ign_topic_sensor_pose_ = "/pose";
        const std::string &ign_topic_sensor_odometry_ = "/odometry";

    public:
        void sendThrustMsg(const std_msgs::msg::Float64 &left_thrust, const std_msgs::msg::Float64 &right_thrust);
        void sendRotationMsg(const std_msgs::msg::Float64 &left_rotation, const std_msgs::msg::Float64 &right_rotation);

        void setPoseCallback(poseCallbackType callback);
        void setOdometryCallback(odometryCallbackType callback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            cameraCallbackType cameraCallback,
            cameraInfoCallbackType cameraInfoCallback);

        void addSensor(
            std::string world_name,
            std::string name_space,
            std::string sensor_name,
            std::string link_name,
            std::string sensor_type,
            laserScanCallbackType laserScanCallback,
            pointCloudCallbackType pointCloudCallback);

    private:
        // Ignition callbacks
        static poseCallbackType poseCallback_;
        static void ignitionPoseCallback(const ignition::msgs::Pose &msg);
        static odometryCallbackType odometryCallback_;
        static void ignitionOdometryCallback(const ignition::msgs::Odometry &msg);


        static std::unordered_map<std::string, std::string> callbacks_sensors_names_;

        static void ignitionCameraCallback(const ignition::msgs::Image &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, cameraCallbackType> callbacks_camera_;
        static void ignitionCameraInfoCallback(const ignition::msgs::CameraInfo &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, cameraInfoCallbackType> callbacks_camera_info_;

        static void ignitionLaserScanCallback(const ignition::msgs::LaserScan &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, laserScanCallbackType> callbacks_laser_scan_;
        static void ignitionPointCloudCallback(const ignition::msgs::PointCloudPacked &msg, const ignition::transport::MessageInfo &_info);
        static std::unordered_map<std::string, pointCloudCallbackType> callbacks_point_cloud_;
    };
}

#endif // IGNITION_BRIDGE_HPP_
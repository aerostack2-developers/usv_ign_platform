/*!*******************************************************************************************
 *  \file       ignition_platform.hpp
 *  \brief      Implementation of an Ignition Gazebo UAV platform
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

#ifndef IGNITION_PLATFORM_HPP_
#define IGNITION_PLATFORM_HPP_

#include <memory>
#include <cmath>
#include <string>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>

#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
// #include <math.h>
#include <as2_core/core_functions.hpp>
#include <as2_core/aerial_platform.hpp>
#include <as2_core/frame_utils/frame_utils.hpp>
#include <as2_core/sensor.hpp>
#include <as2_core/names/topics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float64.hpp>

#include "ignition_bridge.hpp"

#define CMD_FREQ 10 // miliseconds

namespace ignition_platform
{
    using Vector3d = Eigen::Vector3d;

    class USVIgnitionPlatform : public as2::AerialPlatform
    {
    public:
        USVIgnitionPlatform();
        ~USVIgnitionPlatform(){};

    public:
        void configureSensors() override;
        bool ownSendCommand() override;
        bool ownSetArmingState(bool state) override;
        bool ownSetOffboardControl(bool offboard) override;
        bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) override;

        static std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> pose_ptr_;
        static void poseCallback(const geometry_msgs::msg::PoseStamped &msg);

        static std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> odometry_raw_estimation_ptr_;
        static void odometryCallback(const nav_msgs::msg::Odometry &msg);

        static std::unordered_map<std::string, as2::sensors::Camera> callbacks_camera_;
        static void cameraCallback(const sensor_msgs::msg::Image &msg, const std::string &sensor_name);
        static void cameraInfoCallback(const sensor_msgs::msg::CameraInfo &msg, const std::string &sensor_name);

        static std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> callbacks_laser_scan_;
        static void laserScanCallback(const sensor_msgs::msg::LaserScan &msg, const std::string &sensor_name);

        static std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::PointCloud2>> callbacks_point_cloud_;
        static void pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg, const std::string &sensor_name);

    private:
        std::shared_ptr<IgnitionBridge> ignition_bridge_;

        static bool odometry_info_received_;
        bool parameters_read;
        as2_msgs::msg::ControlMode control_in_;

        double yaw_rate_limit_ = M_PI_2;
        double K_yaw_rate_ = 10;
        double K_yaw_force_ = 10;
        double GainThrust_ = 20;
        double maximum_thrust_ = 1000.0;

        float antiwindup_cte_ = 1.0f;
        double alpha_ = 0.1;
        Eigen::Vector3d yaw_ang_mat_ = Eigen::Vector3d::Identity();
        double yaw_accum_error_ = 0.0;

        static geometry_msgs::msg::Quaternion self_orientation_;
        Eigen::Vector2d motor_thrust_cmd_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d motor_pos_cmd_ = Eigen::Vector2d::Zero();

        std::vector<std::string> parameters_to_read_ = {
            "yaw_rate_limit",
            "K_yaw_rate",
            "K_yaw_force",
            "GainThrust",
            "maximum_thrust",
            "antiwindup_cte",
            "alpha",
            "yaw_speed_controller.Kp",
            "yaw_speed_controller.Ki",
            "yaw_speed_controller.Kd",
        };

        std::unordered_map<std::string, double> parameters_ = {
            {"antiwindup_cte", 5.0},
            {"alpha", 0.1},
            {"yaw_speed_controller.Kp", 1.0},
            {"yaw_speed_controller.Ki", 0.0},
            {"yaw_speed_controller.Kd", 0.0},
            {"yaw_rate_limit", 0.78539816339744830962},
            {"K_yaw_rate", 4.0},
            {"K_yaw_force", 10.0},
            {"GainThrust", 50.0},
            {"maximum_thrust", 1000.0},
        };

    private:
        void declareParameters();
        void updateGains();
        void resetCommandMsg();

        void speedController(const Eigen::Vector3d &vel_flu);

        double computeYawSpeed(
            const double &yaw_angle_error,
            const double &dt);
        
        void sendUSVMsg();

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    };
}

#endif // IGNITION_PLATFORM_HPP_

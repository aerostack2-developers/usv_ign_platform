/*!*******************************************************************************************
 *  \file       ignition_platform.cpp
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

#include "usv_ign_platform.hpp"

namespace ignition_platform {
bool USVIgnitionPlatform::odometry_info_received_ = false;
double USVIgnitionPlatform::yaw_ = 0.0;

std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>
    USVIgnitionPlatform::pose_ptr_ = nullptr;
std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>>
    USVIgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

std::unordered_map<std::string, as2::sensors::Camera> USVIgnitionPlatform::callbacks_camera_ = {};
std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::LaserScan>>
    USVIgnitionPlatform::callbacks_laser_scan_ = {};
std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::PointCloud2>>
    USVIgnitionPlatform::callbacks_point_cloud_ = {};

USVIgnitionPlatform::USVIgnitionPlatform() : as2::AerialPlatform() {
  this->declare_parameter<std::string>("sensors");
  ignition_bridge_ = std::make_shared<IgnitionBridge>(this->get_namespace());

  this->configureSensors();

  left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("left/thrust/cmd_thrust",
                                                                    rclcpp::QoS(1));
  right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("right/thrust/cmd_thrust",
                                                                     rclcpp::QoS(1));

  // TODO :COMPLETE
  // this->declare_parameter<float>("kp_speed_to_force", 10);
  // this->declare_parameter<float>("kp_yaw_speed_");


  // Timer to send command
  static auto timer_commands_ = this->create_wall_timer(std::chrono::milliseconds(CMD_FREQ),
                                                        [this]() { this->sendCommand(); });
};

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.emplace_back(item);
  }
  return elems;
};

void USVIgnitionPlatform::configureSensors() {
  pose_ptr_ = std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>("pose", this);
  ignition_bridge_->setPoseCallback(poseCallback);

  odometry_raw_estimation_ptr_ =
      std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
  ignition_bridge_->setOdometryCallback(odometryCallback);

  std::string sensors_param = this->get_parameter("sensors").as_string();
  std::vector<std::string> sensor_config_list = split(sensors_param, ':');

  for (auto sensor_config : sensor_config_list) {
    std::vector<std::string> sensor_config_params = split(sensor_config, ',');

    if (sensor_config_params.size() != 5) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "Wrong sensor configuration: %s",
                        sensor_config.c_str());
      continue;
    }

    std::string sensor_type = sensor_config_params[4];
    if (sensor_type == "camera") {
      as2::sensors::Camera camera = as2::sensors::Camera(sensor_config_params[2], this);
      callbacks_camera_.insert(std::make_pair(sensor_config_params[2], camera));

      ignition_bridge_->addSensor(sensor_config_params[0], sensor_config_params[1],
                                  sensor_config_params[2], sensor_config_params[3],
                                  sensor_config_params[4], cameraCallback, cameraInfoCallback);
    } else if (sensor_type == "lidar") {
      as2::sensors::Sensor<sensor_msgs::msg::LaserScan> laser_scan_sensor(sensor_config_params[2],
                                                                          this);
      as2::sensors::Sensor<sensor_msgs::msg::PointCloud2> point_cloud_sensor(
          sensor_config_params[2] + "/points", this);

      callbacks_laser_scan_.insert(std::make_pair(sensor_config_params[2], laser_scan_sensor));
      callbacks_point_cloud_.insert(std::make_pair(sensor_config_params[2], point_cloud_sensor));

      ignition_bridge_->addSensor(sensor_config_params[0], sensor_config_params[1],
                                  sensor_config_params[2], sensor_config_params[3],
                                  sensor_config_params[4], laserScanCallback, pointCloudCallback);
    } else {
      RCLCPP_WARN(this->get_logger(), "Sensor type not supported: %s", sensor_type.c_str());
    }
  }

  return;
};

void USVIgnitionPlatform::speedController(const Eigen::Vector3d &vel_flu) {
  Eigen::Vector2d motor_thrust_cmd;
  const double K_yaw_rate = 10;
  const double K_yaw_diff = 10;
  const double GainThrust = 20;
  const double maximum_thrust = 1000.0;

  double yaw_rate = 0.0f;

  // compute speed norm
  if (vel_flu.norm() < 0.1) {
    yaw_rate = command_twist_msg_.twist.angular.z;
    if (abs(yaw_rate) < 0.1) {
      PublishUSVCommands(Eigen::Vector2d(0.0, 0.0));
      return;
    }
  } else {
    double yaw_diff = atan2(vel_flu.y(), vel_flu.x());
    yaw_rate = K_yaw_diff * yaw_diff;
    Eigen::Vector2d P_speed = Eigen::Vector2d(vel_flu.x(), vel_flu.y()) * GainThrust;
    motor_thrust_cmd << P_speed.x() / 2.0f, P_speed.x() / 2.0f;
  }

  double diff_force = K_yaw_rate * yaw_rate;
  // yaw_rate control
  if (yaw_rate > 0)
    motor_thrust_cmd += Eigen::Vector2d(-diff_force / 2.0f, diff_force / 2.0f);
  else
    motor_thrust_cmd += Eigen::Vector2d(diff_force / 2.0f, -diff_force / 2.0f);

  // saturate
  motor_thrust_cmd.x() =
      std::min(std::max((double)motor_thrust_cmd.x(), -maximum_thrust), maximum_thrust);
  motor_thrust_cmd.y() =
      std::min(std::max((double)motor_thrust_cmd.y(), -maximum_thrust), maximum_thrust);

  // publish
  PublishUSVCommands(motor_thrust_cmd);
}

void USVIgnitionPlatform::PublishUSVCommands(const Eigen::Vector2d &motor_thrust_cmd) {
  std_msgs::msg::Float64 left_thrust_msg;
  std_msgs::msg::Float64 right_thrust_msg;

  left_thrust_msg.data = motor_thrust_cmd[0];
  right_thrust_msg.data = motor_thrust_cmd[1];

  left_thrust_pub_->publish(left_thrust_msg);
  right_thrust_pub_->publish(right_thrust_msg);
}

bool USVIgnitionPlatform::ownSendCommand() {
  // TODO:COMPLETE
  if (!odometry_info_received_) {
    return true;
  }
  odometry_info_received_ = false;

  if (command_twist_msg_.twist.angular.z > yaw_rate_limit_) {
    command_twist_msg_.twist.angular.z = yaw_rate_limit_;
  } else if (command_twist_msg_.twist.angular.z < -yaw_rate_limit_) {
    command_twist_msg_.twist.angular.z = -yaw_rate_limit_;
  }

  Eigen::Vector3d twist_lineal_enu =
      Eigen::Vector3d(command_twist_msg_.twist.linear.x, command_twist_msg_.twist.linear.y,
                      command_twist_msg_.twist.linear.z);

  Eigen::Vector3d twist_lineal_flu = convertENUtoFLU(yaw_, twist_lineal_enu);

  speedController(twist_lineal_flu);

  return true;
};

bool USVIgnitionPlatform::ownSetArmingState(bool state) {
  resetCommandTwistMsg();
  return true;
};

bool USVIgnitionPlatform::ownSetOffboardControl(bool offboard) {
  resetCommandTwistMsg();
  return true;
};

bool USVIgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in) {
  return true;
};

void USVIgnitionPlatform::resetCommandTwistMsg() {
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0f;
  twist_msg.linear.y = 0.0f;
  twist_msg.linear.z = 0.0f;
  twist_msg.angular.x = 0.0f;
  twist_msg.angular.y = 0.0f;
  twist_msg.angular.z = 0.0f;

  ignition_bridge_->sendTwistMsg(twist_msg);
}

void USVIgnitionPlatform::poseCallback(const geometry_msgs::msg::PoseStamped &pose_msg) {
  pose_ptr_->updateData(pose_msg);
  return;
};

void USVIgnitionPlatform::odometryCallback(const nav_msgs::msg::Odometry &odom_msg) {
  odometry_raw_estimation_ptr_->updateData(odom_msg);

  tf2::Quaternion q(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw_ = yaw;

  odometry_info_received_ = true;
  return;
};

void USVIgnitionPlatform::cameraCallback(const sensor_msgs::msg::Image &image_msg,
                                         const std::string &sensor_name) {
  (callbacks_camera_.find(sensor_name)->second).updateData(image_msg);
  return;
};

void USVIgnitionPlatform::cameraInfoCallback(const sensor_msgs::msg::CameraInfo &info_msg,
                                             const std::string &sensor_name) {
  (callbacks_camera_.find(sensor_name)->second).setParameters(info_msg);
  return;
};

void USVIgnitionPlatform::laserScanCallback(const sensor_msgs::msg::LaserScan &laser_scan_msg,
                                            const std::string &sensor_name) {
  (callbacks_laser_scan_.find(sensor_name)->second).updateData(laser_scan_msg);
  return;
};

void USVIgnitionPlatform::pointCloudCallback(const sensor_msgs::msg::PointCloud2 &point_cloud_msg,
                                             const std::string &sensor_name) {
  (callbacks_point_cloud_.find(sensor_name)->second).updateData(point_cloud_msg);
  return;
};

// Convert from ENU (east, north, up) to local FLU (forward, left, up)
Eigen::Vector3d USVIgnitionPlatform::convertENUtoFLU(const float yaw_angle,
                                                     Eigen::Vector3d &enu_vec) {
  // Convert from ENU to FLU
  Eigen::Matrix3d R_FLU;
  R_FLU << cos(yaw_angle), sin(yaw_angle), 0, -sin(yaw_angle), cos(yaw_angle), 0, 0, 0, 1;

  return R_FLU * enu_vec;
}
}  // namespace ignition_platform

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

namespace ignition_platform
{
  bool USVIgnitionPlatform::odometry_info_received_ = false;
  geometry_msgs::msg::Quaternion USVIgnitionPlatform::self_orientation_ = geometry_msgs::msg::Quaternion();

  std::unique_ptr<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>> USVIgnitionPlatform::pose_ptr_ = nullptr;
  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> USVIgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

  std::unordered_map<std::string, as2::sensors::Camera> USVIgnitionPlatform::callbacks_camera_ = {};
  std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::LaserScan>> USVIgnitionPlatform::callbacks_laser_scan_ = {};
  std::unordered_map<std::string, as2::sensors::Sensor<sensor_msgs::msg::PointCloud2>> USVIgnitionPlatform::callbacks_point_cloud_ = {};

  USVIgnitionPlatform::USVIgnitionPlatform() : as2::AerialPlatform()
  {
    
    ignition_bridge_ = std::make_shared<IgnitionBridge>(this->get_namespace());
    updateGains();

    parameters_read = false;
    static auto parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&USVIgnitionPlatform::parametersCallback, this, std::placeholders::_1));

    this->ownDeclareParameters();

    configureSensors();

    // Timer to send command
    static auto timer_commands_ =
        this->create_wall_timer(
            std::chrono::milliseconds(CMD_FREQ),
            [this]()
            { this->sendCommand(); });
  };

  std::vector<std::string> split(const std::string &s, char delim)
  {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
      elems.emplace_back(item);
    }
    return elems;
  };

  void USVIgnitionPlatform::configureSensors()
  {
    pose_ptr_ =
        std::make_unique<as2::sensors::Sensor<geometry_msgs::msg::PoseStamped>>("pose", this);
    ignition_bridge_->setPoseCallback(poseCallback);

    odometry_raw_estimation_ptr_ =
        std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
    ignition_bridge_->setOdometryCallback(odometryCallback);

    std::string sensors_param = this->get_parameter("sensors").as_string();
    std::vector<std::string> sensor_config_list = split(sensors_param, ':');

    for (auto sensor_config : sensor_config_list)
    {
      std::vector<std::string> sensor_config_params = split(sensor_config, ',');

      if (sensor_config_params.size() != 5)
      {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Wrong sensor configuration: %s",
                          sensor_config.c_str());
        continue;
      }

      std::string sensor_type = sensor_config_params[4];
      if (sensor_type == "camera")
      {
        as2::sensors::Camera camera = as2::sensors::Camera(sensor_config_params[2], this);
        callbacks_camera_.insert(std::make_pair(sensor_config_params[2],
                                                camera));

        ignition_bridge_->addSensor(
            sensor_config_params[0],
            sensor_config_params[1],
            sensor_config_params[2],
            sensor_config_params[3],
            sensor_config_params[4],
            cameraCallback,
            cameraInfoCallback);
      }
      else if (sensor_type == "lidar")
      {
        as2::sensors::Sensor<sensor_msgs::msg::LaserScan> laser_scan_sensor(sensor_config_params[2], this);
        as2::sensors::Sensor<sensor_msgs::msg::PointCloud2> point_cloud_sensor(sensor_config_params[2] + "/points", this);

        callbacks_laser_scan_.insert(std::make_pair(sensor_config_params[2], laser_scan_sensor));
        callbacks_point_cloud_.insert(std::make_pair(sensor_config_params[2], point_cloud_sensor));

        ignition_bridge_->addSensor(
            sensor_config_params[0],
            sensor_config_params[1],
            sensor_config_params[2],
            sensor_config_params[3],
            sensor_config_params[4],
            laserScanCallback,
            pointCloudCallback);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Sensor type not supported: %s", sensor_type.c_str());
      }
    }
    return;
  };

  double angleMinError(
    const double &yaw_angle_state,
    const double &yaw_angle_ref)
  {
    // Wrap angles to [-pi, pi]
    
    double yaw_angle_ref_wrap = yaw_angle_ref;
    if (yaw_angle_ref_wrap < -M_PI)
    {
        yaw_angle_ref_wrap += 2.0 * M_PI;
    }
    else if (yaw_angle_ref_wrap > M_PI)
    {
        yaw_angle_ref_wrap -= 2.0 * M_PI;
    }

    double yaw_angle_state_wrap = yaw_angle_state;
    if (yaw_angle_state_wrap < -M_PI)
    {
        yaw_angle_state_wrap += 2.0 * M_PI;
    }
    else if (yaw_angle_state_wrap > M_PI)
    {
        yaw_angle_state_wrap -= 2.0 * M_PI;
    }
    
    // Compute yaw angle error in rad
    double yaw_angle_diff = yaw_angle_ref_wrap - yaw_angle_state;

    // Wrap angle error to [-pi, pi]
    if (yaw_angle_diff < -M_PI)
    {
        return yaw_angle_diff + 2.0 * M_PI;
    }
    else if (yaw_angle_diff > M_PI)
    {
        return yaw_angle_diff - 2.0 * M_PI;
    }
    return yaw_angle_diff;
  }

  bool USVIgnitionPlatform::ownSendCommand()
  {
    if (!parameters_read)
    {
      RCLCPP_WARN(this->get_logger(), "Parameters not read yet");
      return false;
    }

    if (control_in_.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME)
    {
      if (!odometry_info_received_)
      {
        return false;
      }
      odometry_info_received_ = false;

      if (command_twist_msg_.twist.angular.z > yaw_rate_limit_)
      {
        command_twist_msg_.twist.angular.z = yaw_rate_limit_;
      }
      else if (command_twist_msg_.twist.angular.z < -yaw_rate_limit_)
      {
        command_twist_msg_.twist.angular.z = -yaw_rate_limit_;
      }
      Eigen::Vector3d twist_lineal_enu = Eigen::Vector3d(command_twist_msg_.twist.linear.x,
                                                         command_twist_msg_.twist.linear.y,
                                                         command_twist_msg_.twist.linear.z);

      Eigen::Vector3d twist_lineal_flu = as2::FrameUtils::convertENUtoFLU(self_orientation_, twist_lineal_enu);

      speedController(twist_lineal_flu);
      sendUSVMsg();
    }
    else if (control_in_.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME)
    {
      Eigen::Vector3d twist_lineal_flu = Eigen::Vector3d(command_twist_msg_.twist.linear.x,
                                                         command_twist_msg_.twist.linear.y,
                                                         command_twist_msg_.twist.linear.z);
      speedController(twist_lineal_flu);
      sendUSVMsg();
    }
    return true;
  };

  // Return yaw speed in rad/s to reach a reference yaw angle
  double USVIgnitionPlatform::computeYawSpeed(
        const double &yaw_angle_error,
        const double &dt)
  {

    // Compute the proportional error (yaw error)
    double yaw_p_error = yaw_angle_error;

    // Store the error for the next iteration
    static double last_yaw_p_error_ = yaw_p_error;
    static double filtered_d_yaw_error_ = yaw_p_error;

    // Compute the error of the derivative filtered with a first order filter (yaw derivate)
    double yaw_error_incr = (yaw_p_error - last_yaw_p_error_);
    filtered_d_yaw_error_ = alpha_ * yaw_error_incr + (1.0 - alpha_) * filtered_d_yaw_error_;

    // Update de acumulated error
    yaw_accum_error_ += yaw_p_error * dt;

    // Compute anti-windup. Limit integral contribution
    float antiwindup_value = antiwindup_cte_ / yaw_ang_mat_[1];
    yaw_accum_error_ = (yaw_accum_error_ >  antiwindup_value) ?  antiwindup_value : yaw_accum_error_;
    yaw_accum_error_ = (yaw_accum_error_ < -antiwindup_value) ? -antiwindup_value : yaw_accum_error_;

    // Compute desired acceleration
    return yaw_ang_mat_[0] * yaw_p_error + yaw_ang_mat_[1] * yaw_accum_error_ + yaw_ang_mat_[2] * filtered_d_yaw_error_;
  };

  void USVIgnitionPlatform::speedController(const Eigen::Vector3d &vel_flu)
  {
    rclcpp::Time current_time = this->now();
    static rclcpp::Time last_time_ = current_time;
    double dt = (current_time - last_time_).nanoseconds() / 1.0e9;
    last_time_ = current_time;

    Eigen::Vector2d motor_thrust_cmd = Eigen::Vector2d::Zero();
    Eigen::Vector2d motor_pos_cmd = Eigen::Vector2d::Zero();
    double desired_torque = 0.0;

    // If speed norm > 0.5, move with desired yaw
    // Get yaw rate (rad/s)
    double desired_yaw_flu = as2::FrameUtils::getVector2DAngle(vel_flu.x(), vel_flu.y());

    // double desired_yaw_rate = command_twist_msg_.twist.angular.z;
    
    if (vel_flu.norm() > 1.0)
    {
      // Get yaw rate (rad/s)
      double desired_yaw_rate = K_yaw_rate_ * computeYawSpeed(desired_yaw_flu, dt);
      // RCLCPP_INFO(this->get_logger(), "desired_yaw_flu: %f", desired_yaw_flu);

      // Convert yaw rate (rad/s) to torque (Nm)
      desired_torque = K_yaw_force_ * desired_yaw_rate;

      // Move if velocity in y axis is less than 1.0 m/s. Otherwise, just rotate
      if (abs(vel_flu.y()) < 1.0)
      {
        Eigen::Vector2d desired_force = Eigen::Vector2d(vel_flu.x(), vel_flu.y()) * GainThrust_;
        motor_thrust_cmd << desired_force.x() / 2.0f, desired_force.x() / 2.0f;
      }
    }
    else if (vel_flu.norm() > 0.1)
    {
      Eigen::Vector2d desired_force = Eigen::Vector2d(vel_flu.x(), vel_flu.y()) * GainThrust_;
      motor_thrust_cmd << desired_force.x() / 2.0f, desired_force.x() / 2.0f;

      motor_pos_cmd.x() = desired_yaw_flu;
      motor_pos_cmd.y() = desired_yaw_flu;
    }

    // double desired_yaw_rate = command_twist_msg_.twist.angular.z;

    // If speed norm < 0.1, stay still
    // If vellocity in y axis > 0.5, just rotate
    // Else, move
    // if (vel_flu.norm() > 1.0)
    // {
    //   if (vel_flu.y() < 0.5)
    //   {
    //     // Convert speed (m/s) to force (N)
    //     Eigen::Vector2d desired_force = Eigen::Vector2d(vel_flu.x(), vel_flu.y()) * GainThrust_;
    //     motor_thrust_cmd << desired_force.x() / 2.0f, desired_force.x() / 2.0f;
    //   }
      
      // // Get yaw rate (rad/s)
      // double desired_yaw_flu = as2::FrameUtils::getVector2DAngle(vel_flu.x(), vel_flu.y());
      // double desired_yaw_rate = K_yaw_rate_ * computeYawSpeed(desired_yaw_flu, dt);

      // // Convert yaw rate (rad/s) to torque (Nm)
      // desired_torque = K_yaw_force_ * desired_yaw_rate;

      // RCLCPP_INFO(this->get_logger(), "Desired_yaw_rate: %f", desired_yaw_rate);
    // }

    // RCLCPP_INFO(this->get_logger(), "Desired motor force: %f, %f", motor_thrust_cmd.x(), motor_thrust_cmd.y());
    // RCLCPP_INFO(this->get_logger(), "Desired yaw torque: %f", desired_torque);

    motor_thrust_cmd += Eigen::Vector2d(-desired_torque / 2.0f, desired_torque / 2.0f);

    // RCLCPP_INFO(this->get_logger(), "Desired motor force rot: %f, %f", motor_thrust_cmd.x(), motor_thrust_cmd.y());

    // Saturate max/min force
    motor_thrust_cmd.x() =
        std::min(std::max((double)motor_thrust_cmd.x(), -maximum_thrust_), maximum_thrust_);
    motor_thrust_cmd.y() =
        std::min(std::max((double)motor_thrust_cmd.y(), -maximum_thrust_), maximum_thrust_);

    motor_thrust_cmd_ = motor_thrust_cmd;
    motor_pos_cmd_ = motor_pos_cmd;

    // RCLCPP_INFO(this->get_logger(), "Send motor force: %f, %f", motor_thrust_cmd.x(), motor_thrust_cmd.y());
    // RCLCPP_INFO(this->get_logger(), "Send motor rot: %f, %f", motor_pos_cmd.x(), motor_pos_cmd.y());
    // RCLCPP_INFO(this->get_logger(), "\n");
  }

  void USVIgnitionPlatform::sendUSVMsg()
  {
    std_msgs::msg::Float64 left_thrust_msg;
    std_msgs::msg::Float64 right_thrust_msg;

    left_thrust_msg.data = motor_thrust_cmd_[0];
    right_thrust_msg.data = motor_thrust_cmd_[1];

    ignition_bridge_->sendThrustMsg(left_thrust_msg, right_thrust_msg);

    std_msgs::msg::Float64 left_pos_msg;
    std_msgs::msg::Float64 right_pos_msg;

    left_pos_msg.data = motor_pos_cmd_[0];
    right_pos_msg.data = motor_pos_cmd_[1];

    ignition_bridge_->sendRotationMsg(left_pos_msg, right_pos_msg);
  }

  bool USVIgnitionPlatform::ownSetArmingState(bool state)
  {
    resetCommandMsg();
    return true;
  };

  bool USVIgnitionPlatform::ownSetOffboardControl(bool offboard)
  {
    resetCommandMsg();
    return true;
  };

  void USVIgnitionPlatform::resetCommandMsg()
  {
    motor_thrust_cmd_ = Eigen::Vector2d(0, 0);
  }

  bool USVIgnitionPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &control_in)
  {
    if (control_in.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED &&
        control_in.control_mode == as2_msgs::msg::ControlMode::SPEED &&
        (control_in.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME ||
         control_in.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME))
    {
      control_in_ = control_in;
      resetCommandMsg();
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "IgnitionPlatform::ownSetPlatformControlMode() - unsupported control mode");
    return false;
  };

  void USVIgnitionPlatform::ownDeclareParameters()
  {
    this->declare_parameter<std::string>("sensors");

    std::vector<std::string> params_to_declare(parameters_to_read_);
    for (int i=0; i<params_to_declare.size(); i++)
    {
      this->declare_parameter<double>(params_to_declare[i]);
    }
    return;
  };

  rcl_interfaces::msg::SetParametersResult USVIgnitionPlatform::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (auto &param : parameters)
    {
      if (parameters_.count(param.get_name()) == 1)
      {
        parameters_[param.get_name()] = param.get_value<double>();
        updateGains();

        // Remove the parameter from the list of parameters to be read
        parameters_to_read_.erase(
            std::remove(
                parameters_to_read_.begin(),
                parameters_to_read_.end(),
                param.get_name()),
            parameters_to_read_.end());
        if (parameters_to_read_.empty())
        {
          RCLCPP_DEBUG(this->get_logger(), "All parameters read");
          parameters_read = true;
        }
      }
      else if (param.get_name() == "sensors")
      {
        continue;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Parameter %s not defined in controller params", param.get_name().c_str());
        result.successful = false;
        result.reason = "parameter not found";
      }
    }
    return result;
  };

  void USVIgnitionPlatform::updateGains()
  {
    RCLCPP_DEBUG(this->get_logger(), "Update gains");
    yaw_rate_limit_ = parameters_["yaw_rate_limit"];
    K_yaw_rate_ = parameters_["K_yaw_rate"];
    K_yaw_force_ = parameters_["K_yaw_force"];
    GainThrust_ = parameters_["GainThrust"];
    maximum_thrust_ = parameters_["maximum_thrust"];

    alpha_ = parameters_["alpha"];
    antiwindup_cte_ = parameters_["antiwindup_cte"];
    yaw_ang_mat_ = Vector3d(
                      parameters_["yaw_speed_controller.Kp"],
                      parameters_["yaw_speed_controller.Ki"],
                      parameters_["yaw_speed_controller.Kd"]);
    return;
  }

  void USVIgnitionPlatform::poseCallback(const geometry_msgs::msg::PoseStamped &pose_msg)
  {
    pose_ptr_->updateData(pose_msg);
    return;
  };

  void USVIgnitionPlatform::odometryCallback(const nav_msgs::msg::Odometry &odom_msg)
  {
    nav_msgs::msg::Odometry odom_enu = odom_msg;
    Vector3d twist_flu = Eigen::Vector3d(
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z);

    Eigen::Vector3d twist_enu = as2::FrameUtils::convertFLUtoENU(odom_msg.pose.pose.orientation, twist_flu);

    odom_enu.twist.twist.linear.x = twist_enu(0);
    odom_enu.twist.twist.linear.y = twist_enu(1);
    odom_enu.twist.twist.linear.z = twist_enu(2);

    odometry_raw_estimation_ptr_->updateData(odom_enu);

    self_orientation_ = odom_msg.pose.pose.orientation;

    odometry_info_received_ = true;
    return;
  };

  void USVIgnitionPlatform::cameraCallback(const sensor_msgs::msg::Image &image_msg,
                                           const std::string &sensor_name)
  {
    (callbacks_camera_.find(sensor_name)->second).updateData(image_msg);
    return;
  };

  void USVIgnitionPlatform::cameraInfoCallback(const sensor_msgs::msg::CameraInfo &info_msg,
                                               const std::string &sensor_name)
  {
    (callbacks_camera_.find(sensor_name)->second).setParameters(info_msg);
    return;
  };

  void USVIgnitionPlatform::laserScanCallback(const sensor_msgs::msg::LaserScan &laser_scan_msg,
                                              const std::string &sensor_name)
  {
    (callbacks_laser_scan_.find(sensor_name)->second).updateData(laser_scan_msg);
    return;
  };

  void USVIgnitionPlatform::pointCloudCallback(const sensor_msgs::msg::PointCloud2 &point_cloud_msg,
                                               const std::string &sensor_name)
  {
    (callbacks_point_cloud_.find(sensor_name)->second).updateData(point_cloud_msg);
    return;
  };

} // namespace ignition_platform

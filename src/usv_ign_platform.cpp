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
  std::shared_ptr<IgnitionBridge> USVIgnitionPlatform::ignition_bridge_ = nullptr;
  bool USVIgnitionPlatform::odometry_info_received_ = false;
  bool USVIgnitionPlatform::imu_info_received_ = false;
  geometry_msgs::msg::Quaternion USVIgnitionPlatform::self_orientation_ = geometry_msgs::msg::Quaternion();
  std::string USVIgnitionPlatform::namespace_ = "";

  std::unique_ptr<as2::sensors::Sensor<nav_msgs::msg::Odometry>> USVIgnitionPlatform::odometry_raw_estimation_ptr_ = nullptr;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr USVIgnitionPlatform::ground_truth_pose_pub_ = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr USVIgnitionPlatform::ground_truth_twist_pub_ = nullptr;

  std::unique_ptr<sensor_msgs::msg::Imu> USVIgnitionPlatform::imu_msg_ = nullptr;

  USVIgnitionPlatform::USVIgnitionPlatform() : as2::AerialPlatform()
  {
    this->declare_parameter<std::string>("world");
    std::string world_param = this->get_parameter("world").as_string();

    namespace_ = this->get_namespace();
    ignition_bridge_ = std::make_shared<IgnitionBridge>(namespace_, world_param);

    updateGains();

    left_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("left/thrust/cmd_thrust",
                                                                      rclcpp::QoS(1));
    right_thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("right/thrust/cmd_thrust",
                                                                       rclcpp::QoS(1));

    left_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("left/thrust/joint/cmd_pos",
                                                                      rclcpp::QoS(1));
    right_pos_pub_ = this->create_publisher<std_msgs::msg::Float64>("right/thrust/joint/cmd_pos",
                                                                       rclcpp::QoS(1));

    ground_truth_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        as2_names::topics::ground_truth::pose,
        as2_names::topics::ground_truth::qos);

    ground_truth_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::ground_truth::twist,
        as2_names::topics::ground_truth::qos);

    this->declare_parameter<std::string>("imu_topic");
    std::string imu_topic_param = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        this->generate_global_name(imu_topic_param),
        as2_names::topics::sensor_measurements::qos,
        std::bind(&USVIgnitionPlatform::imuCallback, this, std::placeholders::_1));


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
        this->declare_parameter<bool>("use_odom_plugin");
        bool use_odom_plugin_param = this->get_parameter("use_odom_plugin").as_bool();
        if (use_odom_plugin_param)
        {
            odometry_raw_estimation_ptr_ =
                std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>(
                    "odom", this);
            ignition_bridge_->setOdometryCallback(odometryCallback);
        }

        this->declare_parameter<bool>("use_ground_truth");
        bool use_ground_truth_param = this->get_parameter("use_ground_truth").as_bool();
        if (use_ground_truth_param)
        {
            ignition_bridge_->setGroundTruthCallback(groundTruthCallback);
        }

        parameters_read = false;
        static auto parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&USVIgnitionPlatform::parametersCallback, this, std::placeholders::_1));

        this->ownDeclareParameters();
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
    yaw_accum_error_ = (yaw_accum_error_ > antiwindup_value) ? antiwindup_value : yaw_accum_error_;
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

    // ignition_bridge_->sendThrustMsg(left_thrust_msg, right_thrust_msg);
    left_thrust_pub_->publish(left_thrust_msg);
    right_thrust_pub_->publish(right_thrust_msg);

    std_msgs::msg::Float64 left_pos_msg;
    std_msgs::msg::Float64 right_pos_msg;

    left_pos_msg.data = motor_pos_cmd_[0];
    right_pos_msg.data = motor_pos_cmd_[1];

    left_pos_pub_->publish(left_pos_msg);
    right_pos_pub_->publish(right_pos_msg);
    // ignition_bridge_->sendRotationMsg(left_pos_msg, right_pos_msg);
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
    std::vector<std::string> params_to_declare(parameters_to_read_);
    for (int i = 0; i < params_to_declare.size(); i++)
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
        // result.successful = true;
        // result.reason = "parameter not found";
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
  };



    void USVIgnitionPlatform::odometryCallback(nav_msgs::msg::Odometry &odom_msg)
    {
        odom_msg.header.frame_id = generateTfName(namespace_, "odom");

        odometry_raw_estimation_ptr_->updateData(odom_msg);

        self_orientation_ = odom_msg.pose.pose.orientation;
        odometry_info_received_ = true;
        return;
    };

    void USVIgnitionPlatform::groundTruthCallback(geometry_msgs::msg::Pose &pose_msg)
    {
        if (!imu_info_received_)
        {
            RCLCPP_WARN(rclcpp::get_logger("as2_ignition_platform"), "IMU not received yet");
            return;
        }

        geometry_msgs::msg::Pose last_pose;
        geometry_msgs::msg::Twist twist_msg_enu;

        // Derive twist from ground truth
        static auto last_time = rclcpp::Clock().now();
        auto current_time = rclcpp::Clock().now();
        auto dt = (current_time - last_time).seconds();
        last_time = current_time;

        static auto last_pose_msg = pose_msg;
        geometry_msgs::msg::Pose delta_pose;
        delta_pose.position.x = pose_msg.position.x - last_pose_msg.position.x;
        delta_pose.position.y = pose_msg.position.y - last_pose_msg.position.y;
        delta_pose.position.z = pose_msg.position.z - last_pose_msg.position.z;
        last_pose_msg = pose_msg;

        static auto last_vx = delta_pose.position.x / dt;
        static auto last_vy = delta_pose.position.y / dt;
        static auto last_vz = delta_pose.position.z / dt;

        const double alpha = 0.1f ;

        twist_msg_enu.linear.x = alpha * (delta_pose.position.x / dt) + (1 - alpha) * last_vx;
        twist_msg_enu.linear.y = alpha * (delta_pose.position.y / dt) + (1 - alpha) * last_vy;
        twist_msg_enu.linear.z = alpha * (delta_pose.position.z / dt) + (1 - alpha) * last_vz;

        last_vx = twist_msg_enu.linear.x;
        last_vy = twist_msg_enu.linear.y;
        last_vz = twist_msg_enu.linear.z;

        // Set angular velocity in ENU frame
        twist_msg_enu.angular = imu_msg_->angular_velocity;

        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.frame_id = generateTfName(namespace_, "map");
        pose_stamped_msg.header.stamp = rclcpp::Clock().now();
        pose_stamped_msg.pose = pose_msg;
        ground_truth_pose_pub_->publish(pose_stamped_msg);

        geometry_msgs::msg::TwistStamped twist_stamped_msg;
        twist_stamped_msg.header.frame_id = generateTfName(namespace_, "map");
        twist_stamped_msg.header.stamp = rclcpp::Clock().now();
        twist_stamped_msg.twist = twist_msg_enu;
        ground_truth_twist_pub_->publish(twist_stamped_msg);

        self_orientation_ = pose_msg.orientation;
        odometry_info_received_ = true;
        return;
    };

    void USVIgnitionPlatform::imuCallback(const sensor_msgs::msg::Imu &msg)
    {
      if (!imu_info_received_){
        imu_msg_ = std::make_unique<sensor_msgs::msg::Imu>(msg);
        imu_info_received_ = true;
      } 
      imu_msg_->angular_velocity = msg.angular_velocity;
      imu_msg_->linear_acceleration = msg.linear_acceleration;
      imu_msg_->orientation = msg.orientation;
      imu_msg_->header.stamp = msg.header.stamp;
      imu_msg_->header.frame_id = msg.header.frame_id;
      return;
    }

} // namespace ignition_platform

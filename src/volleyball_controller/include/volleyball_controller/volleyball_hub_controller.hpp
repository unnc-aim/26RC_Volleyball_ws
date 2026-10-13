//
// Created by txy on 7/8/26.
//

#ifndef VOLLEYBALL_HUB_CONTROLLER_HPP
#define VOLLEYBALL_HUB_CONTROLLER_HPP


#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <std_msgs/msg/string.hpp>

#include "custom_msgs/msg/read_sbusrc.hpp"
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/write_lk_motor_torque_control.hpp"
#include "volleyball_controller/msg/stewart_control.hpp"
#include "chassis_controllers/msg/chassis_control.hpp"
#include "custom_msgs/msg/read_dm_motor.hpp"
#include "custom_msgs/msg/write_dm_motor_position_control_with_speed_limit.hpp"
#include "custom_msgs/msg/write_on_board_pwm.hpp"

#include "universal_utils/pid.hpp"

using namespace std::chrono_literals;
using namespace aim::algorithm::pid;

enum class PushState
{
    WAITING,
    GOING_UP,
    RESETTING
};

enum class StrikeState
{
    WAITING, // 准备发球
    PUSHING, // 上抛
    LAYING_DOWN, // 大pitch移走
    GOING_DOWN, // 向下挥拍
    GOING_UP, // 继续挥拍
    DONE // 发球结束
};

enum class ControlMode
{
    MANUAL = 0,
    VISION_ASSIST = 1,
    FULL_AUTO = 2
};

struct RCConstants
{
    enum class RCChannel : size_t
    {
        LEFT_X = 0, // CH1
        LEFT_Y = 1, // CH2
        RIGHT_X = 2, // CH3
        RIGHT_Y = 3, // CH4
        SWITCH_SERVE_2POS = 4, // CH5: 2段开关发球 (高:准备/复位, 低:单次发球)
        SWITCH_ESTOP_BTN = 5, // CH6: 按钮急停
        SWITCH_Z_STRIKE = 6, // CH7: 2段开关 Z轴单次击打
        SWITCH_STEWART_TILT = 7, // CH8: 3段开关 Stewart倾斜 (下:左倾, 中:回中, 上:右倾)
        BACK_R = 9,
        R_DIAL = 11,
        L_DIAL = 12,
    };

    static constexpr int16_t MID_VALUE = 992;
    static constexpr double SCALE = 819;
    static constexpr int16_t SW_DOWN_THRESHOLD = 400;
    static constexpr int16_t SW_UP_THRESHOLD = 1500;
    static constexpr double TILT_ANGLE = 0.5;
};


class VolleyballHub : public rclcpp::Node
{
public:
    explicit VolleyballHub(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("volleyball_hub", options)
    {
        this->declare_parameter<std::string>("sbusrc_read_topic", "/sbus_rc");
        this->declare_parameter<std::string>("chassis_control_write_topic", "/chassis_command");
        this->declare_parameter<std::string>("gimbal_control_write_topic", "/stewart_command");

        this->declare_parameter<std::string>("pitch_motor_read_topic", "/dm_motor_read");
        this->declare_parameter<std::string>("pitch_motor_write_topic", "/dm_motor_write");
        this->declare_parameter<std::string>("strike_motor_read_topic", "/lk_motor_read");
        this->declare_parameter<std::string>("strike_motor_write_topic", "/lk_motor_write");
        this->declare_parameter<std::string>("pwm_write_topic", "/pwm");

        strike_speed_pid.set_deadband(300);
        chassis_command_.extra_info.resize(1);
        chassis_command_.extra_info[0] = 0;

        rc_sub_ =
            this->create_subscription<
                custom_msgs::msg::ReadSBUSRC>(
                this->get_parameter("sbusrc_read_topic").as_string(),
                rclcpp::SensorDataQoS(),
                [this](custom_msgs::msg::ReadSBUSRC::SharedPtr msg)
                {
                    this->rc_ = *msg;
                    if (!this->rc_received_ && rc_.channels[4] > 1000)
                    {
                        strike_flag_ = true;
                    }
                    this->rc_received_ = true;
                });

        pitch_motor_sub_ =
            this->create_subscription<
                custom_msgs::msg::ReadDmMotor>(
                this->get_parameter("pitch_motor_read_topic").as_string(),
                rclcpp::SensorDataQoS(),
                [this](custom_msgs::msg::ReadDmMotor::SharedPtr msg)
                {
                    this->pitch_motor_status_ = *msg;
                }
            );

        strike_motor_sub_ =
            this->create_subscription<
                custom_msgs::msg::ReadLkMotor>(
                this->get_parameter("strike_motor_read_topic").as_string(),
                rclcpp::SensorDataQoS(),
                [this](custom_msgs::msg::ReadLkMotor::SharedPtr msg)
                {
                    this->strike_motor_status_ = *msg;
                }
            );

        chassis_command_pub_ =
            this->create_publisher<
                chassis_controllers::msg::ChassisControl>(
                this->get_parameter("chassis_control_write_topic").as_string(),
                rclcpp::SensorDataQoS()
            );

        gimbal_command_pub_ =
            this->create_publisher<
                volleyball_controller::msg::StewartControl>(
                this->get_parameter("gimbal_control_write_topic").as_string(),
                rclcpp::SensorDataQoS()
            );

        strike_motor_pub_ =
            this->create_publisher<
                custom_msgs::msg::WriteLkMotorTorqueControl>(
                this->get_parameter("strike_motor_write_topic").as_string(),
                rclcpp::SensorDataQoS()
            );

        pitch_motor_pub_ =
            this->create_publisher<
                custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>(
                this->get_parameter("pitch_motor_write_topic").as_string(),
                rclcpp::SensorDataQoS()
            );

        pwm_pub_ =
            this->create_publisher<
                custom_msgs::msg::WriteOnBoardPWM>(
                this->get_parameter("pwm_write_topic").as_string(),
                rclcpp::SensorDataQoS()
            );

        control_timer_ =
            this->create_wall_timer(
                1ms,
                [this] { control_loop(); });

        RCLCPP_INFO(this->get_logger(), "Volleyball hub controller started");
    }

    void emergency_stop()
    {
        pwm_command_.channel1 = 0;
        pwm_command_.channel2 = 0;
        pwm_command_.channel3 = 0;
        pwm_command_.channel4 = 0;
        chassis_command_.emergency_stop = 1;
        chassis_command_.x_speed = 0.0;
        chassis_command_.y_speed = 0.0;
        chassis_command_.spin_speed = 0.0;
        chassis_command_.extra_info[0] = 1;
        gimbal_command_.emergency_stop = 1;
        gimbal_command_.x = 0.0;
        gimbal_command_.y = 0.0;
        gimbal_command_.z = 0.22186;
        gimbal_command_.yaw = 0.0;
        gimbal_command_.pitch = 0.0;
        gimbal_command_.roll = 0.0;
        strike_motor_command_.enable = 0;
        strike_motor_command_.torque = 0.0;
        pitch_motor_command_.enable = 0;
        pitch_motor_command_.speed = 0.0;
        pitch_motor_command_.position = 0.0;

        pwm_pub_->publish(pwm_command_);
        chassis_command_pub_->publish(chassis_command_);
        gimbal_command_pub_->publish(gimbal_command_);
        strike_motor_pub_->publish(strike_motor_command_);
        pitch_motor_pub_->publish(pitch_motor_command_);
    }

private:
    constexpr static double MAX_CHASSIS_SPEED = 3.0;
    constexpr static double MAX_CHASSIS_SPIN_SPEED = 3.0;
    constexpr static int ready_position = 35000;
    constexpr static int STRIKE_MOTOR_ENCODER_MAX = 65536;
    constexpr static double PITCH_MIN = -0.6;
    constexpr static double PITCH_MAX = 0;
    constexpr static int pwm_up = 1100;
    constexpr static int pwm_down = 1900;

    constexpr static double roll_range = 0.12;
    constexpr static double pitch_range = 0.16;
    constexpr static double z_min = 0.2;
    constexpr static double z_min_push = 0.18;
    constexpr static double z_max = 0.36;
    constexpr static double push_going_up_timeout = 0.18;
    constexpr static double push_going_down_timeout = 0.15;
    constexpr static double strike_delay_time = 0.0;

    bool rc_received_{false};
    ControlMode control_mode_{ControlMode::MANUAL};
    StrikeState strike_state_{StrikeState::WAITING};
    PushState push_state_{PushState::WAITING};
    double push_base_z_{z_min};

    bool strike_flag_{false};
    bool strike_push_flag_{false};
    bool push_flag_{false};
    rclcpp::Time push_last_action_time_{now()};
    rclcpp::Time strike_ready_time_{0}; // stewart倒下去的时间

    constexpr static float strike_speed_pid_kp = 0.16;
    constexpr static float strike_speed_pid_ki = 0.3;
    constexpr static float strike_speed_pid_kd = 0.04;
    constexpr static float strike_speed_pid_maxout = 2048.0;
    constexpr static float strike_speed_pid_maxiout = 300.0;
    PID strike_speed_pid{
        strike_speed_pid_kp, strike_speed_pid_ki, strike_speed_pid_kd, strike_speed_pid_maxout, strike_speed_pid_maxiout
    };

    constexpr static float strike_angle_pid_kp = 0.5;
    constexpr static float strike_angle_pid_ki = 0.0;
    constexpr static float strike_angle_pid_kd = 10.0;
    constexpr static float strike_angle_pid_maxout = 5000;
    constexpr static float strike_angle_pid_maxiout = 0.0;
    PID strike_angle_pid{
        strike_angle_pid_kp, strike_angle_pid_ki, strike_angle_pid_kd, strike_angle_pid_maxout, strike_angle_pid_maxiout
    };

    static custom_msgs::msg::ReadSBUSRC rc_;
    static custom_msgs::msg::ReadDmMotor pitch_motor_status_;
    static custom_msgs::msg::ReadLkMotor strike_motor_status_;
    static custom_msgs::msg::WriteLkMotorTorqueControl strike_motor_command_;
    static custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit pitch_motor_command_;
    static custom_msgs::msg::WriteOnBoardPWM pwm_command_;
    static chassis_controllers::msg::ChassisControl chassis_command_;
    static volleyball_controller::msg::StewartControl gimbal_command_;

    rclcpp::Subscription<
        custom_msgs::msg::ReadLkMotor>::SharedPtr
    strike_motor_sub_;

    rclcpp::Subscription<
        custom_msgs::msg::ReadDmMotor>::SharedPtr
    pitch_motor_sub_;

    rclcpp::Subscription<
        custom_msgs::msg::ReadSBUSRC>::SharedPtr
    rc_sub_;

    rclcpp::Publisher<
        chassis_controllers::msg::ChassisControl>::SharedPtr
    chassis_command_pub_;

    rclcpp::Publisher<
        volleyball_controller::msg::StewartControl>::SharedPtr
    gimbal_command_pub_;

    rclcpp::Publisher<
        custom_msgs::msg::WriteLkMotorTorqueControl>::SharedPtr
    strike_motor_pub_;

    rclcpp::Publisher<
        custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit>::SharedPtr
    pitch_motor_pub_;

    rclcpp::Publisher<
        custom_msgs::msg::WriteOnBoardPWM>::SharedPtr
    pwm_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    void control_loop();

    int static calculate_difference(int current_angle, int target_angle)
    {
        const int total_positions = 65536;
        int clockwise_difference = (target_angle - current_angle + total_positions) % total_positions;
        int counterclockwise_difference = (current_angle - target_angle + total_positions) % total_positions;
        if (clockwise_difference <= counterclockwise_difference)
        {
            return clockwise_difference;
        }
        else
        {
            return -counterclockwise_difference;
        }
    }

    void calculate_strike_motor_command(int target_pos)
    {
        int backward_distance = (strike_motor_status_.encoder - target_pos + STRIKE_MOTOR_ENCODER_MAX) %
            STRIKE_MOTOR_ENCODER_MAX;
        double error = -backward_distance;
        // within short distance, allow both directiono
        double short_distance_error = calculate_difference(strike_motor_status_.encoder, target_pos);
        if (std::abs(short_distance_error) < 10000)
        {
            error = -short_distance_error;
        }
        strike_motor_command_.torque = strike_speed_pid.calculate(strike_motor_status_.speed,
                                                                  strike_angle_pid.calculate(error, 0));
    }
};

#endif //VOLLEYBALL_HUB_CONTROLLER_HPP

//
// Created by txy on 7/8/26.
//

#ifndef LK_STEWART_CONTROLLER_HPP
#define LK_STEWART_CONTROLLER_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <array>
#include "custom_msgs/msg/read_lk_motor.hpp"
#include "custom_msgs/msg/write_lk_motor_multi_round_position_control_with_speed_limit.hpp"
#include "volleyball_controller/stewart_ik.hpp"
#include "volleyball_controller/msg/stewart_control.hpp"
#include "fmt/core.h"
#include "fmt/format.h"

#define DEG2RAD(x) ((x) * (M_PI / 180.0))
#define RAD2DEG(x) ((x) * (180.0 / M_PI))

using namespace std::chrono_literals;

class LkStewartController : public rclcpp::Node
{
public:
    explicit LkStewartController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("lk_stewart_gimbal_controller", options)
    {
        this->declare_parameter<double>("scale_eff", 1.0);
        this->declare_parameter<double>("rb", 0.214);
        this->declare_parameter<double>("l1", 0.12);
        this->declare_parameter<double>("l2", 0.3075);
        this->declare_parameter<double>("rp", 0.134);
        this->declare_parameter<double>("zrp", 0.0);
        this->declare_parameter<double>("ei", 0.1);
        this->declare_parameter<double>("bi", 0.03);
        this->declare_parameter<double>("default_height", 0.22816);
        this->declare_parameter<double>("zero_offset", DEG2RAD(18));

        //this->declare_parameter<double>("mit_kp", 25.0);
        //this->declare_parameter<double>("mit_kd", 1.0);
        this->declare_parameter<double>("motor_min_angle", -15.0 * M_PI / 180.0);
        this->declare_parameter<double>("motor_max_angle", 80.0 * M_PI / 180.0);

        this->declare_parameter<std::string>("gimbal_cmd_read_topic", "stewart_command");
        this->declare_parameter<std::string>("gimbal_motor_read_topic_fmt", "lk_motor_{}_read");
        this->declare_parameter<std::string>("gimbal_motor_write_topic_fmt", "lk_motor_{}_write");

        zero_offset_ = this->get_parameter("zero_offset").as_double();

        scale_eff_ = this->get_parameter("scale_eff").as_double();
        rb_ = this->get_parameter("rb").as_double();
        l1_ = this->get_parameter("l1").as_double();
        l2_ = this->get_parameter("l2").as_double();
        rp_ = this->get_parameter("rp").as_double();
        zrp_ = this->get_parameter("zrp").as_double();
        ei_ = this->get_parameter("ei").as_double();
        bi_ = this->get_parameter("bi").as_double();
        stewart_ik_.set_platform_params(
            rb_ * scale_eff_,
            l1_ * scale_eff_,
            l2_ * scale_eff_,
            rp_ * scale_eff_,
            zrp_ * scale_eff_,
            ei_ * scale_eff_,
            bi_ * scale_eff_
        );

      //  mit_kp_ = this->get_parameter("mit_kp").as_double();
      //  mit_kd_ = this->get_parameter("mit_kd").as_double();
        motor_min_angle_ = this->get_parameter("motor_min_angle").as_double();
        motor_max_angle_ = this->get_parameter("motor_max_angle").as_double();

        control_command_.z = this->get_parameter("default_height").as_double();

        gimbal_command_sub_ =
            this->create_subscription<
                volleyball_controller::msg::StewartControl>(
            this->get_parameter("gimbal_cmd_read_topic").as_string(),
            rclcpp::SensorDataQoS(),
            [this](volleyball_controller::msg::StewartControl::SharedPtr msg)
            {
                this->control_command_ = *msg;
            }
            );

        for (int i = 0; i < 6; ++i)
        {
            std::string read_topic = fmt::format(this->get_parameter("gimbal_motor_read_topic_fmt").as_string(), i);
            std::string write_topic = fmt::format(this->get_parameter("gimbal_motor_write_topic_fmt").as_string(),
                                                  i);

            motor_subs_[i] = this->create_subscription<custom_msgs::msg::ReadLkMotor>(
                read_topic, rclcpp::SensorDataQoS(),
                [this, i](custom_msgs::msg::ReadLkMotor::SharedPtr msg)
                {
                    double actual_pos = msg->encoder;
                    if (i == 0 || i == 2 || i == 4)
                    {
                        actual_pos = -actual_pos;
                    }
                    this->actual_motor_angles_[i] = RAD2DEG(actual_pos);
                });

            motor_pubs_[i] = this->create_publisher<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>(
                write_topic, rclcpp::SensorDataQoS());
        }

        control_timer_ =
            this->create_wall_timer(
                2ms,
                [this] { control_loop(); });

        // verify_timer_ =
        //     this->create_wall_timer(
        //         1ms,
        //         [this] { verify_loop(); });

        RCLCPP_INFO(this->get_logger(), "Lk stewart controller started");
    }

private:
    rclcpp::Subscription<volleyball_controller::msg::StewartControl>::SharedPtr gimbal_command_sub_;

    std::array<rclcpp::Subscription<custom_msgs::msg::ReadLkMotor>::SharedPtr, 6> motor_subs_;
    std::array<rclcpp::Publisher<custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit>::SharedPtr, 6> motor_pubs_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr verify_timer_;

    volleyball_controller::msg::StewartControl control_command_;
    std::array<double, 6> actual_motor_angles_{0, 0, 0, 0, 0, 0};

    aim::algorithm::ik::stewart::StewartIk stewart_ik_{};
    std::array<double, 6> target_angles_{};
    std::array<double, 6> fk_pose_{};

    double zero_offset_;

    double scale_eff_;
    double rb_;
    double l1_;
    double l2_;
    double rp_;
    double zrp_;
    double ei_;
    double bi_;

   // double mit_kp_;
   // double mit_kd_;

    // motor angle limitation
    double motor_min_angle_;
    double motor_max_angle_;

    void control_loop();
    void verify_loop();
};

#endif //LK_STEWART_CONTROLLER_HPP
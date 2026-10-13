//
// Created by txy on 7/8/26.
//
#include "rclcpp/rclcpp.hpp"
#include "volleyball_controller/volleyball_hub_controller.hpp"

#include "universal_utils/unit_utils.hpp"

custom_msgs::msg::ReadSBUSRC VolleyballHub::rc_{};
custom_msgs::msg::ReadDmMotor VolleyballHub::pitch_motor_status_{};
custom_msgs::msg::ReadLkMotor VolleyballHub::strike_motor_status_{};
custom_msgs::msg::WriteLkMotorTorqueControl VolleyballHub::strike_motor_command_{};
custom_msgs::msg::WriteDmMotorPositionControlWithSpeedLimit VolleyballHub::pitch_motor_command_{};
custom_msgs::msg::WriteOnBoardPWM VolleyballHub::pwm_command_{};
chassis_controllers::msg::ChassisControl VolleyballHub::chassis_command_{};
volleyball_controller::msg::StewartControl VolleyballHub::gimbal_command_{};


void VolleyballHub::control_loop()
{
    if (!rc_received_
        || !rc_.online
        || rc_.frame_lost
        || rc_.fail_safe
        || rc_.channels[static_cast<size_t>(RCConstants::RCChannel::SWITCH_ESTOP_BTN)] > RCConstants::MID_VALUE)
    {
        emergency_stop();
        return;
    }

    // TODO MODE CHANGE
    control_mode_ = ControlMode::MANUAL;

    switch (control_mode_)
    {
    case ControlMode::MANUAL:
        {
            // chassis
            if (rc_.channels[static_cast<size_t>(RCConstants::RCChannel::BACK_R)] < 1000)
            {
                chassis_command_.spin_speed = -(rc_.channels[static_cast<size_t>(RCConstants::RCChannel::LEFT_X)] -
                    RCConstants::MID_VALUE) / RCConstants::SCALE * MAX_CHASSIS_SPIN_SPEED;
            }
            else
            {
                chassis_command_.spin_speed = 0;
            }
            chassis_command_.x_speed = (rc_.channels[static_cast<size_t>(RCConstants::RCChannel::RIGHT_Y)] -
                RCConstants::MID_VALUE) / RCConstants::SCALE * MAX_CHASSIS_SPEED;
            chassis_command_.y_speed = -(rc_.channels[static_cast<size_t>(RCConstants::RCChannel::RIGHT_X)] -
                RCConstants::MID_VALUE) / RCConstants::SCALE * MAX_CHASSIS_SPEED;

            chassis_command_.emergency_stop = 0;
            chassis_command_.power_limit = 0;

            // pitch
            // float pitch_delta = (rc_.channels[static_cast<size_t>(RCConstants::RCChannel::LEFT_Y)] -
            //     RCConstants::MID_VALUE) / RCConstants::SCALE * 0.01;
            //
            // if (pitch_motor_command_.position + pitch_delta > PITCH_MAX)
            // {
            //     pitch_motor_command_.position = PITCH_MAX;
            // }
            // else if (pitch_motor_command_.position + pitch_delta < PITCH_MIN)
            // {
            //     pitch_motor_command_.position = PITCH_MIN;
            // }
            // else
            // {
            //     pitch_motor_command_.position += pitch_delta;
            // }
            // pitch_motor_command_.position = PITCH_MIN;

            // stewart
            gimbal_command_.x = 0.0;
            gimbal_command_.y = 0.0;
            gimbal_command_.yaw = 0.0;
            if (rc_.channels[static_cast<size_t>(RCConstants::RCChannel::BACK_R)] > 1000
                && strike_state_ == StrikeState::DONE)
            {
                gimbal_command_.pitch = -(rc_.channels[static_cast<size_t>(RCConstants::RCChannel::LEFT_Y)] -
                    RCConstants::MID_VALUE) / RCConstants::SCALE * pitch_range;
                gimbal_command_.roll = -(rc_.channels[static_cast<size_t>(RCConstants::RCChannel::LEFT_X)] -
                    RCConstants::MID_VALUE) / RCConstants::SCALE * roll_range;
            }
            else
            {
                gimbal_command_.pitch = 0.0;
                gimbal_command_.roll = 0.0;
            }

            break;
        }
    }

    // === pwm ===
    // if (rc_.channels[10] > 1000)
    // {
    //     pwm_command_.channel1 = pwm_up;
    // }
    // else
    // {
    //     pwm_command_.channel1 = pwm_down;
    // }
    // === pwm ===

    // === strike ===
    // manual trigger
    if (rc_.channels[4] > 1000)
    {
        if (!strike_flag_)
        {
            if (strike_state_ == StrikeState::WAITING)
            {
                strike_state_ = StrikeState::PUSHING;
                strike_flag_ = true;
            }
        }
    }
    else
    {
        if (strike_state_ == StrikeState::DONE)
        {
            strike_state_ = StrikeState::WAITING;
        }
        strike_flag_ = false;
    }

    switch (strike_state_)
    {
    case StrikeState::WAITING:
        {
            // 拍
            // only allow reverse direction
            calculate_strike_motor_command(ready_position);
            pwm_command_.channel1 = pwm_up;
            // pitch
            pitch_motor_command_.position = PITCH_MAX;
            push_base_z_ = z_min_push;
            break;
        }
    case StrikeState::PUSHING:
        {
            if (strike_push_flag_
                && push_state_ == PushState::RESETTING)
            {
                strike_push_flag_ = false;
                strike_state_ = StrikeState::LAYING_DOWN;
                break;
            }
            if (!strike_push_flag_)
            {
                push_state_ = PushState::GOING_UP;
                push_last_action_time_ = now();
                strike_push_flag_ = true;
            }

            calculate_strike_motor_command(ready_position);
            pwm_command_.channel1 = pwm_up;
            // pitch
            pitch_motor_command_.position = PITCH_MAX;
            push_base_z_ = z_min_push;
            break;
        }
    case StrikeState::LAYING_DOWN:
        {
            if (pitch_motor_status_.position <= -0.5)
            {
                strike_state_ = StrikeState::GOING_DOWN;
                strike_ready_time_ = now();
            }
            calculate_strike_motor_command(ready_position);
            pwm_command_.channel1 = pwm_down;
            // pitch
            pitch_motor_command_.position = PITCH_MIN;
            push_base_z_ = z_min;
            break;
        }
    case StrikeState::GOING_DOWN:
        {
            if ((now() - strike_ready_time_).seconds() >= strike_delay_time)
            {
                // pass the lowest point
                if (strike_motor_status_.encoder > 50000)
                {
                    strike_state_ = StrikeState::GOING_UP;
                }
                strike_motor_command_.torque = -2000;
                pwm_command_.channel1 = pwm_down;
                // pitch
                pitch_motor_command_.position = PITCH_MIN;
                push_base_z_ = z_min;
            }
            else
            {
                calculate_strike_motor_command(ready_position);
                pwm_command_.channel1 = pwm_down;
                // pitch
                pitch_motor_command_.position = PITCH_MIN;
                push_base_z_ = z_min;
            }
            break;
        }
    case StrikeState::GOING_UP:
        {
            // striking done
            if (strike_motor_status_.encoder < 55555)
            {
                strike_state_ = StrikeState::DONE;
            }
            else
            {
                strike_motor_command_.torque = -2000;
            }
            pwm_command_.channel1 = pwm_down;
            // pitch
            pitch_motor_command_.position = PITCH_MIN;
            push_base_z_ = z_min;

            break;
        }
    case StrikeState::DONE:
        {
            // 拍
            // only allow reverse direction
            calculate_strike_motor_command(ready_position);
            pwm_command_.channel1 = pwm_down;
            // pitch
            pitch_motor_command_.position = PITCH_MIN;
            push_base_z_ = z_min;
            break;
        }
    default:
        {
        }
    }
    strike_motor_command_.enable = 1;
    strike_motor_pub_->publish(strike_motor_command_);
    // === strike ===

    // === push ===
    if (rc_.channels[6] > 1000)
    {
        if (!push_flag_)
        {
            if (push_state_ == PushState::WAITING
                && strike_state_ == StrikeState::DONE)
            {
                push_state_ = PushState::GOING_UP;
                push_last_action_time_ = now();
                push_flag_ = true;
            }
        }
    }
    else
    {
        push_flag_ = false;
    }

    switch (push_state_)
    {
    case PushState::WAITING:
        {
            gimbal_command_.z = z_min;
            break;
        }
    case PushState::GOING_UP:
        {
            if ((now() - push_last_action_time_).seconds() > push_going_up_timeout)
            {
                push_state_ = PushState::RESETTING;
                push_last_action_time_ = now();
                break;
            }
            gimbal_command_.z = z_max;
            break;
        }
    case PushState::RESETTING:
        {
            if ((now() - push_last_action_time_).seconds() > push_going_down_timeout)
            {
                push_state_ = PushState::WAITING;
                push_last_action_time_ = now();
            }
            gimbal_command_.z = z_min;
            break;
        }
    }
    // === push ===
    if (push_state_ == PushState::GOING_UP || rc_.channels[8] > 1000)
    {
        chassis_command_.x_speed = 0;
        chassis_command_.y_speed = 0;
        chassis_command_.spin_speed = 0;
        chassis_command_.extra_info[0] = 1;
    }
    else
    {
        chassis_command_.extra_info[0] = 0;
    }
    chassis_command_pub_->publish(chassis_command_);

    pitch_motor_command_.speed = 25;
    pitch_motor_command_.enable = 1;
    pitch_motor_pub_->publish(pitch_motor_command_);

    gimbal_command_.emergency_stop = 0;
    gimbal_command_pub_->publish(gimbal_command_);

    pwm_pub_->publish(pwm_command_);
}

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node =
        std::make_shared<VolleyballHub>();
    rclcpp::on_shutdown([&]
    {
        node->emergency_stop();
        RCLCPP_INFO(node->get_logger(), "Shutting down safely...");
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

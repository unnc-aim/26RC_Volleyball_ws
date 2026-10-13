//
// Created by happydog on 6/3/26.
//
#include "volleyball_controller/dm_stewart_controller.hpp"
#include <algorithm>
#include <cmath>


void LkStewartController::control_loop()
{
    target_angles_ = stewart_ik_.calculate(
        control_command_.x, control_command_.y, control_command_.z,
        control_command_.roll, control_command_.pitch, control_command_.yaw
    );
    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "ik angles = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //     target_angles_[0], target_angles_[1], target_angles_[2],
    //     target_angles_[3], target_angles_[4], target_angles_[5]);

    bool unsafe = false;
    for (const double val : target_angles_)
    {
        if (std::isnan(val))
        {
            unsafe = true;
        }
    }

    if (control_command_.emergency_stop || unsafe)
    {
        for (int i = 0; i < 6; ++i)
        {
            custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit msg;
            msg.enable = 0;
            msg.speed_limit = 0;
            msg.angle = 0;
            motor_pubs_[i]->publish(msg);
        }
        return;
    }

    for (int i = 0; i < 6; ++i)
    {
        custom_msgs::msg::WriteLkMotorMultiRoundPositionControlWithSpeedLimit msg;
        msg.enable = 1;
        const double logical_angle = std::clamp(target_angles_[i] + zero_offset_, motor_min_angle_, motor_max_angle_);

        double final_p_des = logical_angle;
        if (i == 0 || i == 2 || i == 4)
        {
            final_p_des = -final_p_des;
        }

        msg.speed_limit = 200;
        msg.angle = static_cast<int32_t>(RAD2DEG(final_p_des));

        motor_pubs_[i]->publish(msg);
    }
}

void LkStewartController::verify_loop()
{
    fk_pose_ = stewart_ik_.calculate(actual_motor_angles_);

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "fk actual_motor_angles_ = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //     actual_motor_angles_[0], actual_motor_angles_[1], actual_motor_angles_[2],
    //     actual_motor_angles_[3], actual_motor_angles_[4], actual_motor_angles_[5]);
    //
    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "fk pose = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    //     fk_pose_[0], fk_pose_[1], fk_pose_[2],
    //     fk_pose_[3], fk_pose_[4], fk_pose_[5]);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node =
        std::make_shared<LkStewartController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

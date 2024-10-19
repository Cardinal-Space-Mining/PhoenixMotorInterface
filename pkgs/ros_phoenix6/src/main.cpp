#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// CTRE and custom types
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

using TalonFX = ctre::phoenix6::hardware::TalonFX;

namespace constants {
    static const std::string INTERFACE = "can0";
    static constexpr double kP = 0.11;
    static constexpr double kI = 0.5;
    static constexpr double kD = 0.0001;
    static constexpr double kV = 0.12;
} // namespace constants

class Robot : public rclcpp::Node {
public:
    Robot() : Node("robot") {
        setup_subscriptions();
        setup_motors();
        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    // Function to setup subscriptions for the robot
    void setup_subscriptions() {
        heartbeat_sub = this->create_subscription<std_msgs::msg::Int32>(
            "heartbeat", 10, [](const std_msgs::msg::Int32 &msg) {
                ctre::phoenix::unmanaged::FeedEnable(msg.data);
            });

        track_right_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_right_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->track_right, msg);
            });

        track_left_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_left_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->track_left, msg);
            });

        info_timer = this->create_wall_timer(100ms, [this]() { this->info_periodic(); });
    }

    // Function to setup motors for the robot
    void setup_motors() {
        std::array<std::reference_wrapper<TalonFX>, 2> motors = {
            {track_right, track_left}
        };

        for (auto &motor : motors) {
            config_talonfx(motor, constants::kP, constants::kI, constants::kD, constants::kV);
        }
    }

    // Function to configure a TalonFX motor with PID and other settings
    void config_talonfx(TalonFX &motor, double kP, double kI, double kD, double kV) {
        configs::TalonFXConfiguration config;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.GetConfigurator().Apply(config);
    }

    // Function to execute control commands on a motor
    void execute_ctrl(TalonFX &motor, const custom_types::msg::TalonCtrl &msg) {
        motor.SetControl(controls::DutyCycleOut(msg.value));
    }

    // Periodic function for motor information updates
    void info_periodic() {
        // Retrieve the velocity values directly using GetValue()
        auto right_velocity_status = track_right.GetVelocity().GetValue();
        auto left_velocity_status = track_left.GetVelocity().GetValue();

        // Check if the status signal is valid before using it
        if (right_velocity_status) {
            double right_velocity = right_velocity_status.value();
            RCLCPP_INFO(this->get_logger(), "Right motor velocity: %f", right_velocity);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to retrieve right motor velocity.");
        }

        if (left_velocity_status) {
            double left_velocity = left_velocity_status.value();
            RCLCPP_INFO(this->get_logger(), "Left motor velocity: %f", left_velocity);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to retrieve left motor velocity.");
        }
    }
    
private:
    TalonFX track_right{0, constants::INTERFACE};
    TalonFX track_left{1, constants::INTERFACE};  

    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_right_ctrl;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_left_ctrl;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::TimerBase::SharedPtr info_timer;
};

int main(int argc, char **argv) {
    // Initialize ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    // Load the Phoenix library for motor control functions
    ctre::phoenix::unmanaged::LoadPhoenix();
    std::cout << "Loaded Phoenix" << std::endl;

    auto node = std::make_shared<Robot>();
    RCLCPP_INFO(node->get_logger(), "Robot node has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Robot node has shut down");
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

// CTRE and custom types
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"
#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

using TalonInfo = custom_types::msg::TalonInfo;
using TalonFX = ctre::phoenix6::hardware::TalonFX;

enum class RobotStatus : int8_t {
    TELEOP   = 0,
    DISABLED = 1,
    AUTONOMY = 2
};

namespace constants {
    static const std::string INTERFACE = "can0";
    static constexpr double kP = 0.11;
    static constexpr double kI = 0.5;
    static constexpr double kD = 0.0001;
    static constexpr double kV = 0.12;
} // namespace constants

class Robot : public rclcpp::Node {
public:
    Robot() : Node("robot")
    // motor info publishers
    ,
    track_right_info(this->create_publisher<TalonInfo>(
        "track_right_info", 10))
    ,
    track_left_info(this->create_publisher<TalonInfo>(
        "track_left_info", 10))
    ,
    trencher_info(this->create_publisher<TalonInfo>(
        "trencher_info", 10))
    ,
    hopper_info(this->create_publisher<TalonInfo>(
        "hopper_info", 10))
    ,
    info_timer(this->create_wall_timer(100ms, [this]()
        { this->info_periodic(); }))
    {
        setup();
        setup_motors();
        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    // Function to setup subscriptions for the robot
    void setup() {
        track_right_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_right_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->track_right, msg);
            });

        track_left_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "track_left_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->track_left, msg);
            });

        trencher_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "trencher_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->trencher, msg);
            });

        hopper_ctrl = this->create_subscription<custom_types::msg::TalonCtrl>(
            "hopper_belt_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg) {
                execute_ctrl(this->hopper, msg);
            });

        heartbeat_sub = this->create_subscription<std_msgs::msg::Int32>(
            "heartbeat", 10, [](const std_msgs::msg::Int32 &msg) {
                ctre::phoenix::unmanaged::FeedEnable(msg.data);
            });

        robot_status_sub = this->create_subscription<std_msgs::msg::Int8>(
            "robot_status", 10, [this](const std_msgs::msg::Int8 &msg) {
                update_status(msg);
            });

        config_motors_sub = this->create_subscription<std_msgs::msg::Int8>(
            "config_motors", 10, [this](const std_msgs::msg::Int8 &msg) {
                RCLCPP_INFO(this->get_logger(), "%d. Applying motor configs", msg.data);
                setup_motors();
            });
    }

    // Function to setup motors for the robot
    void setup_motors() {

        configs::TalonFXConfiguration config{};
        config.Slot0.kP = constants::kP;
        config.Slot0.kI = constants::kI;
        config.Slot0.kD = constants::kD;
        config.Slot0.kV = constants::kV;
        config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        trencher.GetConfigurator().Apply(config);

        config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        hopper.GetConfigurator().Apply(config);

        config.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        track_right.GetConfigurator().Apply(config);

        config.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
        track_left.GetConfigurator().Apply(config);

        track_right.SetControl(controls::NeutralOut());
        track_left.SetControl(controls::NeutralOut());
        trencher.SetControl(controls::NeutralOut());
        hopper.SetControl(controls::NeutralOut());

        RCLCPP_INFO(this->get_logger(), "\nConfiguring Motors\n");
    }

    // Function to execute control commands on a motor
    void execute_ctrl(TalonFX &motor, const custom_types::msg::TalonCtrl &msg) {
        if (robot_status == RobotStatus::DISABLED) {
            motor.SetControl(controls::NeutralOut());
            return;
        }

        if (abs(msg.value) < 0.1 ) {
            motor.SetControl(controls::NeutralOut());
        } else {
            motor.SetControl(controls::DutyCycleOut(msg.value));
        }
    }

    // Periodic function for motor information updates
    void info_periodic() {
        auto msg = TalonInfo();

        msg.header.stamp = this->get_clock()->now();

        // Track Right motor
        msg.temperature     = track_right.GetDeviceTemp().GetValueAsDouble();
        msg.bus_voltage     = track_right.GetSupplyVoltage().GetValueAsDouble();
        msg.output_percent  = track_right.GetMotorOutputStatus().GetValueAsDouble();
        msg.output_voltage  = track_right.GetMotorVoltage().GetValueAsDouble();
        msg.output_current  = track_right.GetSupplyCurrent().GetValueAsDouble();
        msg.velocity        = track_right.GetVelocity().GetValueAsDouble();
        msg.position        = track_right.GetPosition().GetValueAsDouble();
        track_right_info->publish(msg);

        // Track Left motor
        msg.temperature     = track_left.GetDeviceTemp().GetValueAsDouble();
        msg.bus_voltage     = track_left.GetSupplyVoltage().GetValueAsDouble();
        msg.output_percent  = track_left.GetMotorOutputStatus().GetValueAsDouble();
        msg.output_voltage  = track_left.GetMotorVoltage().GetValueAsDouble();
        msg.output_current  = track_left.GetSupplyCurrent().GetValueAsDouble();
        msg.velocity        = track_left.GetVelocity().GetValueAsDouble();
        msg.position        = track_left.GetPosition().GetValueAsDouble();
        track_left_info->publish(msg);

        // Trencher
        msg.temperature     = trencher.GetDeviceTemp().GetValueAsDouble();
        msg.bus_voltage     = trencher.GetSupplyVoltage().GetValueAsDouble();
        msg.output_percent  = trencher.GetMotorOutputStatus().GetValueAsDouble();
        msg.output_voltage  = trencher.GetMotorVoltage().GetValueAsDouble();
        msg.output_current  = trencher.GetSupplyCurrent().GetValueAsDouble();
        msg.velocity        = trencher.GetVelocity().GetValueAsDouble();
        msg.position        = trencher.GetPosition().GetValueAsDouble();
        trencher_info->publish(msg);

        // hopper
        msg.temperature     = hopper.GetDeviceTemp().GetValueAsDouble();
        msg.bus_voltage     = hopper.GetSupplyVoltage().GetValueAsDouble();
        msg.output_percent  = hopper.GetMotorOutputStatus().GetValueAsDouble();
        msg.output_voltage  = hopper.GetMotorVoltage().GetValueAsDouble();
        msg.output_current  = hopper.GetSupplyCurrent().GetValueAsDouble();
        msg.velocity        = hopper.GetVelocity().GetValueAsDouble();
        msg.position        = hopper.GetPosition().GetValueAsDouble();
        hopper_info->publish(msg);
    }

    void update_status(const std_msgs::msg::Int8 &msg) {
        switch (msg.data) {
        case 0:
            robot_status = RobotStatus::TELEOP;
            break;
        case 1:
            robot_status = RobotStatus::DISABLED;
            break;
        case 2:
            robot_status = RobotStatus::AUTONOMY;
            break;
        default:
            robot_status = RobotStatus::DISABLED;
            break;
        }
    }

private:
    TalonFX track_right{0, constants::INTERFACE};
    TalonFX track_left{1, constants::INTERFACE};
    TalonFX trencher{2, constants::INTERFACE};
    TalonFX hopper{3, constants::INTERFACE};

    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr track_right_info;
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr track_left_info;
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr trencher_info;
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr hopper_info;

    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_right_ctrl;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr track_left_ctrl;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr trencher_ctrl;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr hopper_ctrl;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_status_sub;
    rclcpp::TimerBase::SharedPtr info_timer;

    std::array<std::reference_wrapper<TalonFX>, 4> motors = {
        {track_right, track_left, trencher, hopper}
    };

    RobotStatus robot_status;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr config_motors_sub;
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

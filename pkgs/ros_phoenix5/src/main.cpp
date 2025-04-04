
#include <chrono>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace std::chrono_literals;

using TalonSRX = ctre::phoenix::motorcontrol::can::TalonSRX;
using TalonCtrlSub = rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr;
using TalonInfoPub = rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr;

struct Gains
{
    double P, I, D, F;
};

enum class BrakeMode : uint8_t {
    COAST = 0,
    BRAKE = 1
};

enum class RobotStatus : int8_t {
    TELEOP   = 0,
    DISABLED = 1,
    AUTONOMY = 2
};

namespace constants {
static const std::string INTERFACE = "can0";
static constexpr Gains DEFAULT_GAINS{0.0, 0.0, 0.0, 0.2};
static constexpr Gains DefaultRobotGains{
    0.11,
    0.5,
    0.0001,
    0.12,
};
} // namespace constants

class Robot : public rclcpp::Node {
public:
    Robot()
    : rclcpp::Node("robot_phoenix5")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , hopper_ctrl_teleop(this->create_subscription<custom_types::msg::TalonCtrl>(
        "hopper_ctrl_teleop", 10, [this](const custom_types::msg::TalonCtrl &msg)
        { execute_ctrl(this->hopper_actuator, msg); }))
    , hopper_ctrl_auto(this->create_subscription<custom_types::msg::TalonCtrl>(
        "hopper_ctrl_auto", 10, [this](const custom_types::msg::TalonCtrl &msg)
        { execute_ctrl(this->hopper_actuator, msg); }))
    , hopper_info(this->create_publisher<custom_types::msg::TalonInfo>(
        "hopper_info", 10))
    // robot status (teleop, disabled, autonomy)
    , robot_status(RobotStatus::DISABLED)
    , robot_status_sub(this->create_subscription<std_msgs::msg::Int8>(
        "robot_status", 10, [this](const std_msgs::msg::Int8 &msg)
        { update_status(msg); }))
    , info_timer(
        this->create_wall_timer(100ms, [this]() { this->info_periodic(); }))
    {

        // config(hopper_actuator, constants::DEFAULT_GAINS, BrakeMode::BRAKE);

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:

    void config(TalonSRX &motor, const Gains &gains, BrakeMode brake_mode) {
        // config motor settings here
        motor.Config_kD(0, gains.P);
        motor.Config_kP(0, gains.I);
        motor.Config_kI(0, gains.D);
        motor.Config_kF(0, gains.F);

        switch (brake_mode) {
            case BrakeMode::BRAKE:
                motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
                break;
            case BrakeMode::COAST:
                motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
                break;
        }
    }

    void execute_ctrl(TalonSRX &motor, const custom_types::msg::TalonCtrl &msg) {
        RCLCPP_INFO(this->get_logger(), "Motor output: %f", msg.value);
        // RCLCPP_DEBUG(this->get_logger(), "msg: %s", msg);
        if (robot_status == RobotStatus::TELEOP || robot_status == RobotStatus::AUTONOMY) {
            motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, msg.value);
        } else {
            motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        }
    }

    custom_types::msg::TalonInfo get_info(TalonSRX &motor) {
        custom_types::msg::TalonInfo info;
        // info.temperature    = motor.GetTemperature();
        // info.bus_voltage    = motor.GetBusVoltage();
        // info.output_percent = motor.GetMotorOutputPercent();
        // info.output_voltage = motor.GetMotorOutputVoltage();
        // info.output_current = motor.GetOutputCurrent();
        // info.position       = motor.GetSelectedSensorPosition();
        // info.velocity       = motor.GetSelectedSensorVelocity();
        info.temperature = 0;
        info.bus_voltage = 0;
        info.output_percent = 0;
        info.output_voltage = 0;
        info.output_current = 0;
        info.position = 0;
        info.velocity = 0;

        return info;
    }

    void info_periodic() {
        hopper_info->publish(get_info(hopper_actuator));
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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_sub;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr hopper_ctrl_teleop;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr hopper_ctrl_auto;

private:
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr hopper_info;

private:
    TalonSRX hopper_actuator{4, constants::INTERFACE};

private:
    RobotStatus robot_status;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_status_sub;

private:
    rclcpp::TimerBase::SharedPtr info_timer;

};

int main(int argc, char ** argv)
{
    ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();

    std::cout << "Loaded Pheonix" << std::endl;

    // Init ROS2 for logging capabilities
    rclcpp::init(argc, argv);

    std::cout << "Loaded rclcpp" << std::endl;

    auto node = std::make_shared<Robot>();

    std::cout << "Loaded Bot Node" << std::endl;

    rclcpp::spin(node);

    std::cout << "Spun Node" << std::endl;
    rclcpp::shutdown();

    std::cout << "Loaded Done" << std::endl;
    return EXIT_SUCCESS;
}
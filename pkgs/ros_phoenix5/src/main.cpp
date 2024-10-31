
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

namespace constants
{
static const std::string INTERFACE = "can0";
static constexpr Gains DEFAULT_GAINS{0.0, 0.0, 0.0, 0.2};
static constexpr Gains DefaultRobotGains{
    0.11,
    0.5,
    0.0001,
    0.12,
};
} // namespace constants

class Robot : public rclcpp::Node
{
public:
    Robot()
    : rclcpp::Node("robot_phoenix5")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , info_timer(
        this->create_wall_timer(100ms, [this]() { this->info_periodic(); }))
    , hopper_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
        "hopper_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg)
        { execute_ctrl(this->hopper_actuator, msg); }))
    , hopper_info(this->create_publisher<custom_types::msg::TalonInfo>(
        "hopper_info", 10))
    {

        config(hopper_actuator, constants::DEFAULT_GAINS, BrakeMode::BRAKE);

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
        motor.Set(static_cast<ctre::phoenix::motorcontrol::ControlMode>(msg.mode), msg.value);
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

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr hopper_ctrl;

private:
    rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr hopper_info;

private:
    TalonSRX hopper_actuator{4, constants::INTERFACE};

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
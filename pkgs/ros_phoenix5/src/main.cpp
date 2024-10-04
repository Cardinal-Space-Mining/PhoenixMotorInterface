
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
using TalonCtrlSub =
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr;
using TalonInfoPub = rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr;

struct Gains
{
    double P, I, D, F;
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
    : rclcpp::Node("robot")
    , heartbeat_sub(this->create_subscription<std_msgs::msg::Int32>(
          "heartbeat", 10, [](const std_msgs::msg::Int32 & msg)
          { ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); }))
    , hopper_ctrl(this->create_subscription<custom_types::msg::TalonCtrl>(
        "hopper_ctrl", 10, [this](const custom_types::msg::TalonCtrl &msg)
        { execute_ctrl(this->hopper_actuator, msg); }))
    {

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    void execute_ctrl(TalonSRX &motor, const custom_types::msg::TalonCtrl &msg) {

    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr hopper_ctrl;

private:
    TalonSRX hopper_actuator{5, constants::INTERFACE};
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

#include <chrono>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "TalonWrapper.hpp"

using namespace std::chrono_literals;

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
    , track_right(*this, "track_right", 0)
    , track_left(*this, "track_left", 1)
    , trencher(*this, "trencher", 2)
    , hopper_belt(*this, "hopper_belt", 3)
    , hopper_actuator(*this, "hopper_actuator", 4)
    {

        RCLCPP_DEBUG(this->get_logger(), "Initialized Node");
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> heartbeat_sub;

private:
    TalonWrapper<TalonFX6> track_right, track_left, trencher, hopper_belt;

    TalonWrapper<TalonSRX5> hopper_actuator;
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
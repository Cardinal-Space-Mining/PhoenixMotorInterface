#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop_control/logitech_map.hpp"
#include "teleop_control/states.hpp"

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"

#ifndef ENABLE_HEARTBEAT_PUB
#define ENABLE_HEARTBEAT_PUB 0
#endif


using namespace std::chrono_literals;

using talon_msgs::msg::TalonCtrl;
using talon_msgs::msg::TalonInfo;


enum class RobotControlMode : int8_t    // TODO: disabled should be 0!
{
    TELEOPERATED = 0,
    DISABLED = 1,
    AUTONOMOUS = 2
};

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_PUB_QOS 10


class RobotTeleopInterface
{
private:
    inline void update_motors()
    {
        MotorSettings motor_settings = this->teleop_state.update(this->robot_state, this->joy);

        track_right_ctrl->publish(motor_settings.track_right);
        track_left_ctrl->publish(motor_settings.track_left);
        trencher_ctrl->publish(motor_settings.trencher);
        hopper_ctrl->publish(motor_settings.hopper_belt);
        hopper_act_ctrl->publish(motor_settings.hopper_actuator);
    }

    inline rclcpp::Publisher<TalonCtrl>::SharedPtr talon_ctrl_pub(rclcpp::Node &parent, const char* name)
    {
        return parent.create_publisher<TalonCtrl>(name, TALON_CTRL_PUB_QOS);
    }

public:
    RobotTeleopInterface(rclcpp::Node &parent) :
        joy_sub
        {
            parent.create_subscription<sensor_msgs::msg::Joy>(
                "/joy",
                rclcpp::SensorDataQoS{},
                [this](const sensor_msgs::msg::Joy &joy){ this->joy = joy; })
        },
        track_right_ctrl{ talon_ctrl_pub(parent, ROBOT_TOPIC("track_right/ctrl")) },
        track_left_ctrl{  talon_ctrl_pub(parent, ROBOT_TOPIC("track_left/ctrl")) },
        trencher_ctrl{    talon_ctrl_pub(parent, ROBOT_TOPIC("trencher/ctrl")) },
        hopper_ctrl{      talon_ctrl_pub(parent, ROBOT_TOPIC("hopper_belt/ctrl")) },
        hopper_act_ctrl{  talon_ctrl_pub(parent, ROBOT_TOPIC("hopper_act/ctrl")) },
        teleop_update_timer{ parent.create_wall_timer(100ms, [this](){ this->update_motors(); }) }
    {}

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    rclcpp::Publisher<TalonCtrl>::SharedPtr
        track_right_ctrl,
        track_left_ctrl,
        trencher_ctrl,
        hopper_ctrl,
        hopper_act_ctrl;

private:
    sensor_msgs::msg::Joy joy;
    RobotState robot_state;

private:
    rclcpp::TimerBase::SharedPtr teleop_update_timer;
    TeleopStateMachine teleop_state;

};

class Controller : public rclcpp::Node
{
protected:
    static constexpr auto ENABLE_TIME = 250ms;

public:
    Controller() :
        Node("controller_node"),
        teleop_interface(*this)
    #if ENABLE_HEARTBEAT_PUB
        ,
        heartbeat_pub{ create_publisher<std_msgs::msg::Int32>(ROBOT_TOPIC("watchdog_status"), rclcpp::SensorDataQoS{}) },
        heartbeat_timer
        {
            this->create_wall_timer(
                100ms,
                [this](){ this->heartbeat_pub->publish(std_msgs::msg::Int32{}.set__data(ENABLE_TIME.count())); } )
        }
    #endif
    {
        RCLCPP_INFO(this->get_logger(), "Control node has initialized.");
    }

private:
    RobotTeleopInterface teleop_interface;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    #if ENABLE_HEARTBEAT_PUB
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
        heartbeat_pub;
    rclcpp::TimerBase::SharedPtr
        heartbeat_timer;
    #endif

};


int main(int argc, char * argv[])
{
    // Load ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

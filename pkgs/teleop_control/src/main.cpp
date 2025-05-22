#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "motor_interface.hpp"
#include "controller.hpp"


using namespace std::chrono_literals;

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_PUB_QOS 10
#define MOTOR_UPDATE_DT 50ms


class RobotControlNode :
    public rclcpp::Node
{
protected:
    struct TalonPubSub
    {
        rclcpp::Publisher<TalonCtrl>::SharedPtr ctrl_pub;
        rclcpp::Subscription<TalonInfo>::SharedPtr info_sub;
    };

public:
    #define INIT_TALON_PUB_SUB(device) \
        device##_pub_sub \
        { \
            this->create_publisher<TalonCtrl>( \
                ROBOT_TOPIC(#device"/ctrl"), \
                TALON_CTRL_PUB_QOS ), \
            this->create_subscription<TalonInfo>( \
                ROBOT_TOPIC(#device"/info"), \
                rclcpp::SensorDataQoS{}, \
                [this](const TalonInfo& msg){ this->robot_motor_status.device = msg; } ) \
        }

    RobotControlNode() :
        Node{ "robot_control" },
        INIT_TALON_PUB_SUB(track_right),
        INIT_TALON_PUB_SUB(track_left),
        INIT_TALON_PUB_SUB(trencher),
        INIT_TALON_PUB_SUB(hopper_belt),
        INIT_TALON_PUB_SUB(hopper_actuator),
        joy_sub
        {
            this->create_subscription<JoyMsg>(
                "/joy",
                rclcpp::SensorDataQoS{},
                [this](const JoyMsg& joy){ this->joystick_values = joy; } )
        },
        watchdog_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_status"),
                rclcpp::SensorDataQoS{},
                [this](const std_msgs::msg::Int32& status){ this->watchdog_status = status.data; } )
        },
        control_iteration_timer
        {
            this->create_wall_timer(
                MOTOR_UPDATE_DT,
                [this]()
                {
                    const RobotMotorCommands& mc =
                        this->robot_controller.update(
                            this->watchdog_status,
                            this->joystick_values,
                            this->robot_motor_status );

                    this->track_right_pub_sub.ctrl_pub->publish(mc.track_right);
                    this->track_left_pub_sub.ctrl_pub->publish(mc.track_left);
                    this->trencher_pub_sub.ctrl_pub->publish(mc.trencher);
                    this->hopper_belt_pub_sub.ctrl_pub->publish(mc.hopper_belt);
                    this->hopper_actuator_pub_sub.ctrl_pub->publish(mc.hopper_actuator);
                } )
        }
    {}

private:
    TalonPubSub
        track_right_pub_sub,
        track_left_pub_sub,
        trencher_pub_sub,
        hopper_belt_pub_sub,
        hopper_actuator_pub_sub;

    rclcpp::Subscription<JoyMsg>::SharedPtr
        joy_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        watchdog_sub;
    rclcpp::TimerBase::SharedPtr
        control_iteration_timer;

    RobotControl robot_controller;

    RobotMotorStatus robot_motor_status;
    JoyMsg joystick_values;
    int32_t watchdog_status{ 0 };

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();

    return 0;
}

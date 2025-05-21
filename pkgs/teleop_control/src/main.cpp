#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "states.hpp"


#ifndef ENABLE_HEARTBEAT_PUB
#define ENABLE_HEARTBEAT_PUB 0
#endif


using namespace std::chrono_literals;

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_PUB_QOS 10
#define MOTOR_UPDATE_DT 50ms


class RobotControlNode :
    public rclcpp::Node
{
public:
    RobotControlNode() :
        Node{ "robot_control" },
        track_right_ctrl{
            this->create_publisher<TalonCtrl>(ROBOT_TOPIC("track_right/ctrl"), TALON_CTRL_PUB_QOS) },
        track_left_ctrl{
            this->create_publisher<TalonCtrl>(ROBOT_TOPIC("track_left/ctrl"), TALON_CTRL_PUB_QOS) },
        trencher_ctrl{
            this->create_publisher<TalonCtrl>(ROBOT_TOPIC("trencher/ctrl"), TALON_CTRL_PUB_QOS) },
        hopper_ctrl{
            this->create_publisher<TalonCtrl>(ROBOT_TOPIC("hopper_belt/ctrl"), TALON_CTRL_PUB_QOS) },
        hopper_act_ctrl{
            this->create_publisher<TalonCtrl>(ROBOT_TOPIC("hopper_act/ctrl"), TALON_CTRL_PUB_QOS) },
        joy_sub
        {
            this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy",
                rclcpp::SensorDataQoS{},
                [this](const sensor_msgs::msg::Joy& joy){ this->joy = joy; } )
        },
        watchdog_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_status"),
                rclcpp::SensorDataQoS{},
                [this](const std_msgs::msg::Int32& status){ this->status = status.data; } )
        },
        teleop_update_timer{
            this->create_wall_timer(MOTOR_UPDATE_DT, [this](){ this->update_motors(); }) }
    {}

private:
    inline void update_motors()
    {
        MotorCommands motor_settings;   // TODO: set disable states correctly
        this->teleop_state.update(motor_settings, this->robot_status, this->joy);

        this->track_right_ctrl->publish(motor_settings.track_right);
        this->track_left_ctrl->publish(motor_settings.track_left);
        this->trencher_ctrl->publish(motor_settings.trencher);
        this->hopper_ctrl->publish(motor_settings.hopper_belt);
        this->hopper_act_ctrl->publish(motor_settings.hopper_actuator);
    }

private:
    rclcpp::Publisher<TalonCtrl>::SharedPtr
        track_right_ctrl,
        track_left_ctrl,
        trencher_ctrl,
        hopper_ctrl,
        hopper_act_ctrl;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
        joy_sub;
    rclcpp::Subscription<sensor_msgs::msg::Int32>::SharedPtr
        watchdog_sub;
    rclcpp::TimerBase::SharedPtr
        teleop_update_timer;

    TeleopStateMachine teleop_state;
    RobotMotorInfo robot_status;
    sensor_msgs::msg::Joy joy;
    int32_t status{ 0 };

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();

    return 0;
}

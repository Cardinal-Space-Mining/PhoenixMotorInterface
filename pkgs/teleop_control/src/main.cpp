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
    void update_motors() {
        MotorSettings motor_settings =
        this->teleop_state.update(this->robot_state, this->joy);
        track_right_ctrl->publish(motor_settings.track_right);
        track_left_ctrl->publish(motor_settings.track_left);
        trencher_ctrl->publish(motor_settings.trencher);
        hopper_ctrl->publish(motor_settings.hopper_belt);
        hopper_act_ctrl->publish(motor_settings.hopper_actuator);
    }

    // void display_info_actuator(const talon_msgs::msg::TalonInfo &msg) {
    //     std::clog
    //               << "Motor Temp: \t" << msg.device_temp << "\n"
    //               << "Motor Volt: \t" << msg.bus_voltage << "\n"
    //               << "Output Percent: \t" << msg.output_percent << "\n"
    //               << "Output Voltage: \t" << msg.output_voltage << "\n"
    //               << "Output Current: \t" << msg.output_current << "\n"
    //               << "Motor Position: \t" << msg.position << "\n"
    //               << "Motor Velocity: \t" << msg.velocity << "\n"
    //     << std::flush;
    // }

    std::shared_ptr<rclcpp::Publisher<talon_msgs::msg::TalonCtrl>> talon_ctrl_pub(rclcpp::Node &parent, const std::string &name) {
        return parent.create_publisher<talon_msgs::msg::TalonCtrl>(name, TALON_CTRL_PUB_QOS);
    }

    // std::shared_ptr<rclcpp::Subscription<talon_msgs::msg::TalonInfo>> talon_info_sub(rclcpp::Node &parent, const std::string &name) {
    //     return parent.create_subscription<talon_msgs::msg::TalonInfo>(name, 10, [this](const talon_msgs::msg::TalonInfo &msg) 
    //         { display_info_actuator(msg); });
    // }

public:
    RobotTeleopInterface(rclcpp::Node &parent) : 
        joy_sub(parent.create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, [this](const sensor_msgs::msg::Joy &joy) { this->joy = joy; }))
        // Motor Information subscribers
        // , hopper_info(talon_info_sub(parent, "hopper_info"))
        // // Motor Teleop Control Publishers
        , track_right_ctrl(talon_ctrl_pub(parent, ROBOT_TOPIC("track_right/ctrl") ))
        , track_left_ctrl(talon_ctrl_pub(parent, ROBOT_TOPIC("track_left/ctrl") ))
        , trencher_ctrl(talon_ctrl_pub(parent, ROBOT_TOPIC("trencher/ctrl") ))
        , hopper_ctrl(talon_ctrl_pub(parent, ROBOT_TOPIC("hopper_belt/ctrl") ))
        , hopper_act_ctrl(talon_ctrl_pub(parent, ROBOT_TOPIC("hopper_act/ctrl") ))
        , teleop_update_timer(
            parent.create_wall_timer(100ms, [this]() { this->update_motors(); }))
        {
        }

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    // rclcpp::Subscription<talon_msgs::msg::TalonInfo>::SharedPtr hopper_info;

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
public:
    Controller() : Node("controller_node")
    , teleop_interface(*this)
    #if ENABLE_HEARTBEAT_PUB
    , heartbeat_timer(this->create_wall_timer(100ms,
        [this]()
        {
                std_msgs::msg::Int32 msg;
                msg.data =
                    ENABLE_TIME.count();
                this->heartbeat->publish(msg);
        }))
    , heartbeat(create_publisher<std_msgs::msg::Int32>("heartbeat", 10))
    #endif
    {

        std::cout << "robot init" << std::endl;
    }

    ~Controller()
    {
    }

private:
    RobotTeleopInterface teleop_interface;

    // sensor_msgs::msg::Joy joy;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    #if ENABLE_HEARTBEAT_PUB
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat;
    #endif

private:

    static constexpr auto ENABLE_TIME = 250ms;

    rclcpp::TimerBase::SharedPtr timer_;
    RobotControlMode current_state = RobotControlMode::DISABLED;

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

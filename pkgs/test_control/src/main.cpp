#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "test_control/logitech_map.hpp"
#include "test_control/states.hpp"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

enum class ControlMode {
    DISABLED,
    TELEOP,
    AUTONOMOUS
};

using namespace std::chrono_literals;

class RobotTeleopInterface {
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

    void display_info_actuator(const custom_types::msg::TalonInfo &msg) {
        std::clog
                  << "Motor Temp: \t" << msg.temperature << "\n"
                  << "Motor Volt: \t" << msg.bus_voltage << "\n"
                  << "Output Percent: \t" << msg.output_percent << "\n"
                  << "Output Voltage: \t" << msg.output_voltage << "\n"
                  << "Output Current: \t" << msg.output_current << "\n"
                  << "Motor Position: \t" << msg.position << "\n"
                  << "Motor Velocity: \t" << msg.velocity << "\n"
        << std::flush;
    }

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> talon_ctrl_pub(rclcpp::Node &parent, const std::string &name) {
        return parent.create_publisher<custom_types::msg::TalonCtrl>(name, 10);
    }

    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>> talon_info_sub(rclcpp::Node &parent, const std::string &name) {
        return parent.create_subscription<custom_types::msg::TalonInfo>(name, 10, [this](const custom_types::msg::TalonInfo &msg) 
            { display_info_actuator(msg); });
    }

public:
    RobotTeleopInterface(rclcpp::Node &parent) : 
        joy_sub(parent.create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, [this](const sensor_msgs::msg::Joy &joy) { this->joy = joy; }))
        // Motor Information subscribers
        // , hopper_info(talon_info_sub(parent, "hopper_info"))
        // // Motor Teleop Control Publishers
        , track_right_ctrl(talon_ctrl_pub(parent, "track_right_ctrl"))
        , track_left_ctrl(talon_ctrl_pub(parent, "track_left_ctrl"))
        , trencher_ctrl(talon_ctrl_pub(parent, "trencher_ctrl"))
        , hopper_ctrl(talon_ctrl_pub(parent, "hopper_belt_ctrl"))
        , hopper_act_ctrl(talon_ctrl_pub(parent, "hopper_ctrl_teleop"))
        , teleop_update_timer(
            parent.create_wall_timer(100ms, [this]() { this->update_motors(); }))
        {
        }

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    rclcpp::Subscription<custom_types::msg::TalonInfo>::SharedPtr hopper_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> track_right_ctrl;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> track_left_ctrl;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> trencher_ctrl;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> hopper_ctrl;
    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> hopper_act_ctrl;

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
    , heartbeat_timer(this->create_wall_timer(100ms,
        [this]()
        {
                std_msgs::msg::Int32 msg;
                msg.data =
                    ENABLE_TIME.count();
                this->heartbeat->publish(msg);
        }))
    , heartbeat(create_publisher<std_msgs::msg::Int32>("heartbeat", 10))
    {

        std::cout << "robot init" << std::endl;
    }

    ~Controller()
    {
    }

private:
    RobotTeleopInterface teleop_interface;

    sensor_msgs::msg::Joy joy;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat;

private:

    static constexpr auto ENABLE_TIME = 250ms;

    rclcpp::TimerBase::SharedPtr timer_;
    ControlMode current_state = ControlMode::DISABLED;

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

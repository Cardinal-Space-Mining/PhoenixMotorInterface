
#include <chrono>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <unistd.h>

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace std::chrono_literals;

using talon_msgs::msg::TalonCtrl;
using talon_msgs::msg::TalonInfo;
using ctre::phoenix::motorcontrol::can::TalonSRX;


enum class RobotControlMode : int8_t    // TODO: disabled should be 0!
{
    TELEOPERATED = 0,
    DISABLED = 1,
    AUTONOMOUS = 2
};

namespace TalonStaticConfig
{
    struct Gains
    {
        double P, I, D, F;
    };

    static const std::string INTERFACE = "can0";
    static constexpr Gains DEFAULT_GAINS{ 0.0, 0.0, 0.0, 0.2 };
    static constexpr Gains DefaultRobotGains{
        0.11,
        0.5,
        0.0001,
        0.12,
    };
}

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS 10
#define ROBOT_CTRL_SUB_QOS 10


class Phoenix5Driver :
    public rclcpp::Node
{
public:
    Phoenix5Driver() :
        Node{ "phoenix5_driver" },
        hopper_info
        {
            this->create_publisher<TalonInfo>(ROBOT_TOPIC("hopper_act/info"), TALON_CTRL_SUB_QOS)
        },
        hopper_ctrl
        {
            this->create_subscription<TalonCtrl>(
                ROBOT_TOPIC("hopper_act/ctrl"),
                TALON_CTRL_SUB_QOS,
                [this](const TalonCtrl &msg){ this->execute_ctrl(this->hopper_actuator, msg); } )
        },
        watchdog_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_feed"),
                ROBOT_CTRL_SUB_QOS,
                [](const std_msgs::msg::Int32 & msg){ ctre::phoenix::unmanaged::Unmanaged::FeedEnable(msg.data); } )
        },
        robot_mode_sub
        {
            this->create_subscription<std_msgs::msg::Int8>(
                ROBOT_TOPIC("robot_mode"),
                ROBOT_CTRL_SUB_QOS,
                [this](const std_msgs::msg::Int8 &msg){ this->update_status_cb(msg); })
        },
        info_timer
        {
            this->create_wall_timer(100ms, [this](){ this->info_periodic_cb(); })
        }
    {
        this->hopper_actuator.Config_kP(0, TalonStaticConfig::DEFAULT_GAINS.P);
        this->hopper_actuator.Config_kI(0, TalonStaticConfig::DEFAULT_GAINS.I);
        this->hopper_actuator.Config_kD(0, TalonStaticConfig::DEFAULT_GAINS.D);
        this->hopper_actuator.Config_kF(0, TalonStaticConfig::DEFAULT_GAINS.F);
        this->hopper_actuator.SetNeutralMode(NeutralMode::Brake);

        RCLCPP_DEBUG(this->get_logger(), "Completed Phoenix5 Driver Node Initialization");
    }

private:
    void execute_ctrl(TalonSRX &motor, const TalonCtrl &msg)
    {
        switch(this->robot_mode)
        {
            case RobotControlMode::TELEOPERATED:
            case RobotControlMode::AUTONOMOUS:
            {
                switch(msg.mode)
                {
                    case TalonCtrl::PERCENT_OUTPUT:
                    {
                        motor.Set(ControlMode::PercentOutput, msg.value);
                        break;
                    }
                    case TalonCtrl::POSITION:
                    {
                        motor.Set(ControlMode::Position, msg.value);
                        break;
                    }
                    case TalonCtrl::VELOCITY:
                    {
                        motor.Set(ControlMode::Velocity, msg.value);
                        break;
                    }
                    case TalonCtrl::CURRENT:
                    {
                        motor.Set(ControlMode::Current, msg.value);
                        break;
                    }
                    case TalonCtrl::VOLTAGE:
                    {
                        // motor.Set(ControlMode::Vol)
                        break;
                    }
                    case TalonCtrl::DISABLED:
                    {
                        motor.Set(ControlMode::Disabled, 0.);
                        return;
                    }
                    case TalonCtrl::FOLLOWER:
                    {
                        motor.Set(ControlMode::Follower, msg.value);
                        break;
                    }
                    case TalonCtrl::MOTION_MAGIC:
                    {
                        motor.Set(ControlMode::MotionMagic, msg.value);
                        break;
                    }
                    case TalonCtrl::MOTION_PROFILE:
                    {
                        motor.Set(ControlMode::MotionProfile, msg.value);
                        break;
                    }
                    case TalonCtrl::MOTION_PROFILE_ARC:
                    {
                        motor.Set(ControlMode::MotionProfileArc, msg.value);
                        break;
                    }
                    case TalonCtrl::MUSIC_TONE:
                    {
                        motor.Set(ControlMode::MusicTone, msg.value);
                        break;
                    }
                }

                RCLCPP_DEBUG(this->get_logger(), "Set motor mode to %d and output to: %f", static_cast<int>(msg.mode), msg.value);
                break;
            }
            case RobotControlMode::DISABLED:
            default:
            {
                motor.Set(ControlMode::Disabled, 0.);
                break;
            }
        }
    }

    inline TalonInfo get_info(TalonSRX &motor)
    {
        TalonInfo info;
        info.header.stamp = this->get_clock()->now();

        info.device_temp    = motor.GetTemperature();
        info.bus_voltage    = motor.GetBusVoltage();
        info.output_percent = motor.GetMotorOutputPercent();
        info.output_voltage = motor.GetMotorOutputVoltage();
        info.output_current = motor.GetOutputCurrent();
        info.position       = motor.GetSelectedSensorPosition();
        info.velocity       = motor.GetSelectedSensorVelocity();

        return info;
    }

    void info_periodic_cb()
    {
        hopper_info->publish(get_info(hopper_actuator));
    }

    void update_status_cb(const std_msgs::msg::Int8 &msg)
    {
        switch(msg.data)
        {
            case 0:
            {
                this->robot_mode = RobotControlMode::TELEOPERATED;
                break;
            }
            case 2:
            {
                this->robot_mode = RobotControlMode::AUTONOMOUS;
                break;
            }
            case 1:
            default:
            {
                this->robot_mode = RobotControlMode::DISABLED;
                this->hopper_actuator.Set(ControlMode::Disabled, 0.);
                break;
            }
        }
    }

private:
    TalonSRX hopper_actuator{4, TalonStaticConfig::INTERFACE};

    rclcpp::Publisher<TalonInfo>::SharedPtr hopper_info;
    rclcpp::Subscription<TalonCtrl>::SharedPtr hopper_ctrl;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr watchdog_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr robot_mode_sub;
    rclcpp::TimerBase::SharedPtr info_timer;

    RobotControlMode robot_mode = RobotControlMode::DISABLED;

};



int main(int argc, char ** argv)
{
    ctre::phoenix::unmanaged::Unmanaged::LoadPhoenix();
    std::cout << "Loaded Phoenix 5 Unmanaged" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix5Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix5) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

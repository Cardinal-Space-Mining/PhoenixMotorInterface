#include <chrono>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/platform/Platform.hpp>
#include <ctre/phoenix/unmanaged/Unmanaged.h>

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"
#include "talon_msgs/msg/talon_faults.hpp"


using namespace std::chrono_literals;

using talon_msgs::msg::TalonCtrl;
using talon_msgs::msg::TalonInfo;
using talon_msgs::msg::TalonFaults;
using ctre::phoenix::motorcontrol::can::TalonSRX;


namespace TalonStaticConfig
{
    struct Gains
    {
        double P, I, D, F;
    };

    static constexpr char const* INTERFACE = "can0";
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

        hopper_actuator{ 4, TalonStaticConfig::INTERFACE },
        hopper_info_pub{ this->create_publisher<TalonInfo>(ROBOT_TOPIC("hopper_act/info"), rclcpp::SensorDataQoS{}) },
        hopper_faults_pub{ this->create_publisher<TalonFaults>(ROBOT_TOPIC("hopper_act/faults"), rclcpp::SensorDataQoS{}) },
        hopper_ctrl_sub
        {
            this->create_subscription<TalonCtrl>(
                ROBOT_TOPIC("hopper_act/ctrl"),
                TALON_CTRL_SUB_QOS,
                [this](const TalonCtrl &msg){ this->execute_ctrl(this->hopper_actuator, msg); } )
        },
        hopper_joint_pub{ this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10) },
        watchdog_status_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_status"),
                rclcpp::SensorDataQoS{},
                [this](const std_msgs::msg::Int32& msg){ this->feed_watchdog_status(msg.data); } )
        },
        info_pub_timer{ this->create_wall_timer(100ms, [this](){ this->pub_motor_info_cb(); }) },
        fault_pub_timer{ this->create_wall_timer(250ms, [this](){ this->pub_motor_fault_cb(); }) }
    {
        this->hopper_actuator.ConfigFactoryDefault();
        this->hopper_actuator.Config_kP(0, TalonStaticConfig::DEFAULT_GAINS.P);
        this->hopper_actuator.Config_kI(0, TalonStaticConfig::DEFAULT_GAINS.I);
        this->hopper_actuator.Config_kD(0, TalonStaticConfig::DEFAULT_GAINS.D);
        this->hopper_actuator.Config_kF(0, TalonStaticConfig::DEFAULT_GAINS.F);
        this->hopper_actuator.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
        this->hopper_actuator.SetInverted(true);
        this->hopper_actuator.SetNeutralMode(NeutralMode::Brake);
        this->hopper_actuator.ConfigNeutralDeadband(5.);
	this->hopper_actuator.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::Analog);
        this->hopper_actuator.ClearStickyFaults();

        RCLCPP_DEBUG(this->get_logger(), "Completed Phoenix5 Driver Node Initialization");
    }

private:
    void feed_watchdog_status(int32_t status);

    void pub_motor_info_cb();
    void pub_motor_fault_cb();

    void execute_ctrl(TalonSRX &motor, const TalonCtrl &msg);

private:
    TalonSRX hopper_actuator;

    rclcpp::Publisher<TalonInfo>::SharedPtr
        hopper_info_pub;
    rclcpp::Publisher<TalonFaults>::SharedPtr
        hopper_faults_pub;
    rclcpp::Subscription<TalonCtrl>::SharedPtr
        hopper_ctrl_sub;
        
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        hopper_joint_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        watchdog_status_sub;
    rclcpp::TimerBase::SharedPtr
        info_pub_timer,
        fault_pub_timer;

    bool is_disabled = false;

};


TalonInfo& operator<<(TalonInfo& info, TalonSRX& m)
{
    info.position       = -m.GetSelectedSensorPosition();
    info.velocity       = m.GetSelectedSensorVelocity();

    info.device_temp    = m.GetTemperature();
    info.bus_voltage    = m.GetBusVoltage();
    info.supply_current = m.GetSupplyCurrent();

    info.output_percent = m.GetMotorOutputPercent();
    info.output_voltage = m.GetMotorOutputVoltage();
    info.output_current = m.GetOutputCurrent();

    info.control_mode   = static_cast<uint8_t>(m.GetControlMode());

    return info;
}

TalonFaults& operator<<(TalonFaults& faults, TalonSRX& m)
{
    Faults f;
    m.GetFaults(f);

    faults.faults = f.ToBitfield();

    faults.hardware_fault = f.HardwareFailure;
    faults.undervoltage_fault = f.UnderVoltage;
    faults.boot_fault = f.ResetDuringEn;
    faults.overvoltage_fault = f.SupplyOverV;
    faults.unstable_voltage_fault = f.SupplyUnstable;

    StickyFaults sf;
    m.GetStickyFaults(sf);

    faults.sticky_faults = sf.ToBitfield();

    faults.sticky_hardware_fault = sf.HardwareESDReset;
    faults.sticky_undervoltage_fault = sf.UnderVoltage;
    faults.sticky_boot_fault = sf.ResetDuringEn;
    faults.sticky_overvoltage_fault = sf.SupplyOverV;
    faults.sticky_unstable_voltage_fault = sf.SupplyUnstable;

    return faults;
}


void Phoenix5Driver::feed_watchdog_status(int32_t status)
{
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    if(!status)
    {
        this->hopper_actuator.Set(ControlMode::Disabled, 0.);
        this->is_disabled = true;
    }
    else
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(std::abs(status));
        this->is_disabled = false;
    }
}

void Phoenix5Driver::pub_motor_info_cb()
{
    TalonInfo info_msg{};
    info_msg.header.stamp = this->get_clock()->now();
    info_msg.enabled = !this->is_disabled;

    this->hopper_info_pub->publish( (info_msg << this->hopper_actuator) );

    sensor_msgs::msg::JointState joint_msg{};
    joint_msg.header.stamp = info_msg.header.stamp;
    joint_msg.name.push_back("dump_joint");
    joint_msg.position.push_back((M_PI / 180.) * (15. + info_msg.position / (-1000. / 30.)));
    this->hopper_joint_pub->publish(joint_msg);
}

void Phoenix5Driver::pub_motor_fault_cb()
{
    TalonFaults faults_msg{};
    faults_msg.header.stamp = this->get_clock()->now();

    this->hopper_faults_pub->publish( (faults_msg << this->hopper_actuator) );
}

void Phoenix5Driver::execute_ctrl(TalonSRX &motor, const TalonCtrl &msg)
{
    if(this->is_disabled)
    {
        motor.Set(ControlMode::Disabled, 0.);
    }
    else
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
    }
}



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

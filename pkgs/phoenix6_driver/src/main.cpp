#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"
#include "talon_msgs/msg/talon_faults.hpp"


#define phx6 ctre::phoenix6
using namespace std::chrono_literals;

using talon_msgs::msg::TalonCtrl;
using talon_msgs::msg::TalonInfo;
using talon_msgs::msg::TalonFaults;
using phx6::hardware::TalonFX;


enum class RobotControlMode : int8_t    // TODO: disabled should be 0!
{
    TELEOPERATED = 0,
    DISABLED = 1,
    AUTONOMOUS = 2
};

namespace TalonStaticConfig
{
    static constexpr char const* INTERFACE = "can0";

    static constexpr double kP = 0.11;
    static constexpr double kI = 0.5;
    static constexpr double kD = 0.0001;
    static constexpr double kV = 0.12;

    static constexpr double DUTY_CYCLE_DEADBAND = 0.05;
    static constexpr int NEUTRAL_MODE = ctre::phoenix6::signals::NeutralModeValue::Brake;

    static constexpr phx6::configs::Slot0Configs
        SLOT0_CONFIG =
            phx6::configs::Slot0Configs{}
                .WithKP(kP)
                .WithKI(kI)
                .WithKD(kD)
                .WithKV(kV);

    static constexpr phx6::configs::MotorOutputConfigs
        MOTOR_OUTPUT_CONFIG =
            phx6::configs::MotorOutputConfigs{}
                .WithDutyCycleNeutralDeadband(DUTY_CYCLE_DEADBAND)  // <-- deadband which applies neutral mode!
                .WithNeutralMode(NEUTRAL_MODE);

    static constexpr phx6::configs::FeedbackConfigs
        FEEDBACK_CONFIGS =
            phx6::configs::FeedbackConfigs{}
                .WithFeedbackSensorSource(phx6::signals::FeedbackSensorSourceValue::RotorSensor);

    static const phx6::configs::CurrentLimitsConfigs    // TODO -- analysis
        CURRENT_LIMIT_CONFIG =
            phx6::configs::CurrentLimitsConfigs{}
                .WithStatorCurrentLimitEnable(true)
                .WithStatorCurrentLimit(100_A)
                .WithSupplyCurrentLowerLimit(50_A)
                .WithSupplyCurrentLowerTime(5_s);
}

#define ROBOT_TOPIC(subtopic) "/lance/" subtopic
#define TALON_CTRL_SUB_QOS 10
#define ROBOT_CTRL_SUB_QOS 10


class Phoenix6Driver :
    public rclcpp::Node
{
protected:
    struct TalonFXPubSub
    {
        rclcpp::Publisher<TalonInfo>::SharedPtr info_pub;
        rclcpp::Publisher<TalonFaults>::SharedPtr faults_pub;

        rclcpp::Subscription<TalonCtrl>::SharedPtr ctrl_sub;
    };

public:
    #define INIT_TALON_PUB_SUB(device) \
        device##_pub_sub \
        { \
            this->create_publisher<TalonInfo>(ROBOT_TOPIC(#device"/info"), rclcpp::SensorDataQoS{}), \
            this->create_publisher<TalonFaults>(ROBOT_TOPIC(#device"/faults"), rclcpp::SensorDataQoS{}), \
            this->create_subscription<TalonCtrl>( \
                ROBOT_TOPIC(#device"/ctrl"), \
                TALON_CTRL_SUB_QOS, \
                [this](const TalonCtrl& msg){ this->execute_ctrl_cb(this->device, msg); } ) \
        }

    Phoenix6Driver() :
        Node{ "phoenix6_driver" },

        track_right{ 0, TalonStaticConfig::INTERFACE },
        track_left{ 1, TalonStaticConfig::INTERFACE },
        trencher{ 2, TalonStaticConfig::INTERFACE },
        hopper_belt{ 3, TalonStaticConfig::INTERFACE },

        INIT_TALON_PUB_SUB(track_right),
        INIT_TALON_PUB_SUB(track_left),
        INIT_TALON_PUB_SUB(trencher),
        INIT_TALON_PUB_SUB(hopper_belt),

        watchdog_sub
        {
            this->create_subscription<std_msgs::msg::Int32>(
                ROBOT_TOPIC("watchdog_feed"),
                ROBOT_CTRL_SUB_QOS,
                [](const std_msgs::msg::Int32& msg)
                {
                    ctre::phoenix::unmanaged::FeedEnable(msg.data);
                } )
        },
        robot_mode_sub
        {
            this->create_subscription<std_msgs::msg::Int8>(
                ROBOT_TOPIC("robot_mode"),
                ROBOT_CTRL_SUB_QOS,
                [this](const std_msgs::msg::Int8& msg)
                {
                    this->update_status_cb(msg);
                } )
        },
        reconfigure_sub
        {
            this->create_subscription<std_msgs::msg::Int8>(
                ROBOT_TOPIC("reconfigure_motors"),
                ROBOT_CTRL_SUB_QOS,
                [this](const std_msgs::msg::Int8& msg)
                {
                    RCLCPP_INFO(this->get_logger(), "%d. Applying motor configs", msg.data);
                    this->configure_motors_cb();
                } )
        },
        info_timer{ this->create_wall_timer(100ms, [this](){ this->info_periodic_cb(); }) }
    {
        this->configure_motors_cb();
        this->neutralAll();

        RCLCPP_DEBUG(this->get_logger(), "Completed Phoenix6 Driver Node Initialization");
    }

    #undef INIT_TALON_PUB_SUB

private:
    inline void neutralAll()
    {
        for(TalonFX& m : this->motors)
        {
            m.SetControl(phx6::controls::NeutralOut{});
        }
    }

    // Function to setup motors for the robot
    void configure_motors_cb();
    // Periodic function for motor information updates
    void info_periodic_cb();
    // Updates robot control mode
    void update_status_cb(const std_msgs::msg::Int8& msg);
    // Function to execute control commands on a motor
    void execute_ctrl_cb(TalonFX& motor, const TalonCtrl& msg);

private:
    TalonFX
        track_right,
        track_left,
        trencher,
        hopper_belt;

    TalonFXPubSub
        track_right_pub_sub,
        track_left_pub_sub,
        trencher_pub_sub,
        hopper_belt_pub_sub;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
        watchdog_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
        robot_mode_sub,
        reconfigure_sub;
    rclcpp::TimerBase::SharedPtr
        info_timer;

    std::array<std::reference_wrapper<TalonFX>, 4>
        motors{ { track_right, track_left, trencher, hopper_belt } };
    std::array<std::reference_wrapper<TalonFXPubSub>, 4>
        motor_pub_subs{ { track_right_pub_sub, track_left_pub_sub, trencher_pub_sub, hopper_belt_pub_sub } };

    RobotControlMode robot_mode = RobotControlMode::DISABLED;

};


TalonInfo& operator<<(TalonInfo& info, TalonFX& m)
{
    info.velocity       = m.GetVelocity().GetValueAsDouble();
    info.position       = m.GetPosition().GetValueAsDouble();
    info.acceleration   = m.GetAcceleration().GetValueAsDouble();

    info.device_temp    = m.GetDeviceTemp().GetValueAsDouble();
    info.processor_temp = m.GetProcessorTemp().GetValueAsDouble();
    info.bus_voltage    = m.GetSupplyVoltage().GetValueAsDouble();
    info.supply_current = m.GetSupplyCurrent().GetValueAsDouble();

    info.output_percent = m.GetDutyCycle().GetValueAsDouble();
    info.output_voltage = m.GetMotorVoltage().GetValueAsDouble();
    info.output_current = m.GetStatorCurrent().GetValueAsDouble();

    info.motor_state    = static_cast<uint8_t>(m.GetMotorOutputStatus().GetValue().value);
    info.bridge_mode    = static_cast<uint8_t>(m.GetBridgeOutput().GetValue().value);
    info.control_mode   = static_cast<uint8_t>(m.GetControlMode().GetValue().value);
    info.enabled        = static_cast<bool>(m.GetDeviceEnable().GetValue().value);

    return info;
}

TalonFaults& operator<<(TalonFaults& faults, TalonFX& m)
{
    faults.faults = m.GetFaultField().GetValue();
    faults.sticky_faults = m.GetStickyFaultField().GetValue();

    faults.sticky_hardware_fault = m.GetStickyFault_Hardware().GetValue();
    faults.sticky_proc_temp_fault = m.GetStickyFault_ProcTemp().GetValue();
    faults.sticky_device_temp_fault = m.GetStickyFault_DeviceTemp().GetValue();
    faults.sticky_undervoltage_fault = m.GetStickyFault_Undervoltage().GetValue();
    faults.sticky_boot_fault = m.GetStickyFault_BootDuringEnable().GetValue();
    faults.sticky_unliscensed_fault = m.GetStickyFault_UnlicensedFeatureInUse().GetValue();
    faults.sticky_bridge_brownout_fault = m.GetStickyFault_BridgeBrownout().GetValue();
    faults.sticky_overvoltage_fault = m.GetStickyFault_OverSupplyV().GetValue();
    faults.sticky_unstable_voltage_fault = m.GetStickyFault_UnstableSupplyV().GetValue();
    faults.sticky_stator_current_limit_fault = m.GetStickyFault_StatorCurrLimit().GetValue();
    faults.sticky_supply_current_limit_fault = m.GetStickyFault_SupplyCurrLimit().GetValue();
    faults.sticky_static_brake_disabled_fault = m.GetStickyFault_StaticBrakeDisabled().GetValue();

    return faults;
}


void Phoenix6Driver::configure_motors_cb()
{
    phx6::configs::TalonFXConfiguration config =
        phx6::configs::TalonFXConfiguration{}
            .WithSlot0(TalonStaticConfig::SLOT0_CONFIG)
            .WithMotorOutput(TalonStaticConfig::MOTOR_OUTPUT_CONFIG)
            .WithFeedback(TalonStaticConfig::FEEDBACK_CONFIGS)
            .WithCurrentLimits(TalonStaticConfig::CURRENT_LIMIT_CONFIG);

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;     // trencher positive should result in digging
    trencher.GetConfigurator().Apply(config);

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;     // hopper positive should result in dumping
    hopper_belt.GetConfigurator().Apply(config);

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::Clockwise_Positive;
    track_right.GetConfigurator().Apply(config);

    config.MotorOutput.Inverted = phx6::signals::InvertedValue::CounterClockwise_Positive;
    track_left.GetConfigurator().Apply(config);

    RCLCPP_INFO(this->get_logger(), "Reconfigured motors.");
}

void Phoenix6Driver::info_periodic_cb()
{
    TalonInfo talon_info_msg{};
    TalonFaults talon_faults_msg{};

    talon_info_msg.header.stamp = talon_faults_msg.header.stamp = this->get_clock()->now();

    for(size_t i = 0; i < 4; i++)
    {
        talon_info_msg << this->motors[i];
        talon_faults_msg << this->motors[i];

        this->motor_pub_subs[i].get().info_pub->publish(talon_info_msg);
        this->motor_pub_subs[i].get().faults_pub->publish(talon_faults_msg);
    }
}

void Phoenix6Driver::update_status_cb(const std_msgs::msg::Int8 &msg)
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
            this->neutralAll();
            break;
        }
    }
}

void Phoenix6Driver::execute_ctrl_cb(TalonFX &motor, const TalonCtrl &msg)
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
                    motor.SetControl(phx6::controls::DutyCycleOut{ msg.value });
                    break;
                }
                case TalonCtrl::POSITION:
                {
                    motor.SetControl(phx6::controls::PositionVoltage{ units::angle::turn_t{ msg.value } });
                    break;
                }
                case TalonCtrl::VELOCITY:
                {
                    motor.SetControl(phx6::controls::VelocityVoltage{ units::angular_velocity::turns_per_second_t{ msg.value } });
                    break;
                }
                case TalonCtrl::CURRENT:
                {
                    // torque/current control required phoenix pro
                    break;
                }
                case TalonCtrl::VOLTAGE:
                {
                    motor.SetControl(phx6::controls::VoltageOut{ units::voltage::volt_t{ msg.value } });
                    break;
                }
                case TalonCtrl::DISABLED:
                {
                    motor.SetControl(phx6::controls::NeutralOut());
                    break;
                }
                case TalonCtrl::FOLLOWER:
                case TalonCtrl::MOTION_MAGIC:
                case TalonCtrl::MOTION_PROFILE:
                case TalonCtrl::MOTION_PROFILE_ARC:
                {
                    break;  // idk
                }
                case TalonCtrl::MUSIC_TONE:
                {
                    motor.SetControl(phx6::controls::MusicTone{ units::frequency::hertz_t{ msg.value } });
                    break;
                }
            }
            break;
        }
        case RobotControlMode::DISABLED:
        default:
        {
            motor.SetControl(phx6::controls::NeutralOut());
            break;
        }
    }
}



int main(int argc, char **argv)
{
    ctre::phoenix::unmanaged::LoadPhoenix();
    std::cout << "Loaded Phoenix 6 Unmanaged" << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix6Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

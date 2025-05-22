#pragma once

#include <variant>
#include <limits>

#include "sensor_msgs/msg/joy.hpp"

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"


using talon_msgs::msg::TalonCtrl;
using talon_msgs::msg::TalonInfo;
using JoyMsg = sensor_msgs::msg::Joy;

struct RobotMotorCommands
{
    TalonCtrl track_right;
    TalonCtrl track_left;
    TalonCtrl trencher;
    TalonCtrl hopper_belt;
    TalonCtrl hopper_actuator;
};

struct RobotMotorStatus
{
    TalonInfo track_right;
    TalonInfo track_left;
    TalonInfo trencher;
    TalonInfo hopper_belt;
    TalonInfo hopper_actuator;
};


class TeleopStateMachine
{
protected:
    enum class RobotControlMode
    {
        Manual,
        Trench,
        Offload
    };
    enum class CommandStage
    {
        Start,
        Norm,
        End
    };

protected:
    struct ManualInfo
    {
        double speed_scalar = 1.0;
    };
    struct OffloadInfo
    {
        CommandStage stage = CommandStage::Start;
        double start_pos = std::numeric_limits<double>::min();
        double track_scalar = 1.0;
    };
    struct TrenchInfo
    {
        CommandStage stage = CommandStage::Start;
    };

public:
    TeleopStateMachine() :
        command_data{ ManualInfo{} }
    {
        RobotMotorStatus info;
        this->setControlMode(RobotControlMode::Manual, info);
    }

public:
    bool update(
        MotorCommands& settings,
        const RobotMotorInfo& robot,
        const JoyMsg& ctrl );

private:
    void setControlMode(
        RobotControlMode mode,
        const RobotMotorInfo& motor_info );

    bool runManualMode(
        MotorCommands& settings,
        const RobotMotorInfo& motor_info,
        const JoyMsg& ctrl );
    bool runTrenchCommand(
        MotorCommands& settings,
        const RobotMotorInfo& motor_info,
        const JoyMsg& ctrl );
    bool runOffloadCommand(
        MotorCommands& settings,
        const RobotMotorInfo& motor_info,
        const JoyMsg& ctrl );

private:
    RobotControlMode current_control_mode;
    std::variant<ManualInfo, OffloadInfo, TrenchInfo> command_data;

};

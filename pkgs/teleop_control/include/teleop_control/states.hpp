#ifndef MISSION_CTRL_STATES_HPP_6_27_2024
#define MISSION_CTRL_STATES_HPP_6_27_2024

#include <variant>
#include <limits>

#include "sensor_msgs/msg/joy.hpp"

#include "talon_msgs/msg/talon_ctrl.hpp"
#include "talon_msgs/msg/talon_info.hpp"

struct MotorSettings
{
    talon_msgs::msg::TalonCtrl track_right;
    talon_msgs::msg::TalonCtrl track_left;
    talon_msgs::msg::TalonCtrl trencher;
    talon_msgs::msg::TalonCtrl hopper_belt;
    talon_msgs::msg::TalonCtrl hopper_actuator;
};

struct RobotState
{
    talon_msgs::msg::TalonInfo track_right;
    talon_msgs::msg::TalonInfo track_left;
    talon_msgs::msg::TalonInfo trencher;
    talon_msgs::msg::TalonInfo hopper_belt;
    talon_msgs::msg::TalonInfo hopper_actuator;
};

class TeleopStateMachine
{
public:
    TeleopStateMachine()
    : state_info{NormalInfo()}
    {
        RobotState state;
        set_state(State::Normal, state);
    }

public:
    MotorSettings update(const RobotState & robot,
                         const sensor_msgs::msg::Joy & ctrl);

private:
    MotorSettings normal_state(const RobotState & robot,
                               const sensor_msgs::msg::Joy & ctrl);
    MotorSettings trench_state(const RobotState & robot,
                               const sensor_msgs::msg::Joy & ctrl);
    MotorSettings offload_state(const RobotState & robot,
                                const sensor_msgs::msg::Joy & ctrl);

private:
    enum class Lifecycle
    {
        Start,
        Norm,
        End
    };

    struct NormalInfo
    {
        double speed_scalar = 1.0;
    };
    struct OffloadInfo
    {
        Lifecycle lifecycle = Lifecycle::Start;
        double start_pos = std::numeric_limits<double>::min();
        double track_scalar = 1.0;
    };

    struct TrenchInfo
    {
        Lifecycle lifecycle = Lifecycle::Start;
    };

    enum class State
    {
        Normal,
        Trench,
        Offload
    };

private:
    void set_state(State state, const RobotState & robot);

    State current_state;
    std::variant<NormalInfo, OffloadInfo, TrenchInfo> state_info;
};

#endif
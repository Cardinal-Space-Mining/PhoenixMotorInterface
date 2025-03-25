#include "test_control/states.hpp"
#include "test_control/logitech_map.hpp"
#include "test_control/robot_constants.hpp"

#include <cmath>
#include <numbers>
#include <iostream>

MotorSettings TeleopStateMachine::update(const RobotState & robot,
                                         const sensor_msgs::msg::Joy & ctrl)
{
    if(ctrl.axes.size() < LogitechMapping::Axes::NUM_AXES ||
       ctrl.buttons.size() < LogitechMapping::Buttons::NUM_BUTTONS)
    {
        return MotorSettings();
    }

    if(ctrl.buttons[LogitechMapping::Buttons::A])
    {
        this->set_state(State::Normal, robot);
        return MotorSettings();
    }

    switch(this->current_state)
    {
    case State::Normal: return normal_state(robot, ctrl);

    case State::Trench: return trench_state(robot, ctrl);

    case State::Offload: return offload_state(robot, ctrl);
    }
    // Unreachable
    throw std::runtime_error("Reached the unreachable");
}

namespace
{
std::array<double, 2> compute_track_scalars(double x, double y,
                                            double mag_deadzone)
{
    const double augmented_angle =
        std::atan2(x, y) +
        (std::numbers::pi /
         4.0); // x and y are inverted to make a CW "heading" angle
    double magnitude = std::sqrt(x * x + y * y);
    if(magnitude < mag_deadzone)
        return {0.0, 0.0};

    return {
        magnitude * std::sin(augmented_angle),
        magnitude *
            std::cos(
                augmented_angle) // this is the same as cos("raw theta" - pi/4) like from the original code
    };
}

double apply_deadband(double value, double deadband)
{
    if(std::abs(value) < deadband)
    {
        return 0.0;
    }
    return value;
}

bool double_near(double first, double second, double epsilon)
{
    return std::abs(second - first) < std::abs(epsilon);
}
} // namespace

MotorSettings
TeleopStateMachine::normal_state(const RobotState & state,
                                 const sensor_msgs::msg::Joy & ctrl)
{
    // State Transitions
    if(RobotConstants::TELEOMETRY)
    {
        if(ctrl.buttons[LogitechMapping::Buttons::L_STICK])
        {
            this->set_state(State::Trench, state);
            return MotorSettings();
        }

        if(ctrl.buttons[LogitechMapping::Buttons::R_STICK])
        {
            this->set_state(State::Offload, state);
            return MotorSettings();
        }

        if(ctrl.axes[LogitechMapping::Axes::DPAD_U_D] ==
           LogitechMapping::Axes::DPAD_K::DPAD_RIGHT)
        {
            this->set_state(State::Offload, state);
            return MotorSettings();
        }

        if(ctrl.axes[LogitechMapping::Axes::DPAD_U_D] ==
           LogitechMapping::Axes::DPAD_K::DPAD_UP)
        {
            this->set_state(State::Trench, state);
            return MotorSettings();
        }
    }

    // Controls
    {
        NormalInfo & state = std::get<NormalInfo>(state_info);
        MotorSettings bot;

        // Speed Buttons
        {
            if(ctrl.buttons[LogitechMapping::Buttons::B])
            {
                state.speed_scalar = 0.3;
            }
            else if(ctrl.buttons[LogitechMapping::Buttons::Y])
            {
                state.speed_scalar = 0.7;
            }
            else if(ctrl.buttons[LogitechMapping::Buttons::X])
            {
                state.speed_scalar = 1.0;
            }
        }

        // Joystick Drive
        {

            auto vars = compute_track_scalars(
                // -ctrl.axes[LogitechMapping::Axes::LEFTX], // commented out for test to fix left right issue
                ctrl.axes[LogitechMapping::Axes::LEFTX],
                ctrl.axes[LogitechMapping::Axes::LEFTY],
                RobotConstants::DRIVING_MAGNITUDE_DEADZONE_SCALAR);
            bot.track_right.mode = bot.track_right.PERCENT_OUTPUT;
            bot.track_right.value =
                vars[0] *
                state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;

            bot.track_left.mode = bot.track_right.PERCENT_OUTPUT;
            bot.track_left.value =
                vars[1] *
                state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;
        }

        // Hopper Up and Down
        {
            bot.hopper_actuator.mode = bot.hopper_actuator.PERCENT_OUTPUT;
            bot.hopper_actuator.value =
                -1.0 * apply_deadband(ctrl.axes[LogitechMapping::Axes::RIGHTY],
                                      RobotConstants::GENERIC_DEADZONE_SCALAR);
        }

        // Hopper Belt
        {
            double trigger_percentage = 1.0 -
                (ctrl.axes[LogitechMapping::Axes::L_TRIGGER] + 1.0) / 2.0;
            if(ctrl.buttons[LogitechMapping::Buttons::LB])
            {
                trigger_percentage *= -1.0;
            }

            bot.hopper_belt.mode = bot.hopper_belt.PERCENT_OUTPUT;
            bot.hopper_belt.value =
                trigger_percentage; //* RobotConstants::HOPPER_BELT_MAX_VELO;
        }

        // Trencher Speed
        {
            double trigger_percentage = 1.0 - 
                (ctrl.axes[LogitechMapping::Axes::R_TRIGGER] + 1.0) / 2.0;

            if(ctrl.buttons[LogitechMapping::Buttons::RB])
            {
                trigger_percentage *= -1.0;
            }

            bot.trencher.mode = bot.trencher.PERCENT_OUTPUT;
            bot.trencher.value =
                trigger_percentage; //* RobotConstants::TRENCHER_MAX_VELO;
        }
        return bot;
    }
}

MotorSettings
TeleopStateMachine::offload_state(const RobotState & robot,
                                  const sensor_msgs::msg::Joy & ctrl)
{
    OffloadInfo & state = std::get<OffloadInfo>(this->state_info);
    MotorSettings bot;

    // Transitions
    {
        // If RStick is pressed
        if(ctrl.buttons[LogitechMapping::Buttons::R_STICK] &&
           state.lifecycle != Lifecycle::End)
        {
            state.lifecycle = Lifecycle::End;
        }

        // Once the hopper is raised
        if(double_near(robot.hopper_actuator.position,
                       RobotConstants::HOPPER_RAISED,
                       RobotConstants::HOPPER_EPSILON) &&
           state.lifecycle == Lifecycle::Start)
        {
            state.lifecycle = Lifecycle::Norm;
        }

        // Once the hopper has rotated once
        if(robot.hopper_belt.position >
           state.start_pos + RobotConstants::OFFLOAD_HOPPER_DELTA)
        {
            state.lifecycle = Lifecycle::End;
        }

        // Once the hopper is lowered
        if(double_near(robot.hopper_actuator.position,
                       RobotConstants::HOPPER_DOWN,
                       RobotConstants::HOPPER_EPSILON) &&
           state.lifecycle == Lifecycle::End)
        {
            this->set_state(State::Normal, robot);
        }
    }
    switch(state.lifecycle)
    {
    case Lifecycle::Start:
        bot.hopper_actuator.mode = bot.hopper_actuator.POSITION;
        bot.hopper_actuator.value = RobotConstants::HOPPER_RAISED;
        break;

    case Lifecycle::Norm:
        bot.hopper_belt.mode = bot.hopper_actuator.VELOCITY;
        bot.hopper_belt.value = RobotConstants::HOPPER_BELT_MAX_VELO;
        bot.hopper_actuator.mode = bot.hopper_actuator.POSITION;
        bot.hopper_actuator.value = RobotConstants::HOPPER_RAISED;
        break;
    case Lifecycle::End:
        bot.hopper_actuator.mode = bot.hopper_actuator.POSITION;
        bot.hopper_actuator.value = RobotConstants::HOPPER_DOWN;
        break;

    default: break;
    }

    // Speed Buttons
    {
        if(ctrl.buttons[LogitechMapping::Buttons::B])
        {
            state.track_scalar = 0.3;
        }
        else if(ctrl.buttons[LogitechMapping::Buttons::Y])
        {
            state.track_scalar = 0.7;
        }
        else if(ctrl.buttons[LogitechMapping::Buttons::X])
        {
            state.track_scalar = 1.0;
        }
    }

    // Tracks back and forth no turn
    {
        bot.track_right.mode = bot.track_right.VELOCITY;
        bot.track_right.value = RobotConstants::TRACKS_OFFLOAD_VELO *
                                state.track_scalar *
                                ctrl.axes[LogitechMapping::Axes::LEFTY];

        bot.track_left.mode = bot.track_left.VELOCITY;
        bot.track_left.value = RobotConstants::TRACKS_OFFLOAD_VELO *
                               state.track_scalar *
                               ctrl.axes[LogitechMapping::Axes::LEFTY];
    }

    return bot;
}

MotorSettings
TeleopStateMachine::trench_state(const RobotState & robot,
                                 const sensor_msgs::msg::Joy & ctrl)
{
    TrenchInfo & state = std::get<TrenchInfo>(state_info);
    MotorSettings bot;

    // Transitions
    {
        // If RStick is pressed
        if(ctrl.buttons[LogitechMapping::Buttons::R_STICK] &&
           state.lifecycle != Lifecycle::End)
        {
            state.lifecycle = Lifecycle::End;
        }

        // If DPAD Down is pressed
        if(ctrl.buttons[LogitechMapping::Axes::DPAD_U_D] ==
               LogitechMapping::Axes::DPAD_K::DPAD_DOWN &&
           state.lifecycle != Lifecycle::End)
        {
            state.lifecycle = Lifecycle::End;
        }

        // Once the hopper Down
        if(double_near(robot.hopper_actuator.position,
                       RobotConstants::TRENCH_DOWN,
                       RobotConstants::TRENCH_EPSILON) &&
           state.lifecycle == Lifecycle::Start)
        {
            state.lifecycle = Lifecycle::Norm;
        }

        // Once the hopper is lowered
        if(double_near(robot.hopper_actuator.position,
                       RobotConstants::HOPPER_DOWN,
                       RobotConstants::HOPPER_EPSILON) &&
           state.lifecycle == Lifecycle::End)
        {
            this->set_state(State::Normal, robot);
        }
    }

    // Control
    {
        // Set Trencher Speed
        {
            double trencher_percent =
                ((ctrl.axes[LogitechMapping::Axes::R_TRIGGER] - 1.0) / 2.0) *
                -1.0;
            bot.trencher.mode = bot.trencher.VELOCITY;
            bot.trencher.value =
                RobotConstants::TRENCHER_NOMINAL_MINING_VELO * trencher_percent;
        }

        // Tracks
        {
            double tracks_speed_percent =
                (ctrl.axes[LogitechMapping::Axes::LEFTY] + 1.0) / 2.0;
            double tracks_speed_value =
                RobotConstants::TRACKS_MINING_VELO * 2.0 * tracks_speed_percent;
            bot.track_right.mode = bot.track_right.VELOCITY;
            bot.track_right.value = tracks_speed_value;

            bot.track_left.mode = bot.track_left.VELOCITY;
            bot.track_left.value = tracks_speed_value;
        }

        // Hopper Level
        if(state.lifecycle == Lifecycle::Norm)
        {
            double hopper_height_percent =
                (ctrl.axes[LogitechMapping::Axes::LEFTY] + 1.0) / 2.0;
            double hopper_height_value =
                ((RobotConstants::TRENCH_DOWN - RobotConstants::HOPPER_LEVEL) *
                 hopper_height_percent) +
                RobotConstants::HOPPER_LEVEL;

            bot.hopper_actuator.mode = bot.hopper_actuator.POSITION;
            bot.hopper_actuator.value = hopper_height_value;
        }
        return bot;
    }
}

void TeleopStateMachine::set_state(State state, const RobotState & robot)
{
    switch(state)
    {
    case State::Normal: state_info = NormalInfo(); break;

    case State::Trench: state_info = TrenchInfo(); break;

    case State::Offload:
    {
        auto new_state = OffloadInfo();
        new_state.start_pos = robot.hopper_belt.position;
        state_info = new_state;
        break;
    }

    default: break;
    }
    this->current_state = state;
}
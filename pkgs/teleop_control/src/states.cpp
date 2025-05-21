#include "teleop_control/states.hpp"
#include "teleop_control/logitech_map.hpp"
#include "teleop_control/robot_constants.hpp"

#include <cmath>
#include <numbers>
#include <iostream>


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





bool TeleopStateMachine::update(
    MotorCommands& settings,
    const RobotMotorInfo& motor_info,
    const JoyMsg& ctrl )
{
    if( ctrl.axes.size() < LogitechMapping::Axes::NUM_AXES ||
        ctrl.buttons.size() < LogitechMapping::Buttons::NUM_BUTTONS )
    {
        return false;
    }

    if(ctrl.buttons[LogitechMapping::Buttons::A])
    {
        this->setControlMode(RobotControlMode::Manual, motor_info);
        return false;
    }

    switch(this->current_control_mode)
    {
        case RobotControlMode::Manual:
        {
            this->runManualMode(settings, motor_info, ctrl);
            break;
        }
        case RobotControlMode::Trench:
        {
            this->runTrenchCommand(settings, motor_info, ctrl);
            break;
        }
        case RobotControlMode::Offload:
        {
            this->runOffloadCommand(settings, motor_info, ctrl);
            break;
        }
        default: return false;
    }

    return true;
}


void TeleopStateMachine::setControlMode(RobotControlMode state, const RobotMotorInfo& motor_info)
{
    switch(state)
    {
        case RobotControlMode::Manual:
        {
            this->command_data = ManualInfo{};
            break;
        }
        case RobotControlMode::Trench:
        {
            this->command_data = TrenchInfo{};
            break;
        }
        case RobotControlMode::Offload:
        {
            this->command_data = OffloadInfo{ .start_pos = motor_info.hopper_belt.position };
            break;
        }
        default: break;
    }
    this->current_control_mode = state;
}

bool TeleopStateMachine::runManualMode(
    MotorCommands& settings,
    const RobotMotorInfo& motor_info,
    const JoyMsg& ctrl )
{
    // State Transitions
    if(RobotConstants::TELEOMETRY)
    {
        if(ctrl.buttons[LogitechMapping::Buttons::L_STICK])
        {
            this->setControlMode(RobotControlMode::Trench, motor_info);
            return false;
        }

        if(ctrl.buttons[LogitechMapping::Buttons::R_STICK])
        {
            this->setControlMode(RobotControlMode::Offload, motor_info);
            return false;
        }

        if( ctrl.axes[LogitechMapping::Axes::DPAD_U_D] ==
            LogitechMapping::Axes::DPAD_K::DPAD_RIGHT )
        {
            this->setControlMode(RobotControlMode::Offload, motor_info);
            return false;
        }

        if( ctrl.axes[LogitechMapping::Axes::DPAD_U_D] ==
            LogitechMapping::Axes::DPAD_K::DPAD_UP )
        {
            this->setControlMode(RobotControlMode::Trench, motor_info);
            return false;
        }
    }

    // Controls
    {
        ManualInfo& control_state = std::get<ManualInfo>(command_data);

        // Speed Buttons
        {
            if(ctrl.buttons[LogitechMapping::Buttons::B])
            {
                control_state.speed_scalar = 0.3;
            }
            else if(ctrl.buttons[LogitechMapping::Buttons::Y])
            {
                control_state.speed_scalar = 0.7;
            }
            else if(ctrl.buttons[LogitechMapping::Buttons::X])
            {
                control_state.speed_scalar = 1.0;
            }
        }

        // Joystick Drive
        {
            auto vars = compute_track_scalars(
                // -ctrl.axes[LogitechMapping::Axes::LEFTX], // commented out for test to fix left right issue
                ctrl.axes[LogitechMapping::Axes::LEFTX],
                ctrl.axes[LogitechMapping::Axes::LEFTY],
                RobotConstants::DRIVING_MAGNITUDE_DEADZONE_SCALAR);
            settings.track_right.mode = settings.track_right.PERCENT_OUTPUT;
            settings.track_right.value =
                vars[0] *
                control_state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;

            settings.track_left.mode = settings.track_right.PERCENT_OUTPUT;
            settings.track_left.value =
                vars[1] *
                control_state.speed_scalar; //* RobotConstants::TRACKS_MAX_VELO;
        }

        // Hopper Up and Down
        {
            settings.hopper_actuator.mode = settings.hopper_actuator.PERCENT_OUTPUT;
            settings.hopper_actuator.value =
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

            settings.hopper_belt.mode = settings.hopper_belt.PERCENT_OUTPUT;
            settings.hopper_belt.value =
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
                
                settings.trencher.mode = settings.trencher.PERCENT_OUTPUT;
                settings.trencher.value =
                trigger_percentage; //* RobotConstants::TRENCHER_MAX_VELO;
        }
    }

    return true;
}

bool TeleopStateMachine::runTrenchCommand(
    MotorCommands& settings,
    const RobotMotorInfo& motor_info,
    const JoyMsg& ctrl )
{
    TrenchInfo& command_state = std::get<TrenchInfo>(command_data);

    // Transitions
    {
        // If RStick is pressed
        if( ctrl.buttons[LogitechMapping::Buttons::R_STICK] &&
            command_state.stage != CommandStage::End )
        {
            command_state.stage = CommandStage::End;
        }

        // If DPAD Down is pressed
        if( ctrl.buttons[LogitechMapping::Axes::DPAD_U_D] ==
                LogitechMapping::Axes::DPAD_K::DPAD_DOWN &&
            command_state.stage != CommandStage::End )
        {
            command_state.stage = CommandStage::End;
        }

        // Once the hopper Down
        if( double_near(motor_info.hopper_actuator.position,
                        RobotConstants::TRENCH_DOWN,
                        RobotConstants::TRENCH_EPSILON) &&
            command_state.stage == CommandStage::Start )
        {
            command_state.stage = CommandStage::Norm;
        }

        // Once the hopper is lowered
        if( double_near(motor_info.hopper_actuator.position,
                        RobotConstants::HOPPER_DOWN,
                        RobotConstants::HOPPER_EPSILON) &&
            command_state.stage == CommandStage::End )
        {
            this->setControlMode(RobotControlMode::Manual, motor_info);
        }
    }

    // Control
    {
        // Set Trencher Speed
        {
            double trencher_percent =
                ((ctrl.axes[LogitechMapping::Axes::R_TRIGGER] - 1.0) / 2.0) *
                -1.0;
            settings.trencher.mode = settings.trencher.VELOCITY;
            settings.trencher.value =
                RobotConstants::TRENCHER_NOMINAL_MINING_VELO * trencher_percent;
        }

        // Tracks
        {
            double tracks_speed_percent =
                (ctrl.axes[LogitechMapping::Axes::LEFTY] + 1.0) / 2.0;
            double tracks_speed_value =
                RobotConstants::TRACKS_MINING_VELO * 2.0 * tracks_speed_percent;
            settings.track_right.mode = settings.track_right.VELOCITY;
            settings.track_right.value = tracks_speed_value;

            settings.track_left.mode = settings.track_left.VELOCITY;
            settings.track_left.value = tracks_speed_value;
        }

        // Hopper Level
        if(command_state.stage == CommandStage::Norm)
        {
            double hopper_height_percent =
                (ctrl.axes[LogitechMapping::Axes::LEFTY] + 1.0) / 2.0;
            double hopper_height_value =
                ((RobotConstants::TRENCH_DOWN - RobotConstants::HOPPER_LEVEL) *
                    hopper_height_percent) +
                RobotConstants::HOPPER_LEVEL;

            settings.hopper_actuator.mode = settings.hopper_actuator.POSITION;
            settings.hopper_actuator.value = hopper_height_value;
        }
    }

    return true;
}

bool TeleopStateMachine::runOffloadCommand(
    MotorCommands& settings,
    const RobotMotorInfo& motor_info,
    const JoyMsg& ctrl )
{
    OffloadInfo& state = std::get<OffloadInfo>(this->command_data);

    // Transitions
    {
        // If RStick is pressed
        if(ctrl.buttons[LogitechMapping::Buttons::R_STICK] &&
           state.stage != CommandStage::End)
        {
            state.stage = CommandStage::End;
        }

        // Once the hopper is raised
        if(double_near(motor_info.hopper_actuator.position,
                       RobotConstants::HOPPER_RAISED,
                       RobotConstants::HOPPER_EPSILON) &&
           state.stage == CommandStage::Start)
        {
            state.stage = CommandStage::Norm;
        }

        // Once the hopper has rotated once
        if(motor_info.hopper_belt.position >
           state.start_pos + RobotConstants::OFFLOAD_HOPPER_DELTA)
        {
            state.stage = CommandStage::End;
        }

        // Once the hopper is lowered
        if(double_near(motor_info.hopper_actuator.position,
                       RobotConstants::HOPPER_DOWN,
                       RobotConstants::HOPPER_EPSILON) &&
           state.stage == CommandStage::End)
        {
            this->setControlMode(RobotControlMode::Manual, motor_info);
        }
    }
    switch(state.stage)
    {
        case CommandStage::Start:
        {
            settings.hopper_actuator.mode = settings.hopper_actuator.POSITION;
            settings.hopper_actuator.value = RobotConstants::HOPPER_RAISED;
            break;
        }
        case CommandStage::Norm:
        {
            settings.hopper_belt.mode = settings.hopper_actuator.VELOCITY;
            settings.hopper_belt.value = RobotConstants::HOPPER_BELT_MAX_VELO;
            settings.hopper_actuator.mode = settings.hopper_actuator.POSITION;
            settings.hopper_actuator.value = RobotConstants::HOPPER_RAISED;
            break;
        }
        case CommandStage::End:
        {
            settings.hopper_actuator.mode = settings.hopper_actuator.POSITION;
            settings.hopper_actuator.value = RobotConstants::HOPPER_DOWN;
            break;
        }
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
        settings.track_right.mode = settings.track_right.VELOCITY;
        settings.track_right.value = RobotConstants::TRACKS_OFFLOAD_VELO *
                                state.track_scalar *
                                ctrl.axes[LogitechMapping::Axes::LEFTY];

        settings.track_left.mode = settings.track_left.VELOCITY;
        settings.track_left.value = RobotConstants::TRACKS_OFFLOAD_VELO *
                               state.track_scalar *
                               ctrl.axes[LogitechMapping::Axes::LEFTY];
    }

    return true;
}

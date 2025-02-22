#ifndef ROBOT_CONSTANTS_08_20_24_HPP
#define ROBOT_CONSTANTS_08_20_24_HPP

constexpr long falcon_tps_to_pheonix_5_velo(long tps)
{
    constexpr long FALCON_CPR = 2048;
    return tps * FALCON_CPR * 0.1;
}

namespace RobotConstants
{
static constexpr long
    // motor physical speed targets
    TRENCHER_MAX_VELO =
        falcon_tps_to_pheonix_5_velo(80), // maximum mining speed
    TRENCHER_NOMINAL_MINING_VELO =
        falcon_tps_to_pheonix_5_velo(80), // base trenching speed
    HOPPER_BELT_MAX_VELO = falcon_tps_to_pheonix_5_velo(45),
    HOPPER_BELT_MAX_MINING_VELO = falcon_tps_to_pheonix_5_velo(10),
    TRACKS_MAX_VELO = falcon_tps_to_pheonix_5_velo(125),
    TRACKS_MINING_VELO = falcon_tps_to_pheonix_5_velo(8),
    TRACKS_MAX_ADDITIONAL_MINING_VEL = falcon_tps_to_pheonix_5_velo(6);
static constexpr long TRACKS_OFFLOAD_VELO = TRACKS_MAX_VELO * 0.25;

// static constexpr auto MOTOR_SETPOINT_ACC = 5_tr_per_s_sq;

static constexpr double
    // motor constants
    GENERIC_MOTOR_kP =
        0.11, // An error of 1 rotation per second results in 2V output
    GENERIC_MOTOR_kI = 0.5,    // An error of 1 rotation per second increases
                               // output by 0.5V every second
    GENERIC_MOTOR_kD = 0.0001, // A change of 1 rotation per second squared
                               // results in 0.0001 volts output
    GENERIC_MOTOR_kV =
        0.12, // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
              // driving
    DRIVING_MAGNITUDE_DEADZONE_SCALAR = 0.1, DRIVING_LOW_SPEED_SCALAR = 0.3,
    DRIVING_MEDIUM_SPEED_SCALAR = 0.7, DRIVING_HIGH_SPEED_SCALAR = 1.0,
    GENERIC_DEADZONE_SCALAR = 0.05,
    // hopper
    HOPPER_ACTUATOR_PLUNGE_SPEED = 0.40, HOPPER_ACTUATOR_EXTRACT_SPEED = 0.80,
    HOPPER_ACUTATOR_MOVE_SPEED = 1.0, // all other movement (ie. dumping)
                                      // actuator potentiometer target values
    OFFLOAD_POT_VALUE = 0.95,         // dump height
    TRAVERSAL_POT_VALUE = 0.60,       // traversal height
    AUTO_TRANSPORT_POT_VALUE = 0.55,  // height for transporting regolith
    MINING_DEPTH_NOMINAL_POT_VALUE =
        0.21, // nominal mining depth from which manual adjustments can be made
    MINING_DEPTH_LIMIT_POT_VALUE = 0.03, // lowest depth we ever want to go
    HOPPER_POT_TARGETING_EPSILON = 0.01,
    // timed operations
    MINING_RUN_TIME_SECONDS = 1.0,          // teleauto mining run time
    TELE_OFFLOAD_BACKUP_TIME_SECONDS = 3.0, // teleauto offload duration
    AUTO_OFFLOAD_BACKUP_TIME_SECONDS = 2.0, OFFLOAD_DUMP_TIME = 6.0,
    // auto belt duty cycle
    HOPPER_BELT_TIME_ON_SECONDS = 1.0, HOPPER_BELT_TIME_OFF_SECONDS = 2.5;


    // TODO
    constexpr static double HOPPER_RAISED = 0.0, HOPPER_LEVEL = 0.0, HOPPER_DOWN = 0.0, HOPPER_EPSILON = 100;
    constexpr static double OFFLOAD_HOPPER_DELTA = 0.0, OFFLOAD_HOPPER_EPSILON = 100;
    constexpr static double TRENCH_DOWN = 0.0, TRENCH_EPSILON = 0.0;

    
    //Compile time teleometry toggle
    constexpr static bool TELEOMETRY = false;


} // namespace RobotConstants

#endif
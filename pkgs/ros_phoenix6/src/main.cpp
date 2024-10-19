#include <memory>
#include <span>
#include <utility>
#include <vector>
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix/export.h"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

int main(int argc, char ** argv)
{
    // ROS 2 initialization and node spinning
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<Robot>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}

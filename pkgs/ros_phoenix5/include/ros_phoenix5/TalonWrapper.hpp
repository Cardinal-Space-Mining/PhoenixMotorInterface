#include <rclcpp/rclcpp.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/phoenix/export.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

#pragma GCC diagnostic pop


#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

using TalonFX6 = ctre::phoenix6::hardware::TalonFX;
using TalonFX5 = ctre::phoenix::motorcontrol::can::TalonFX;
using TalonSRX5 = ctre::phoenix::motorcontrol::can::TalonSRX;

using TalonCtrlSub =
    rclcpp::Subscription<custom_types::msg::TalonCtrl>::SharedPtr;
using TalonInfoPub = rclcpp::Publisher<custom_types::msg::TalonInfo>::SharedPtr;

struct Gains
{
    double P, I, D, F;
};

enum class BrakeMode : uint8_t
{
    COAST = 0,
    BRAKE = 1
};

// Using explicit template instatiation because they do not expose a common API,
// but I want it to seem like they do
template <typename T>
class TalonWrapper
{
public:
    TalonWrapper(rclcpp::Node & parent_node, std::string name, int id);
    ~TalonWrapper() = default;

public:
    void config(const Gains & gains, BrakeMode break_mode);
    void on_ctrl_msg(const custom_types::msg::TalonCtrl & msg);
    custom_types::msg::TalonInfo get_info();

public:
    // Copy constructor (deleted)
    TalonWrapper(const TalonWrapper &) = delete;

    // Copy assignment operator (deleted)
    TalonWrapper & operator=(const TalonWrapper &) = delete;

    // Move constructor (deleted)
    TalonWrapper(TalonWrapper &&) = delete;

    // Move assignment operator (deleted)
    TalonWrapper & operator=(TalonWrapper &&) = delete;

private:
    static std::string ctrl_name(std::string name) { return name + "_ctrl"; }

    static std::string info_name(std::string name) { return name + "_info"; }

    TalonCtrlSub create_ctrl_sub(rclcpp::Node & parent_node, std::string name)
    {
        return parent_node.create_subscription<custom_types::msg::TalonCtrl>(
            this->ctrl_name(name), 10,
            [this](const custom_types::msg::TalonCtrl & msg)
            { this->on_ctrl_msg(msg); });
    }

    TalonInfoPub create_info_pub(rclcpp::Node & parent_node, std::string name)
    {
        return parent_node.create_publisher<custom_types::msg::TalonInfo>(
            this->info_name(name), 10);
    }

    rclcpp::TimerBase::SharedPtr create_info_timer(rclcpp::Node & parent_node)
    {
        using namespace std::chrono_literals;
        return parent_node.create_timer(100ms,
                                        [this]()
                                        {
                                            auto info = this->get_info();
                                            this->m_pub->publish(info);
                                        });
    }

private:
    T m_motor;
    TalonCtrlSub m_sub;
    TalonInfoPub m_pub;
    rclcpp::TimerBase::SharedPtr m_info_timer;
};

// Explicit template instantiation
template class TalonWrapper<TalonFX6>;
template class TalonWrapper<TalonFX5>;
template class TalonWrapper<TalonSRX>;

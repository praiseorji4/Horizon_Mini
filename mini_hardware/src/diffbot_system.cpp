#include "mini_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mini_hardware
{

hardware_interface::CallbackReturn DiffDriveMiniHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.front_left_wheel_name  = info_.hardware_parameters["front_left_wheel_name"];
    cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
    cfg_.rear_left_wheel_name   = info_.hardware_parameters["rear_left_wheel_name"];
    cfg_.rear_right_wheel_name  = info_.hardware_parameters["rear_right_wheel_name"];
    cfg_.loop_rate              = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
    cfg_.device                 = info_.hardware_parameters["device"];
    cfg_.baud_rate              = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms             = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev     = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    if (info_.hardware_parameters.count("pid_p") > 0)
    {
        cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
        cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
        cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
        cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "PID values not supplied, using defaults.");
    }

    wheel_fl_.setup(cfg_.front_left_wheel_name,  cfg_.enc_counts_per_rev);
    wheel_fr_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
    wheel_rl_.setup(cfg_.rear_left_wheel_name,   cfg_.enc_counts_per_rev);
    wheel_rr_.setup(cfg_.rear_right_wheel_name,  cfg_.enc_counts_per_rev);

    RCLCPP_INFO(get_logger(),
        "rads_per_count: fl=%.6f fr=%.6f rl=%.6f rr=%.6f",
        wheel_fl_.rads_per_count, wheel_fr_.rads_per_count,
        wheel_rl_.rads_per_count, wheel_rr_.rads_per_count);

    // Validate joints
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has %zu command interfaces. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' command interface. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has %zu state interfaces. 2 expected.",
                joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(get_logger(),
                "Joint '%s' has '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveMiniHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos);
    state_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel);

    state_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos);
    state_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel);

    state_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_POSITION, &wheel_rl_.pos);
    state_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.vel);

    state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_POSITION, &wheel_rr_.pos);
    state_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.vel);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveMiniHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd);
    command_interfaces.emplace_back(wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd);
    command_interfaces.emplace_back(wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.cmd);
    command_interfaces.emplace_back(wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd);

    return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveMiniHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    if (comms_.connected()) comms_.disconnect();
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMiniHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
    if (comms_.connected()) comms_.disconnect();
    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMiniHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    if (!comms_.connected())
    {
        RCLCPP_ERROR(get_logger(), "Cannot activate: serial not connected.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (cfg_.pid_p > 0)
    {
        comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
    }
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMiniHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    // Stop all motors on deactivate
    if (comms_.connected())
    {
        comms_.set_motor_values(0, 0, 0, 0);
    }
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveMiniHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    if (!comms_.connected()) return hardware_interface::return_type::ERROR;

    comms_.read_encoder_values(wheel_fl_.enc, wheel_fr_.enc, wheel_rl_.enc, wheel_rr_.enc);

    RCLCPP_DEBUG(get_logger(),
        "ENC fl=%d fr=%d rl=%d rr=%d",
        wheel_fl_.enc, wheel_fr_.enc, wheel_rl_.enc, wheel_rr_.enc);

    double delta_seconds = period.seconds();
    if (delta_seconds == 0.0)
    {
        RCLCPP_WARN_ONCE(get_logger(), "read() called with zero period; skipping velocity update.");
        return hardware_interface::return_type::OK;
    }

    auto update_wheel = [&](Wheel & w) {
        double pos_prev = w.pos;
        w.pos = w.calc_enc_angle();
        w.vel = (w.pos - pos_prev) / delta_seconds;
    };

    update_wheel(wheel_fl_);
    update_wheel(wheel_fr_);
    update_wheel(wheel_rl_);
    update_wheel(wheel_rr_);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveMiniHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!comms_.connected()) return hardware_interface::return_type::ERROR;

    int fl = static_cast<int>(wheel_fl_.cmd / wheel_fl_.rads_per_count / cfg_.loop_rate);
    int fr = static_cast<int>(wheel_fr_.cmd / wheel_fr_.rads_per_count / cfg_.loop_rate);
    int rl = static_cast<int>(wheel_rl_.cmd / wheel_rl_.rads_per_count / cfg_.loop_rate);
    int rr = static_cast<int>(wheel_rr_.cmd / wheel_rr_.rads_per_count / cfg_.loop_rate);

    RCLCPP_DEBUG(get_logger(),
        "CMD rad/s: fl=%.3f fr=%.3f rl=%.3f rr=%.3f  →  counts: fl=%d fr=%d rl=%d rr=%d",
        wheel_fl_.cmd, wheel_fr_.cmd, wheel_rl_.cmd, wheel_rr_.cmd,
        fl, fr, rl, rr);

    comms_.set_motor_values(fl, fr, rl, rr);
    return hardware_interface::return_type::OK;
}

}  // namespace mini_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mini_hardware::DiffDriveMiniHardware, hardware_interface::SystemInterface)
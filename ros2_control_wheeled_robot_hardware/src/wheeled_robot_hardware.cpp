#include "ros2_control_wheeled_robot_hardware/wheeled_robot_hardware.hpp"
#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_wheeled_robot_hardware
{
WheeledRobotHardware::WheeledRobotHardware()
: Node("wheeled_robot_hardware")
{
  udp_socket_ = std::make_unique<Eth_Socket>();
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Initialized joint: %s", joint.name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Configuring ...please wait...");

  for (uint i = 0; i < hw_velocities_.size(); i++)
  {
    hw_velocities_[i] = 0;
    hw_positions_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
WheeledRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
WheeledRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Activating ...please wait...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = hw_velocities_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheeledRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double wheel_velocities[6] = {0.0};
  double wheel_positions[6] = {0.0};

  // Получаем и скорости, и позиции
  udp_socket_->GetWheelStates(wheel_velocities, wheel_positions);

  // Обновляем состояния
  for (uint i = 0; i < hw_velocities_.size(); i++) {
    hw_velocities_[i] = wheel_velocities[i];
    
    // Если позиции не приходят от устройства, интегрируем скорости
    if (wheel_positions[i] == 0.0) {
      hw_positions_[i] += hw_velocities_[i] * period.seconds();
    } else {
      hw_positions_[i] = wheel_positions[i];
    }
  }

  rclcpp::spin_some(this->get_node_base_interface());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (info_.joints.size() >= 6)
  {
    double wheel_velocities[6];
    for (uint i = 0; i < 6; ++i)
    {
      wheel_velocities[i] = hw_commands_[i];
    }

    double wheel_separation = 0.8;
    double wheel_radius = 0.245;

    double left_wheel_velocity = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2]) / 3.0 * wheel_radius;
    double right_wheel_velocity = (wheel_velocities[3] + wheel_velocities[4] + wheel_velocities[5]) / 3.0 * wheel_radius;

    double linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_separation;

    double vel_command[2];
    vel_command[0] = linear_velocity;
    vel_command[1] = angular_velocity;

    udp_socket_->SendWheelSpeeds(vel_command);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("WheeledRobotHardware"), "Not enough joints to send commands.");
  }

  rclcpp::spin_some(this->get_node_base_interface());

  return hardware_interface::return_type::OK;
}
}  // namespace ros2_control_wheeled_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_wheeled_robot_hardware::WheeledRobotHardware, hardware_interface::SystemInterface)
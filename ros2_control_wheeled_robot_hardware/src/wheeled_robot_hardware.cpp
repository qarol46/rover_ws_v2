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
  // Инициализация UDP-сокета
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

  // Инициализация векторов для состояний и команд
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheeledRobotHardware"), "Configuring ...please wait...");

  // Сброс состояний и команд
  for (uint i = 0; i < hw_velocities_.size(); i++)
  {
    hw_velocities_[i] = 0;
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

  // Команды и состояния должны быть равны при активации
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Получение текущих скоростей колес через UDP
  double wheel_speeds[6];
  udp_socket_->GetWheelSpeeds(wheel_speeds);

  // Обновление состояний
  for (uint i = 0; i < hw_velocities_.size(); i++)
  {
    hw_velocities_[i] = wheel_speeds[i];
  }

  // Обработка сообщений ROS 2
  rclcpp::spin_some(this->get_node_base_interface());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Отправка целевых скоростей колес через UDP
  udp_socket_->SendWheelSpeeds(hw_commands_.data());

  // Обработка сообщений ROS 2
  rclcpp::spin_some(this->get_node_base_interface());

  return hardware_interface::return_type::OK;
}
}  // namespace ros2_control_wheeled_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_wheeled_robot_hardware::WheeledRobotHardware, hardware_interface::SystemInterface)
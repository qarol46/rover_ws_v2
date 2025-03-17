cmake_minimum_required(VERSION 3.5)
project(ros2_control_wheeled_robot_hardware)

# Поиск необходимых зависимостей
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Добавление исходных файлов
add_library(wheeled_robot_hardware
  src/wheeled_robot_hardware.cpp
  src/udp_wheeled_robot.cpp
)

# Указание зависимостей для библиотеки
target_include_directories(wheeled_robot_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wheeled_robot_hardware
  hardware_interface
  rclcpp
  pluginlib
)

# Установка библиотеки
install(TARGETS
  wheeled_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Установка заголовочных файлов
install(DIRECTORY include/
  DESTINATION include
)

# Экспорт плагина для ROS 2 Control
pluginlib_export_plugin_description_file(hardware_interface wheeled_robot_hardware.xml)

# Экспорт зависимостей
ament_export_dependencies(
  hardware_interface
  rclcpp
  pluginlib
)

# Создание пакета
ament_package()
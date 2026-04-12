cmake_minimum_required(VERSION 3.8)
project(cpp_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. Crear los ejecutables
add_executable(twist_sequencer src/twist_sequencer.cpp)
add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(data_logger src/data_logger_node.cpp)


# 3. Vincular dependencias a los ejecutables
ament_target_dependencies(twist_sequencer rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(odom_publisher rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(data_logger rclcpp std_msgs nav_msgs)

# 4. Instalar los ejecutables (para que 'ros2 run' los encuentre)
install(TARGETS
  twist_sequencer
  odom_publisher
  data_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
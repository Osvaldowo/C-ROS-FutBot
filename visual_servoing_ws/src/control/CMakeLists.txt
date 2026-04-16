cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. Encontrar dependencias de ROS 2 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --- 2. Nodos de Control Cinemático ---
# (Asegúrate de que estos archivos .cpp estén dentro de control/src/)

add_executable(pi_straight_sequencer src/pi_straight_sequencer.cpp)
ament_target_dependencies(pi_straight_sequencer rclcpp geometry_msgs nav_msgs)

add_executable(pi_turn_sequencer src/pi_turn_sequencer.cpp)
ament_target_dependencies(pi_turn_sequencer rclcpp geometry_msgs nav_msgs)

add_executable(visual_tracker_node src/visual_tracker_node.cpp)
ament_target_dependencies(visual_tracker_node rclcpp geometry_msgs)

add_executable(ball_planner src/ball_planner.cpp)
ament_target_dependencies(ball_planner rclcpp geometry_msgs nav_msgs)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node rclcpp geometry_msgs nav_msgs)

# --- 3. Reglas de Instalación ---
install(TARGETS
  pi_straight_sequencer
  pi_turn_sequencer
  visual_tracker_node
  ball_planner
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
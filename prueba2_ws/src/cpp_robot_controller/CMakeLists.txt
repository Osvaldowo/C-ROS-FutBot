cmake_minimum_required(VERSION 3.8)
project(cpp_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --- NODO SECUENCIADOR PI ---
add_executable(pi_turn_sequencer src/pi_turn_sequencer.cpp)
ament_target_dependencies(pi_turn_sequencer 
  rclcpp 
  geometry_msgs 
  nav_msgs
)

add_executable(pi_straight_sequencer src/pi_straight_sequencer.cpp)
ament_target_dependencies(pi_straight_sequencer 
  rclcpp 
  geometry_msgs
  nav_msgs
)

# Instalamos el ejecutable
install(TARGETS
  pi_turn_sequencer
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pi_turn_sequencer
  pi_straight_sequencer
  DESTINATION lib/${PROJECT_NAME}
)

# ¡VITAL PARA SIMULACIÓN! Instalar carpetas de launch y urdf
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
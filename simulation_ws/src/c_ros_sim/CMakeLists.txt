cmake_minimum_required(VERSION 3.8)
project(c_ros_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  launch 
  urdf 
  meshes 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- NUEVO: Instalamos el script de teleoperación dual ---
# Esto permite que 'ros2 run' encuentre tu archivo de Python
install(PROGRAMS
  scripts/dual_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
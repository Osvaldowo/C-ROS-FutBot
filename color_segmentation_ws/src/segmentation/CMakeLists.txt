cmake_minimum_required(VERSION 3.8)
project(segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. ENCONTRAR DEPENDENCIAS ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# --- 2. CONFIGURAR EJECUTABLES (NODOS) ---

# Nodo de la Cámara
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${OpenCV_LIBRARIES})
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge)

# Nodo de Calibración
add_executable(calibrator_node src/calibrator_node.cpp)
target_link_libraries(calibrator_node ${OpenCV_LIBRARIES})
ament_target_dependencies(calibrator_node rclcpp sensor_msgs std_msgs cv_bridge)

# Nodo Detector (Filtro Alfa, Z, CLAHE)
add_executable(detector_node src/detector_node.cpp)
target_link_libraries(detector_node ${OpenCV_LIBRARIES})
ament_target_dependencies(detector_node rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge)

# --- 3. REGLAS DE INSTALACIÓN ---

# Instalar los binarios compilados
install(TARGETS
  camera_node
  calibrator_node
  detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar carpetas de lanzamiento y configuración
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
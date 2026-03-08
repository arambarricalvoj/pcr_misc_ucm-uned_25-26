cmake_minimum_required(VERSION 3.8)
project(tarea2)

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)


include_directories(include)

# Biblioteca con la lógica del controlador y utilidades
add_library(tarea2_lib
    src/braitenberg_target_controller.cpp
    src/braitenberg_controller.cpp
    src/braitenberg.cpp
    src/service_utils.cpp
    src/metrics_utils.cpp
)

ament_target_dependencies(tarea2_lib
    rclcpp
    rclcpp_action
    std_srvs
    geometry_msgs
    sensor_msgs
    ament_index_cpp
    interfaces
)

# Ejecutable principal
add_executable(braitenberg_target_controller src/main_braitenberg_target_controller.cpp)
target_link_libraries(braitenberg_target_controller tarea2_lib)

# Instalación
install(TARGETS braitenberg_target_controller
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(braitenberg_controller src/main_braitenberg_controller.cpp)
target_link_libraries(braitenberg_controller tarea2_lib)

# Instalación
install(TARGETS braitenberg_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
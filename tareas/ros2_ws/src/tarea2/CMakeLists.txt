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
    src/khepera_controller.cpp
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
add_executable(khepera_controller src/main.cpp)
target_link_libraries(khepera_controller tarea2_lib)

# Instalación
install(TARGETS khepera_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(tarea1)

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)


include_directories(include)

# Biblioteca con la lógica del controlador y utilidades
add_library(tarea1_lib
    src/icc_controller.cpp
    src/service_utils.cpp
    src/metrics_utils.cpp
)

ament_target_dependencies(tarea1_lib
    rclcpp
    rclcpp_action
    std_srvs
    geometry_msgs
    ament_index_cpp
    interfaces
)

# Ejecutable principal
add_executable(icc_controller src/main.cpp)
target_link_libraries(icc_controller tarea1_lib)

# Instalación
install(TARGETS icc_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
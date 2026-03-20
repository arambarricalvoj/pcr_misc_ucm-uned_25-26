cmake_minimum_required(VERSION 3.8)
project(tarea3)

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
add_library(tarea3_lib
    src/pose_controller.cpp
    src/formation_controller.cpp
    src/service_utils.cpp
    src/metrics_utils.cpp
)

ament_target_dependencies(tarea3_lib
    rclcpp
    rclcpp_action
    std_srvs
    geometry_msgs
    ament_index_cpp
    interfaces
)

# Ejecutable principal
add_executable(pose_controller src/main_pose_controller.cpp)
target_link_libraries(pose_controller tarea3_lib)

# Instalación
install(TARGETS pose_controller
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(formation_controller src/main_formation_controller.cpp)
target_link_libraries(formation_controller tarea3_lib)

# Instalación
install(TARGETS formation_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
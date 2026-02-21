#!/bin/bash
set -e

# Cargar entorno de ROS y arrancar CoppeliaSim
source /opt/ros/jazzy/setup.bash
/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/coppeliaSim.sh 

# Ejecutar el comando que se pase al contenedor
exec "$@"
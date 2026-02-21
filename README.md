# 1. Construir la imagen de Docker o descargarla 
Construir:
```bash
docker build -t coppelia_sim_ros2:jazzy .
```

Descargar:
```bash
docker pull arambarricalvoj/coppelia_sim_ros2:jazzy
```
<br><br>

# 2. Ejecutar imagen de Docker
```bash
sudo chmod u+x run.sh
./run.sh
```
Esto arranca automáticamente Coppelia dentro de Docker. Para acceder a otra terminal del contenedor e interactuar con ROS2:
```bash
docker exec -it coppelia_sim
```

En las terminales accedidas mediante ``docker exec`` es necesario ejecutar ``source /opt/ros/jazzy/setup.bash``.
<br><br>

# ROS2 Interface Plugin
En ``tareas/`` se encuentra tanto la escena probada como el script Python de Coppelia que permite obtener la Pose de un robot móvil en movimiento y publicarlo al tópico ``/robot_pose`` de ROS2 mediante el mensaje ``geometry_msgs/msg/Pose``.

El plugin de ROS2 por defecto solo admite los mensajes descritos en ``CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/programming/ros2_packages/sim_ros2_interface/meta/interfaces.txt``. Para añadir otro revisar la documentación: [https://github.com/CoppeliaRobotics/simROS2](https://github.com/CoppeliaRobotics/simROS2)

# Referencias
[https://www.coppeliarobotics.com/#](https://www.coppeliarobotics.com/#)

[https://manual.coppeliarobotics.com/en/ros2Tutorial.htm](https://manual.coppeliarobotics.com/en/ros2Tutorial.htm)

[https://manual.coppeliarobotics.com/en/rosInterf.htm](https://manual.coppeliarobotics.com/en/rosInterf.htm)

[https://github.com/CoppeliaRobotics/simROS2](https://github.com/CoppeliaRobotics/simROS2)

[https://www.youtube.com/watch?v=v3IyN6lRG5A](https://www.youtube.com/watch?v=v3IyN6lRG5A)

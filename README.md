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
<br><br>

# Uso de ROS2 en Lua o Python
En CoppeliaSim, los servicios ROS2 escritos en Lua suelen ser más estables que sus equivalentes en Python debido a cómo se integran ambos lenguajes con el motor interno. Lua es el lenguaje nativo de CoppeliaSim: sus scripts se ejecutan dentro del scheduler interno del simulador, y los callbacks —incluidos los de servicios ROS 2— se atienden de forma inmediata y sin intermediarios. Esto garantiza que los servicios creados en Lua respondan siempre, incluso bajo carga o cuando se realizan múltiples llamadas consecutivas.

Python, en cambio, funciona como un intérprete externo conectado mediante un puente IPC. Cada llamada ROS 2 --> CoppeliaSim --> Python atraviesa varias capas adicionales, lo que introduce latencia y hace que el callback dependa del estado del intérprete en ese instante. Si el intérprete está ocupado o el scheduler no lo despierta a tiempo, la llamada puede perderse, generando respuestas intermitentes como success=False. Además, ciertas funciones como sim.step(), que son seguras en Lua, no lo son en Python y pueden bloquear el hilo.

Por estas razones, aunque Python es totalmente compatible con CoppeliaSim, Lua ofrece una ejecución más fiable para servicios ROS2, especialmente cuando se requiere que los callbacks respondan siempre y sin fallos, tal y como se ha observado no solo en la práctica, sino en varios foros por parte de numerosos desarrolladores.

Consecuentemente, se recomienda el uso de Lua para utilizar ROS2, y en la ruta ``tareas/simROS2-master/examples/`` están los ejemplos oficiales. Abrir cada una de las escenas y después clickar en los script dentro de CoppeliaSim.
<br><br>

# Ejecución del controller para la tarea 1
```bash
source /opt/ros/jazzy/setup.bash
cd /home/javierac/pcr_misc/tareas/ros2_ws/
colcon build
source /home/javierac/pcr_misc/tareas/ros2_ws/install/setup.bash
ros2 run tarea1 icc_controller --ros-args --params-file tarea1/config/params.yaml 

```

# Referencias
[https://www.coppeliarobotics.com/#](https://www.coppeliarobotics.com/#)

[https://manual.coppeliarobotics.com/en/ros2Tutorial.htm](https://manual.coppeliarobotics.com/en/ros2Tutorial.htm)

[https://manual.coppeliarobotics.com/en/rosInterf.htm](https://manual.coppeliarobotics.com/en/rosInterf.htm)

[https://forum.coppeliarobotics.com/viewtopic.php?t=10793](https://forum.coppeliarobotics.com/viewtopic.php?t=10793)

[https://forum.coppeliarobotics.com/viewtopic.php?t=10758](https://forum.coppeliarobotics.com/viewtopic.php?t=10758)

[https://forum.coppeliarobotics.com/viewtopic.php?t=10790](https://forum.coppeliarobotics.com/viewtopic.php?t=10790)

[https://github.com/CoppeliaRobotics/simROS2](https://github.com/CoppeliaRobotics/simROS2)

[https://www.youtube.com/watch?v=v3IyN6lRG5A](https://www.youtube.com/watch?v=v3IyN6lRG5A)

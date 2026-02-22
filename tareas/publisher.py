def sysCall_init():
    global sim, simROS2, robot, pub

    sim = require('sim')
    simROS2 = require('simROS2')

    # Obtener el objeto al que está asociado este script
    robot = sim.getObject('.')

    sim.addLog(sim.verbosity_scriptinfos,
               f"Python script iniciado. Handle del robot: {robot}")


def sysCall_thread():
    global pub

    sim.addLog(sim.verbosity_scriptinfos, "Thread iniciado (publisher)")

    # Crear publicador ROS2 para geometry_msgs/msg/Pose
    pub = simROS2.createPublisher(
        '/robot_pose',
        'geometry_msgs/msg/Pose'
    )

    sim.addLog(sim.verbosity_scriptinfos,
               "ROS2 publisher creado en /robot_pose (Pose)")

    # Bucle de publicación (igual que el ejemplo oficial)
    while True:
        # Obtener posición y orientación absolutas
        pos = sim.getObjectPosition(robot, -1)
        quat = sim.getObjectQuaternion(robot, -1)

        # Crear mensaje Pose
        msg = {
            'position': {
                'x': pos[0],
                'y': pos[1],
                'z': pos[2]
            },
            'orientation': {
                'x': quat[0],
                'y': quat[1],
                'z': quat[2],
                'w': quat[3]
            }
        }

        # Publicar
        simROS2.publish(pub, msg)

        # Esperar un poco (como en el ejemplo oficial)
        sim.wait(0.05, False)


def sysCall_cleanup():
    simROS2.shutdownPublisher(pub)
    sim.addLog(sim.verbosity_scriptinfos, "Publisher apagado")

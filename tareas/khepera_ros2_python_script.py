def sysCall_init():
    global sim, simROS2, robot, pub

    sim = require('sim')
    simROS2 = require('simROS2')

    # Obtener el objeto al que está asociado este script
    robot = sim.getObject('.')

    sim.addLog(sim.verbosity_scriptinfos,
               f"Python script iniciado. Handle del robot: {robot}")

    # Crear publicador ROS2 para geometry_msgs/msg/Pose
    pub = simROS2.createPublisher(
        '/robot_pose',
        'geometry_msgs/msg/Pose'
    )

    sim.addLog(sim.verbosity_scriptinfos,
               "ROS2 publisher creado en /robot_pose (Pose)")


def sysCall_actuation():
    # Obtener posición y orientación absolutas
    pos = sim.getObjectPosition(robot, -1)
    quat = sim.getObjectQuaternion(robot, -1)

    # Imprimir en consola
    sim.addLog(sim.verbosity_scriptinfos,
               "Pose pos: {:.3f}, {:.3f}, {:.3f} | quat: {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
                   pos[0], pos[1], pos[2],
                   quat[0], quat[1], quat[2], quat[3]
               ))

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


def sysCall_sensing():
    pass


def sysCall_cleanup():
    sim.addLog(sim.verbosity_scriptinfos, "Script finalizado")

function sysCall_init()
    sim = require'sim'
    simROS2 = require'simROS2'

    -- Obtener el objeto al que está asociado este script
    robot = sim.getObject('.')

    sim.addLog(sim.verbosity_scriptinfos,
        string.format("Lua script iniciado. Handle del robot: %d", robot))
end


function sysCall_thread()
    sim.addLog(sim.verbosity_scriptinfos, "Thread iniciado (publisher)")

    -- Crear publicador ROS2 para geometry_msgs/msg/Pose
    pub = simROS2.createPublisher(
        '/robot_pose',
        'geometry_msgs/msg/Pose'
    )

    sim.addLog(sim.verbosity_scriptinfos,
        "ROS2 publisher creado en /robot_pose (Pose)")

    -- Bucle de publicación (igual que el ejemplo oficial)
    while true do
        -- Obtener posición y orientación absolutas
        local pos = sim.getObjectPosition(robot, -1)
        local quat = sim.getObjectQuaternion(robot, -1)

        -- Crear mensaje Pose
        local msg = {
            position = {
                x = pos[1],
                y = pos[2],
                z = pos[3]
            },
            orientation = {
                x = quat[1],
                y = quat[2],
                z = quat[3],
                w = quat[4]
            }
        }

        -- Publicar
        simROS2.publish(pub, msg)

        -- Esperar un poco (como en el ejemplo oficial)
        sim.wait(0.05, false)
    end
end


function sysCall_cleanup()
    simROS2.shutdownPublisher(pub)
    sim.addLog(sim.verbosity_scriptinfos, "Publisher apagado")
end

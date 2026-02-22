function sysCall_init()
    sim = require'sim'
    simROS2 = require'simROS2'

    -- Obtener el objeto TargetPosition
    target = sim.getObject('/TargetPosition')
end

function srv_callback(req)
    sim.addLog(sim.verbosity_msgs, 'service called')

    -- Obtener posición y orientación del target
    local pos = sim.getObjectPosition(target, -1)
    local quat = sim.getObjectQuaternion(target, -1)

    -- Crear respuesta Trigger
    local resp = simROS2.createInterface('std_srvs/srv/TriggerResponse')
    resp.success = true
    resp.message = string.format(
        '{"position":{"x":%f,"y":%f,"z":%f},"orientation":{"x":%f,"y":%f,"z":%f,"w":%f}}',
        pos[1], pos[2], pos[3],
        quat[1], quat[2], quat[3], quat[4]
    )

    return resp
end

function sysCall_thread()
    sim.addLog(sim.verbosity_msgs, 'thread started')

    -- Crear servicio dentro del thread (como el ejemplo oficial)
    srv = simROS2.createService(
        '/get_target_pose',
        'std_srvs/srv/Trigger',
        srv_callback
    )

    -- Mantener vivo el hilo
    while true do
        sim.step()
    end
end

function sysCall_cleanup()
    simROS2.shutdownService(srv)
    sim.addLog(sim.verbosity_msgs, 'thread finished')
end

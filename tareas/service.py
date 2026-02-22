import json

def sysCall_init():
    global sim, simROS2, target
    sim = require('sim')
    simROS2 = require('simROS2')

    # Objeto TargetPosition
    target = sim.getObject('/TargetPosition')


def getTargetPose_callback(req):
    sim.addLog(sim.verbosity_scriptinfos, "service called")

    # Obtener posición y orientación del target
    pos = sim.getObjectPosition(target, -1)
    quat = sim.getObjectQuaternion(target, -1)

    # Crear respuesta Trigger
    resp = simROS2.createInterface('std_srvs/srv/TriggerResponse')
    resp['success'] = True
    resp['message'] = json.dumps({
        'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
        'orientation': {'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]}
    })

    return resp


def sysCall_thread():
    global srv

    sim.addLog(sim.verbosity_scriptinfos, "thread started")

    # Crear servicio dentro del thread (igual que el ejemplo Lua)
    srv = simROS2.createService(
        '/get_target_pose',
        'std_srvs/srv/Trigger',
        getTargetPose_callback
    )

    # Mantener vivo el hilo (equivalente a sim.step() en Lua)
    while True:
        sim.wait(0.01)   # NO usar sim.step() en Python


def sysCall_cleanup():
    simROS2.shutdownService(srv)
    sim.addLog(sim.verbosity_scriptinfos, "thread finished")

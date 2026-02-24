function sysCall_init()
    sim = require'sim'
    simROS2 = require'simROS2'

    -- Obtener los motores del Khepera IV 
    rightMotor=sim.getObjectHandle('K4_Right_Motor')
    leftMotor=sim.getObjectHandle('K4_Left_Motor')
    
    sim.setJointTargetVelocity(leftMotor, 0) 
    sim.setJointTargetVelocity(rightMotor, 0)

    -- Parámetros físicos reales del Khepera IV
    wheelRadius = 0.021   -- metros (21 mm)
    wheelBase   = 0.088   -- metros (88 mm)

    -- Crear subscriber a /cmd_vel (Twist)
    sub = simROS2.createSubscription(
        '/cmd_vel',
        'geometry_msgs/msg/Twist',
        cmdVel_callback
    )

    sim.addLog(sim.verbosity_scriptinfos, "Subscriber /cmd_vel creado")
end


-- CALLBACK: recibe Twist y lo convierte a velocidades de rueda
function cmdVel_callback(msg)
    local v = msg.linear.x
    local w = msg.angular.z

    -- Cinemática diferencial
    local vLeft  = (v - w * wheelBase/2) / wheelRadius
    local vRight = (v + w * wheelBase/2) / wheelRadius

    -- Enviar velocidades a los motores
    sim.setJointTargetVelocity(leftMotor,  vLeft)
    sim.setJointTargetVelocity(rightMotor, vRight)
end


function sysCall_cleanup()
    simROS2.shutdownSubscription(sub)
    sim.addLog(sim.verbosity_scriptinfos, "Subscriber /cmd_vel apagado")
end

function sysCall_init()
    sim = require 'sim'
    simROS2 = require 'simROS2'

    -- Obtener handles de los 8 sensores IR
    ir = {}
    for i=1,8 do
        ir[i] = sim.getObjectHandle('K4_Infrared_'..i)
    end

    -- Crear 8 publicadores Range
    pubs = {}
    for i=1,8 do
        local topic = '/khepera/ir'..i
        pubs[i] = simROS2.createPublisher(topic, 'sensor_msgs/msg/Range')
        sim.addLog(sim.verbosity_scriptinfos, "Publisher creado: "..topic)
    end
end


function sysCall_sensing()
    for i=1,8 do
        local detected, distance = sim.readProximitySensor(ir[i])
        --sim.addLog(sim.verbosity_scriptinfos, "IR: "..i)
        --sim.addLog(sim.verbosity_scriptinfos, "Detected: "..detected)

        if detected == 0 then
            distance = 0.250   -- sin detección ? distancia máxima
        end

        local msg = {
            radiation_type = 1,     -- INFRARED
            field_of_view = 0.26,   -- ~15 grados
            min_range = 0.002,
            max_range = 0.250,
            range = distance
        }

        simROS2.publish(pubs[i], msg)
    end
end


function sysCall_cleanup()
    for i=1,8 do
        simROS2.shutdownPublisher(pubs[i])
    end
end

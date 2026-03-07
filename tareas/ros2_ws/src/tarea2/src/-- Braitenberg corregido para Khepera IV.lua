-- Braitenberg corregido para Khepera IV (8 IR: K4_Infrared_1..8).
-- Ahora los sensores izquierdos hacen girar el robot a la derecha (evitan hacia afuera).
sim = require 'sim'

function sysCall_init()
    bodyElements = sim.getObjectHandle('Khepera_IV')
    leftMotor    = sim.getObjectHandle('K4_Left_Motor')
    rightMotor   = sim.getObjectHandle('K4_Right_Motor')

    -- sensores IR (1..8)
    ir = {}
    for i = 1, 8 do
        ir[i] = sim.getObjectHandle('K4_Infrared_'..i)
        if not ir[i] or ir[i] == -1 then
            sim.addStatusbarMessage("No se encontró: K4_Infrared_"..i)
        end
    end

    -- parámetros de normalización
    min_range = 0.002
    max_range = 0.250
    maxDetectionDist = 0.02

    detect = {0,0,0,0,0,0,0,0}

    -- PESOS CORREGIDOS (acoplamiento cruzado):
    -- sensores 1..4 (izquierda) afectan a la rueda derecha (negativo -> frenan derecha)
    -- sensores 5..8 (derecha) afectan a la rueda izquierda (negativo -> frenan izquierda)
    -- Ajusta magnitudes si quieres más/menos reacción.
    braitenbergL = { 0.0,  0.0,  0.0,  0.0,  -0.4, -0.6, -0.8, -1.0}
    braitenbergR = {-1.0, -0.8, -0.6, -0.4,   0.0,  0.0,  0.0,  0.0}

    -- velocidad base y límite
    v0 = 0.5
    vmax = 1.0
end

function sysCall_cleanup()
end

local function clamp(x, a, b)
    if x < a then return a end
    if x > b then return b end
    return x
end

function sysCall_actuation()
    -- leer y normalizar sensores
    for i = 1, 8 do
        local handle = ir[i]
        if handle and handle ~= -1 then
            local detected, distance = sim.readProximitySensor(handle)
            if detected == 0 then distance = max_range end

            if distance < max_range then
                local d = distance
                if d < maxDetectionDist then d = maxDetectionDist end
                local dnorm = 1 - ((d - maxDetectionDist) / (max_range - maxDetectionDist))
                if dnorm < 0 then dnorm = 0 end
                if dnorm > 1 then dnorm = 1 end
                detect[i] = dnorm
            else
                detect[i] = 0
            end
        else
            detect[i] = 0
        end
    end

    -- calcular velocidades con Braitenberg (acoplamiento cruzado)
    local vLeft = v0
    local vRight = v0
    for i = 1, 8 do
        vLeft  = vLeft  + braitenbergL[i] * detect[i]
        vRight = vRight + braitenbergR[i] * detect[i]
    end

    -- limitar y aplicar
    vLeft  = clamp(vLeft, -vmax, vmax)
    vRight = clamp(vRight, -vmax, vmax)

    if leftMotor and rightMotor then
        sim.setJointTargetVelocity(leftMotor, vLeft)
        sim.setJointTargetVelocity(rightMotor, vRight)
    else
        sim.addStatusbarMessage("Motores no inicializados correctamente.")
    end
end

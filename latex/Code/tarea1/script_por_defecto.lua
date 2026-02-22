------------Llamando a los elementos------------------    
        
    bodyElements=sim.getObjectHandle('Khepera_IV')
    rightMotor=sim.getObjectHandle('K4_Right_Motor')
    leftMotor=sim.getObjectHandle('K4_Left_Motor')

    ------------------------------------------------------

    -------Separando a Objetivo del Robot Khepera IV------

    ------------------------------------------------------ 

    ----------Calling camera----------------------

    frontCam=sim.getObjectHandle('K4_Camera')
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(frontView,frontCam,64)

    ------------------------------------------------------

     while (true) do 

        rv=5
        lv=3     
        
        sim.setJointTargetVelocity(leftMotor,lv) --Cm/s a m/s
        sim.setJointTargetVelocity(rightMotor,rv)
     

end

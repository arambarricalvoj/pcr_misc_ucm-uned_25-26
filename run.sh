xhost +local:*
docker run -e DISPLAY=$DISPLAY \
           -e USER=$USER \
           -e QT_X11_NO_MITSHM=1 \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v ./bibliografia/:/home/$USER/pcr_misc/bibliografia \
           -v ./robot_description/:/home/$USER/pcr_misc/robot_description \
           -v ./tareas/:/home/$USER/pcr_misc/tareas \
           -it \
           --rm \
           --gpus all \
           --runtime=nvidia \
           --name 2coppelia_sim \
           coppelia_sim_ros2:jazzy
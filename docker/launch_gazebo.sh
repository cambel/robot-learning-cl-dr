# Simpler way to create a temporary Docker container that will be remove once stopped
# After executing this file, use the command `docker exec -it trufus bash` to connect to the container.
# The container will be removed if the PC is rebooted.
docker stop trufus
docker create -it \
    --gpus all \
    --runtime=nvidia --rm -it \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env QT_X11_NO_MITSHM=1 \
    --ipc host \
    --privileged \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/cambel/trufus/ros-ur:/root/ros_ws/src/trufus \
    -v /home/cambel/trufus/pylibs:/root/pylibs \
    -v /home/cambel/trufus/gps:/root/gps \
    -v ~/dev:/root/dev \
    -v /dev:/dev \
    -v /var/run/spnav.sock:/var/run/spnav.sock \
    --network host \
    -w '/root/dev/' \
    --name=trufus \
    trufus:melodic-gazebo \
    && export containerId=$(docker ps -l -q) \
    && xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId` \
    && docker start $containerId

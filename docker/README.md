# Docker tutorial

To build this docker execute the following command in this directory

```
docker build -t ros-ur3 .
```

Then, start the docker container 
1. be sure to create the following directories
```
mrkdir ~/dev/container_ws
mrkdir ~/dev/container_pylibs
```

2. Execute the script
```
bash launch_docker.sh
```

3. To access the running container from any new terminal, execute:
```
docker exec -it ros-ur3 bash
```
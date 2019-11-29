### Ros Docker
docker build -t drone-sim .

### if not done define network
docker network create foo

### run the core
```
docker run -it --rm \
    --net foo \
    --name master \
    -v "/home/alexander/Schreibtisch/DroneSimulation/catkin_ws":/usr/src/catkin_ws \
    drone-sim \
    roscore
```

### connect to docker shell
```
docker exec -it master bash
source /ros_entrypoint.sh
rosrun gazebo_ros gzserver
```


### start Gazebo Client
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.Networks.foo.IPAddress }}' master)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
gzclient --verbose
```

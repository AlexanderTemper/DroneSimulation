### Ros Docker
```
docker build -t drone-sim .
```

### if not done define network
```
docker network create foo
```

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
source /ros_entrypoint.sh && cd usr/src/catkin_ws/
catkin_make
source devel/setup.bash


roslaunch spd_simulation world.launch

rosrun gazebo_ros spawn_model -file `rospack find spd_simulation`/models/test_drone/model.sdf -sdf -x 0 -y 0 -z 1 -model MYROBOT

rosrun gazebo_ros gzserver
```


### start Gazebo Client
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.Networks.foo.IPAddress }}' master)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
export GAZEBO_MODEL_PATH=$(pwd)/volumen/models
sudo chown -R alexander:alexander $GAZEBO_MODEL_PATH
gzclient --verbose

```

### PX4
```
Terminal 1: 
docker run -it --rm \
    --net foo \
    --name master \
    -v "$(pwd)/volumen/":/tmp/models/ \
    drone-sim-px4
source /ros_entrypoint.sh && cd usr/src/catkin_ws/ && catkin build && source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

Terminal 2:
docker exec -it master bash
cd /usr/src/px4src/Firmware
no_sim=1 make px4_sitl_default gazebo

Terminal 3: 
docker exec -it master bash
source /ros_entrypoint.sh && cd usr/src/catkin_ws/ && source devel/setup.bash
cd /usr/src/px4src/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
cp -r /usr/src/px4src/Firmware/Tools/sitl_gazebo/models /tmp/models
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world


```

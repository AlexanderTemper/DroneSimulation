## Ros Docker
```
docker build -t drone-sim .
```
## define network
```
docker network create droneSimNetwork
```
## get px4
```
cd px4src
git clone https://github.com/PX4/Firmware.git && cd Firmware && git checkout v1.9.2 && git submodule update --init --recursive
```
## run simmulation
### terminal 1: px4 macros Server: 
docker run -it --rm \
    --net droneSimNetwork \
    --name master \
    -v "$(pwd)/catkin_ws/src/spd_simulation":/usr/src/catkin_ws/src/spd_simulation \
    -v "$(pwd)/px4src":/usr/src/px4src \
    drone-sim
source /ros_entrypoint.sh && cd usr/src/catkin_ws/ && catkin build && source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
### terminal 2: px4 sitl
docker exec -it master bash
cd /usr/src/px4src/Firmware
no_sim=1 make px4_sitl_default gazebo
```

### terminal 3: gazebo ros
docker exec -it master bash
source /ros_entrypoint.sh && cd usr/src/catkin_ws/ && source devel/setup.bash
cd /usr/src/px4src/Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```

### terminal 4: gazebo client
```
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.Networks.droneSimNetwork.IPAddress }}' master)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
export GAZEBO_MODEL_PATH=$(pwd)/px4src/Firmware/Tools/sitl_gazebo/models
gzclient --verbose

```


# Install CrazyS
Folow instruction under https://github.com/gsilano/CrazyS/tree/master



## Build simulation
```
catkin init
catkin build
catkin build
```
## launch simulation
```
source devel/setup.bash 
roslaunch src/gateway/launch/world.launch
rosrun joy joy_node
rosrun baseflight_controller baseflight_controller_node
rosrun gateway gateway_node
```
# Simulation

![](https://raw.githubusercontent.com/AlexanderTemper/DroneSimulation/master/simulation.gif)

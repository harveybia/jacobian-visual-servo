# jacobian-visual-servo
This is a class project for CMU 16-711 Kinematics, Dynamics and Control,
Spring 2020.

## Getting Started

### Dependencies

1. Ubuntu 18.04
1. ROS Melodic
1. CoppeliaSim v4.0.0+
([Install CoppeliaSim](https://www.coppeliarobotics.com/downloads))
1. CoppeliaSim ROS 1.0 Plugin
(Copy compiledRosPlugins/libsimExtROSInterface.so out to installation dir)
1. Apriltag: `rosdep install --from-paths src --ignore-src -r -y`

### Optional Dependencies

1. Vortex Studio (for better dynamics simulation)

### Running The Demo
#### Setting up ROS workspace and build the project
```sh
mkdir -p ~/16711_ws/src
cd ~/16711_ws/src
git clone https://github.com/harveybia/jacobian-visual-servo
catkin build
source ~/16711_ws/src/jacobian-visual-servo/devel/setup.bash
```
#### Starting CoppeliaSim robot simulation
```sh
roscore & # Run roscore however you like

cd <CoppeliaSim_Installation_Dir>
./coppeliaSim.sh # Run after roscore is running

# Open snake_arm.ttt and tap play button
```

#### Running our control script
```sh
rosrun jacobian-visual-servo servo_node
```

## Authors

- Haowen (Harvey) Shi (haowensh@andrew.cmu.edu)
- Daqian Cheng (daqianc@andrew.cmu.edu)
- Hengrui (Henry) Zhang (hengruiz@andrew.cmu.edu)

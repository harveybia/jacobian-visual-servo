# Visual Servoing for Flexible Robot Manipulators
This is a class project for CMU 16-711 Kinematics, Dynamics and Control,
Spring 2020.

Demo Video:

[![demo_video](http://img.youtube.com/vi/B46CvOQX2VM/0.jpg)](http://www.youtube.com/watch?v=B46CvOQX2VM "Visual servoing for deformable robot arms")

[Link to Project Report](https://github.com/harveybia/jacobian-visual-servo/blob/master/docs/16_711_Project_Report.pdf)

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

# Open snake_arm.simscene.xml and tap play button
```

#### Running our control script
```sh
rosrun jacobian-visual-servo uncertain_IK_server
```

#### Running April Tag localization
1. april tag node
```sh
roslaunch apriltag_ros continuous_detection.launch
```

2. camera info publisher node
```
roslaunch apriltag_utils caminfo_pub.launch
```

## Authors

- Haowen (Harvey) Shi (haowensh@andrew.cmu.edu)
- Daqian Cheng (daqianc@andrew.cmu.edu)
- Hengrui (Henry) Zhang (hengruiz@andrew.cmu.edu)

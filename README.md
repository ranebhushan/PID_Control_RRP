# Joint Space PID Control of Robot Manipulators in ROS

Course Project for RBE 500 - Foundations of Robotics at Worcester Polytechnic Institute

Group Members:-
- Bhushan Ashok Rane
- Yash Rajendra Patil

## Description

### Dependencies

Install the ROS dependencies by the following command:

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

To run this project, you also need to add the following repository in the `/src` of your workspace.

[rrp_robot](https://github.com/ranebhushan/rrp_robot.git){:target=``_blank``}

### Usage Guidelines

Launch the RRP robot manipulator in Gazebo using by the following command:
```
roslaunch rrp_gazebo gazebo.launch
```

Once the robot is successfully spawned in Gazebo, open a new terminal and launch the effort controller node and the joint state publisher by using the following command:
```
roslaunch rrp_control rrp_effort_control.launch
```

Now, test the inverse kinematics and position control scripts by running the `rrp.launch` file in another terminal using:
```
roslaunch rbe500_project rrp.launch
```

Meanwhile all these processes are running, open a new terminal to observe current position, velocities and forces/torques of each robot joint by running a rostopic as follows:

```
rostopic echo /rrp/joint_states
```
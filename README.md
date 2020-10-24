
# As a summary:
 Currently, there are four packages in my github repositories:
## [Barry-Liang/universal_robot](https://github.com/Barry-Liang/universal_robot):
Forked from [ros-industrial's official package](https://github.com/ros-industrial/universal_robot), I have fixed  some bugs in the ur5_e_moveit_config to make it avilable. Also, the urdf files in this packages can be used to generate ikfast packages. Please pay attention that there are many branches, if you are stick in ros-melodic, you should try  

```bash
$ git clone -b melodic-devel https://github.com/Barry-Liang/universal_robot.git
```


## [Barry-Liang/my_universal_robot](https://github.com/Barry-Liang/my_universal_robot): 
Copied from [fmauch's package](https://github.com/fmauch/universal_robot) last year, I have fixed  some bugs in the ur5_e_moveit_config to make it avilable. However, the urdf files in this packages can not be used to generate ikfast packages.
-----

## [Barry-Liang/Universal_Robots_Ros_Driver](https://github.com/Barry-Liang/Universal_Robots_ROS_Driver): 
Forked from the [UniversalRobots' s offical driver package](github.com/UniversalRobots/Universal_Robots_ROS_Driver).

However, it will make error when combined with the official **universial_robot** package from ros-industrial.

```bash
 error: ‘ur_msgs::SetPayload::Request {aka struct ur_msgs::SetPayloadRequest_<std::allocator<void> >}’ has no member named ‘center_of_gravity’
             << " set_payload(" << req.payload << ", [" << req.center_of_gravity.x << ", " << req.center_of_gravity.y
                                                    
```


It will make other error when it is built with [Barry-Liang/my_universal_robot](https://github.com/Barry-Liang/my_universal_robot).

According to the original readme, until now, it should be built with fmauch's calibration-devel.

```bash
# clone fork of the description. This is currently necessary, until the changes are merged upstream.
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
```


## [Barry-Liang/my_Universal_Robots_Ros_Driver](https://github.com/Barry-Liang/my_Universal_Robots_Ros_Driver): 

Copied from a early version of [UniversalRobots' s offical driver package](github.com/UniversalRobots/Universal_Robots), which can be combined-built with the offical universal robots package---> [Barry-Liang/universal_robot](https://github.com/Barry-Liang/universal_robot).



------



# Universal Robot

[![Build Status](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__universal_robot__ubuntu_xenial_amd64)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial](http://wiki.ros.org/Industrial) Universal Robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.


__Installation__

There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.


___Using apt (Ubuntu, Debian)___

On supported Linux distributions (Ubuntu, up to 16.04 (Xenial), `i386` and `amd64`) and ROS versions:

```
sudo apt-get install ros-$ROS_DISTRO-universal-robot
```

replace `$ROS_DISTRO` with `hydro`, `indigo` or `kinetic`, depending on which ROS version you have installed.


___Building from Source___

There are releases available for ROS Hydro, Indigo and Kinetic. However, for the latest features and developments you might want to build the packages from source.

**NOTE**: please prefer using the binary release (see previous section) over building from source where possible. Source installs will not be automatically updated by new package releases and require more work to setup.

The following instructions assume that a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created at `$HOME/catkin_ws` and that the source space is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

In all other cases the packages will have to be build from sources in a Catkin workspace: 

```
cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```


__Usage__

___With real Hardware___
There are launch files available to bringup a real robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the real robot, run:

```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```


CAUTION:  
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.


___MoveIt! with real Hardware___  
Additionally, you can use MoveIt! to control the robot.  
There exist MoveIt! configuration packages for both robots.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_bringup ur5_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


___Usage with Gazebo Simulation___  
There are launch files available to bringup a simulated robot - either UR5 or UR10.  
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!   

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur5.launch```


___MoveIt! with a simulated robot___  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


NOTE:  
As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:  

```roslaunch ur_gazebo ur5.launch limited:=true```

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true```

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```



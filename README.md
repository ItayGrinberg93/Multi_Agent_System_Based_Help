# multi_agent_system_based_help
two robots, cyton_300_gamma and turtlebot3, achive a common goal 
<!--pic of simulation-->

<p align="center">
<b><i>Cyton_300_gamma and Turtlebot3,need to Build pile of cudes</i></b>
</p>

<p align="center">
<img src="figure/intro-1/gzebo_intro.png" alt="" width="51%">
</p>


<p align="center">
<b>Itay Grinberg & Benjamin Barns</b>
<br>
<a href="mailto:itaygrinberg@campus.technion.ac.il" target="_top">itaygrinberg@campus.technion.ac.il</a>
</p>
<p align="center">
<a href="mailto:barnes@campus.technion.ac.il" target="_top">benjib@campus.technion.ac.il</a>
</p>

------------

<a id="top"></a>
### Contents
1. [Introduction](#1.0)
2. [Environment Setup](#2.0)
3. [Theoretical Background](#3.0)
4. [Design Requirements](#4.0)
5. [Design Implementation](#5.0)
7. [Testing and Review](#6.0)

------------

### Abbreviations

* **DOF** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Degrees Of Freedom](https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics))
* **ROS** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Robot Operating System](http://www.ros.org/)
* **ARC** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Amazon Robotics Challenge](https://www.amazonrobotics.com/#/roboticschallenge)
* **EE** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [End-Effector](https://en.wikipedia.org/wiki/Robot_end_effector)
* **WC** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Wrist Center](https://www.youtube.com/watch?v=V_6diIcQl0U)
* **DH** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Denavit–Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
* **FK** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Forward Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics)
* **IK** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Inverse Kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics)
* **RRR** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Revolute Revolute Revolute](http://www.roboticsbible.com/robot-links-and-joints.html)
* **URDF** &nbsp;&nbsp;&nbsp; [Unified Robot Description Format](http://wiki.ros.org/urdf)
* **TB3** &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

------------

<a name="1.0"></a>
### 1. Introduction
The proposed project focuses on heterogeneous teams of autonomous robots.This project originated from Cyton 300 gamma [Robotic arm - Pick & Place project](http://new.robai.com/assets/Cyton-Gamma-300-Arm-Specifications_2014.pdf)

<p align="center">
<img src="figure/intro-1/cyton_300.png" alt="" width="53%">
<br>
<sup><b>Fig. 1.1&nbsp;&nbsp;A robotic arm </b></sup>
</p>

which get assistnce from [Turtlebot3 - Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)  .
<p align="center">
<img src="figure/intro-1/turtlebot-3.png" alt="" width="53%">
<br>
<sup><b>Fig. 1.2&nbsp;&nbsp;A Turtlebot3 - Burger </b></sup>
</p>



##### Objective
In order to collaborate effectively, AI agents must be able to reason about the behaviors of other agents, learn to communicate effectively, learn to ask for help from other agents and understand how they can offer valuable assistance to other agents. . 

The objective is to find methods that allow the robots to compute behaviors that maximize their `helpfulness' to other robots while complying with their own resources and objectives.

This is a challenging problem since it requires an online analysis of the current setting and the current capabilities of each agent, an online computation of an optimal policy for each agent, and a way for the agents to monitor and coordinate their progress.In addition, it requires an integration of high-level task planning and low-level motion planning in a multi-agent setting.  

Within the context of this project, a single *task* cycle can be divided into the following tasks:

* Identify the target object on the area
* Plan and perform a clean movement towards the object
* Efficiently grasp/pick the target object without disturbing other objects
* Plan and perform a clean movement towards the drop-off site
* Efficiently stow/place the object at the drop-off site
* If possible back to 1. Else ask for help and countine.
* Plan and perform movement to the object the first agent can't reach
* attach the object
* Plan and perform a clean movement towards the pick-up point
* detach the object
* movefrom the area


##### Relevance
Collaborative ability can be beneficial in several ways: Performing complex tasks that can't be done by one agent alone. Additionally, information sharing should occur and maximize the utilization of each agent's abilities, whether together or individually. Furthermore, when dealing with an environment involving people, we can define roles for the robot and the person.

<p align="center">
<img src="figures/1-intro/relevance.png" alt="" width="73%">
<br>
<sup><b>Fig. 1.3&nbsp;&nbsp;relevant pic</b></sup>
</p>

Robotic manipulators and Differential Drive Robot like Tb3 have become ubiquitous in almost every industry; from food, beverage, shipping and packaging to manufacturing, foundry and space:

* Palletizing food in a bakery
* Serving food
* Precision painting of automobiles and aircrafts
* Targeting products to their packaging stations in a warehouse

These jobs all require the same set of core capabilities, namely, the robotic arm's end-effector being able to reach specific coordinates within its workspace, and the TB3 can navigate to a point in the environment at the location. 

------------
<a name="2.0"></a>
<!--<div style="text-align:left;">
  <span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;"> 2. Environment Setup</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 2. Environment Setup
The project uses [ROS Melodic Morenia](http://wiki.ros.org/melodic) running on [Ubuntu 18.04 LTS ( bionic beaver)](https://releases.ubuntu.com/18.04/).

The following tools are used for simulation and motion planning:
* [Gazebo](http://gazebosim.org/): a physics based 3D simulator extensively used in the robotics world
* [RViz](http://wiki.ros.org/rviz): a 3D visualizer for sensor data analysis, and robot state visualization
* [MoveIt!](http://moveit.ros.org/): a ROS based software framework for motion planning, kinematics and robot control

Once ROS is installed, we can proceed with the environment setup for the project:

##### Verify Project Tools

1\. Chack that you have tje following dependencies and if not run the following commands:
```sh
$ sudo apt-get install ros-melodic-ros-controllers
```
```sh
$ sudo apt-get install ros-melodic-gazebo*
```
```sh
$ sudo apt-get install ros-melodic-moveit*
```
```sh
$ sudo apt-get install ros-melodic-industrial-core
```
```sh
$ sudo apt-get install ros-melodic-trac-ik-kinematics-plugin
```
```sh
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

```
##### Create ROS Workspace
2\. Create a [catkin](http://wiki.ros.org/catkin/conceptual_overview) workspace if haven't already
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_init_workspace
$ ls -l
```

3\. Clone or download project repository into the *src* directory of the catkin workspace
```sh
cd ~/catkin_ws/src
$ git clone https://github.com/andreasBihlmaier/gazebo2rviz.git
$ git clone https://github.com/andreasBihlmaier/pysdf.git
$ git clone https://github.com/JenniferBuehler/general-message-pkgs.git
$ git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
$ git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
$ git clone https://github.com/ItayGrinberg93/multi_agent.git
```
4\. Build the project
```sh
$ cd ~/catkin_ws
$ catkin_make
```
#### Run the simulation
5\. To use TB3 add to bash.rc or open two terminals and run in each: 
```sh
$ source devel/setup.bash
$ export TURTLEBOT3_MODEL=burger 
```
6\. At the first terminal run:
```sh
$ roslaunch new_moveit_config multi_gazebo_update.launch 
```
7\. At the secound terminal run:
```sh
$ roslaunch new_moveit_config bringup_update.launch
 
```







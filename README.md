# Multi Agent System Based Help
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
<a href="mailto:barnes@campus.technion.ac.il" target="_top">barnes@campus.technion.ac.il</a>
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
#### python code
8\. Open a new terminal and run:

```sh
$ cd ~/catkin_ws
$ source devel/setup.bash
$ cd src/py_moveit/scripts
$ chmod +x python_sim.py
$ rosrun py_moveit python_sim.py
```

------------

<a name="3.0"></a>
<!--<div style="text-align:left;">
<span style="font-size: 1.4em; margin-top: 0.83em; margin-bottom: 0.83em; margin-left: 0; margin-right: 0; font-weight: bold;">3. Theoretical Background</span><span style="float:right;"><a href="#top">Back to Top</a></span>
</div>-->
### 3. Theoretical Background
The following theoretical concepts are used in this project:



* Generalized Coordinates and Degrees of Freedom
* Common industrial serial manipulators and their workspace
* Rotation matrices and composition of rotations
* Euler angles and Euler theorem
* Euler to Quaternions

* Homogeneous transforms

#### For the manipulator

* Denavit–Hartenberg parameters
* Forward and Inverse Kinematics

#### For the TB3

* Generalized Coordinates and Degrees of Freedom


#### 3.1 Serial Manipulators
[Serial manipulators](https://en.wikipedia.org/wiki/Serial_manipulator) are robots composed of an assembly of links connected by joints (a [Kinematic Chain](https://en.wikipedia.org/wiki/Kinematic_chain)), and the most common types of robots in industry.

##### Generalized Coordinates
Generalized coordinates are parameters that are used to uniquely describe the instantaneous dynamical configuration of a [rigid](https://en.wikipedia.org/wiki/Rigid_body) [multi-body system](https://en.wikipedia.org/wiki/Multibody_system) relative to some reference configuration. In the robotics of serial manipulators, they are used to define the *configuration space* or *joint space*, which refers to the set of all possible configurations a manipulator may have.

##### Degrees of Freedom
The [degree of freedom (DOF)](https://en.wikipedia.org/wiki/Degrees_of_freedom_(mechanics)) of a rigid body or mechanical system is the number of independent parameters or coordinates that fully define its configuration in free space.

Common DOFs:

* *6*: coordinates required to fully describe the configuration of a rigid body in 3D free space
* *12*: coordinates required to fully describe simultaneously the configuration of two separate rigid bodies in 3D free space
* *7*: coordinates required to fully describe the configuration of two rigid bodies in 3D free space connected by a joint

<p align="center">
<img src="figure/3-theory/workspace_RRR.jpg" alt="" width="73%">
<br>
<sup><b>Fig. 3.1&nbsp;&nbsp;Geometry of a 3-DOF anthropomorphic robot</b></sup>
<br>
<sup>[Source: Narong Aphiratsakun. AIT]</sup>
</p>

The serial manipulator shown in figure 3.1  has n=3 joints: each a [revolute](https://en.wikipedia.org/wiki/Revolute_joint) with 1-DOF. Each joint connects with two links, making the total number of links, n+1 = 4, including the fixed base link.

Therefore, the total number of DOF for any serial manipulator with *three* 1-DOF joints is:

&nbsp;<img src="figures/3-theory/codecogseqn3.gif" alt="" width="5%">

*Note:* The DOF of a serial manipulator with only [revolute](https://en.wikipedia.org/wiki/Revolute_joint) and/or [prismatic](https://en.wikipedia.org/wiki/Prismatic_joint) joints is *always* equal to the number of its joints, except when both ends of the manipulator are fixed (closed chain linkage).

##### Workspace
The *workspace* of a robotic manipulator is defined as the set of points that can be reached by its [end-effector](https://en.wikipedia.org/wiki/Robot_end_effector) <sup>[2]</sup>. In other words, it is simply the 3D space in which the robot mechanism works.

<p align="center">
<img src="figures/3-theory/scara_anthro_wksp.png" alt="" width="65%">
<br>
<sup><b>Fig. 3.2&nbsp;&nbsp;&nbsp;Robot Reach “Bubble”</b></sup>
<br>
<sup>[Source: Federica.EU]</sup>
</p>

Figure 3.2 shows two types of serial manipulators, [SCARA](https://en.wikipedia.org/wiki/SCARA) and [Anthropomorphic](https://en.wikipedia.org/wiki/SCARA) with their associated workspaces. Figure 3.1 also shows the workspace of the 3-DOF manipulator from a top and side perspective. 

It is important to note that no kinematic solution exists for the manipulator's configuration or joint space for any desired end-effector position outside of the workspace.

#### 3.2 Rotation of Coordinate Frames
Rotation matrices are a means of *expressing* a vector in one coordinate frame in terms of some other coordinate frame.

<p align="center">
<img src="figures/3-theory/rot_derivation_ab_3.png" alt="" width="68%">
<br>
<sup><b>Fig. 3.3&nbsp;&nbsp;A 2D geometric rotation between coordinate frames A and B</b></sup>
<br>
</p>

In figure 3.2, Point **P** is expressed with vector **u** relative to coordinate *frame B*. The objective is to express point **P** with vector **v** relative to coordinate *frame A*. The basis vectors of **v**, **v<sub>x</sub>** and **v<sub>y</sub>** can be expressed in terms of the basis vectors of **u**, **u<sub>x</sub>** and **u<sub>y</sub>**  as follows:

<p align="center">
<img src="figures/3-theory/rot_deriv_1.png" alt="" width="32%">
</p>

where unit vectors of *frame A*, **a<sub>x</sub>** and **a<sub>y</sub>** are expressed in terms of unit vectors of *frame B*, **b<sub>x</sub>** and **b<sub>y</sub>** as follows:

<p align="center">
<img src="figures/3-theory/rot_deriv_2.png" alt="" width="32%">
</p>

Substituting (2) in (1) and solving for the dot products yields the following equation:

<p align="center">
<img src="figures/3-theory/rot_deriv_3.png" alt="" width="32%">
</p>

where the first term on the right-hand side is the **2D Rotation Matrix**, denoted in this case as **<i><sup>a</sup><sub>b</sub>R</i>**. Any point on coordinate *frame B* multiplied by <i><sup>a</sup><sub>b</sub>R</i> will project it onto *frame A*. In other words, to express a vector **u** on some *frame B* as a vector **v** on a different *frame A*, **u** is multiplied by the rotation matrix with angle theta by which *frame A* is rotated from *fram B*. Also worth noting is that the rotation from A to B is equal to the *transpose* of the rotation of B to A.


#### 3.3 Euler Angles
Euler angles are a system to describe a sequence or a composition of rotations. According to [Euler's Rotation Theorem](https://en.wikipedia.org/wiki/Euler%27s_rotation_theorem), the orientation of any [rigid body](https://en.wikipedia.org/wiki/Rigid_body) w.r.t. some fixed reference frame can always be described by **three** elementary rotations in a given **sequence** as shown in figure 3.3.

<p align="center">
<img src="figures/3-theory/Inertial-Frame.png" alt="" width="52%">
<br>
<sup><b>Fig. 3.4&nbsp;&nbsp;Defining Euler angles from a sequence of rotations</b></sup>
<br>
<sup>[Source: CHRobotics]</sup>
</p>

Conventionally, the movements about the three axes of rotations and their associated angles are described by the 3D rotation matrices in figure 3.3.

<p align="center">
<img src="figures/3-theory/euler_rotation_matrices_c.png" alt="" width="85%">
<br>
<sup><b>Fig. 3.5&nbsp;&nbsp;3D counter-clockwise rotation matrices describing yaw, pitch and roll</b></sup>
</p>

Euler angles are characterized by the following properties:

* Tait-Bryan vs. Classic
* Rotation Order
* Intrinsic (body fixed) vs. Extrinsic (fixed axes) rotations

**Intrinsic** or body-fixed rotations are performed about the coordinate system *as* rotated by the previous rotation. The rotation sequence changes the axis orientation after each elemental rotation while the body remains fixed.

<p align="center">
<img src="figure/3-theory/in_rot_matrices.png" alt="" width="62%">
</p>

In an intrinsic sequence of rotations, such as, a Z-Y-X convention of a yaw, followed by a pitch, followed by a roll, subsequent elemental rotations are *post-multiplied*.

**Extrinsic** or fixed-axis rotations are performed about the *fixed* world reference frame. The original coordinate frame remains motionless while the body changes orientation.

<p align="center">
<img src="figure/3-theory/ex_rot_matrices.png" alt="" width="62%">
</p>

In an extrinsic sequence of rotations, such as, a Z-Y-X convention of a yaw, followed by a pitch, followed by a roll, subsequent elemental rotations are *pre-multiplied*.

**Note:** An extrinsic rotation sequence of A, B, C = an intrinsic rotation sequence of C, B, A.

Euler angles, normally in the [Tait–Bryan](https://commons.wikimedia.org/wiki/Tait-Bryan_angles), **Z-X-Y** convention, are also used in robotics for describing the degrees of freedom of a spherical wrist of a robotic manipulator.

Of particular importance is a phenomenon associated with Euler angles known as a [Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock) which occurs when there is a loss of one degree of freedom as a result of the axes of two of the three [gimbals](https://en.wikipedia.org/wiki/Gimbal) driven into a parrallel configuration.

#### 3.4 Quaternions 
Quaternions are an algebraic structure that extends the familiar concept of complex numbers.A quaternion is a 4-tuple written formally as q0 + q1*i + q2*j + q3*k, where qi are real numbers and the symbols i, j, k sustain an orthogonal system. While quaternions are much less intuitive than angles, rotations defined by quaternions can be computed more efficiently
and with more stability, and therefore are widely used.

**From Euler angles to a quaternion** The formula seems to have been derived this way:

First, roll around the world x
axis. The quaternion for this is qx,α=cosα2/+(sinα/2)i.

Second, pitch around the world y
axis. The quaternion is qy,β=cosβ/2+(sinβ/2)j.

Third, yaw around the world z
axis. The quaternion is  qz,γ=cosγ/2+(sinγ/2)k..

A rotation that is done in steps like this is modeled by multiplying the quaternions.

#### 3.5 Homogeneous Transforms
In the case where a reference frame is both simultaneously rotated *and* translated (transformed) with respect to some other reference frame, a *homogeneous transform matrix* describes the transformation.

<p align="center">
<img src="figure/3-theory/homo_tf_2.png" alt="" width="44%">
<br>
<sup><b>Fig. 3.6&nbsp;&nbsp;Rotation and Translation of frame B relative to frame A</b></sup>
<br>
<sup>[Source: Salman Hashmi. BSD License]</sup>
</p>

In figure 3.7, point P is expressed w.r.t. frame B and the objective is to express it w.r.t. frame A. To do so would require projecting or superimposing frame B onto frame A i.e. first rotating frame B to orient it with frame A and then translating it such that the centers B<sub>0</sub> and A<sub>0</sub> of both frames are aligned.

<p align="center">
<img src="figure/3-theory/homog_transform_eqns.png" alt="" width="52%">
</p>

The relationship between the three vectors in figure 3.7 is shown in equation (1). The desired vector to point P from A<sub>0</sub> is the sum of the vector to point P from B<sub>0</sub>, rotated to frame A, and the translation vector to B<sub>0</sub> w.r.t A<sub>0</sub>. Equations (2) and (3) are the matrix-forms of equation (1) so that it can be rendered in software with linear algebra libraries.

<p align="center">
<img src="figure/3-theory/homog_tf_2_exp.png" alt="" width="55%">
<br>
<sup><b>Fig. 3.7&nbsp;&nbsp;Anatomy of the homogeneous transform relationship</b></sup>
</p>

Figure 3.8 describes the components of equation (2). The desired vector to point P (w.r.t. to A<sub>0</sub>) is obtained by multiplying the given vector to point P (w.r.t. B<sub>0</sub>) by the *homogeneous transform* matrix, composed of the block Rotation matrix projecting B onto A and the block translation vector to B w.r.t A<sub>0</sub>.

<p align="center">
<img src="figure/3-theory/robo_arm_w.png" alt="" width="43%">
<br>
<sup><b>Fig. 3.8&nbsp;&nbsp;Transformation between adjacent revolute joint frames</b></sup>
</p>

As shown in figure 3.8, the position of the end-effector is known w.r.t. *its* coordinate reference frame *C*. The objective is to express it w.r.t. the *fixed* world coordinate reference frame *W*. This is because the positions of all objects of interest in the manipulator's environment are expressed w.r.t. the world reference frame. In other worlds, both, the end-effector, *and* the objects it interacts with need to be defined on the *same* coordinate reference frame. 

Point P relative to frame *W* can be found by successively applying equation (4) between adjacent joints:

<p align="center">
<img src="figure/3-theory/robo_arm_sol_complete_2.png" alt="" width="84%">
</p>

The above process can be summarized in terms of equation (1) with *<sup>W</sup><sub>C</sub>T* being the desired composite homogeneous transform that projects *frame C* onto *frame W*.

<p align="center">
<img src="figure/3-theory/robo_arm_sol_summary_3.png" alt="" width="21%">
</p>

#### 3.6 Denavit–Hartenberg parameters
Before the homogeneous transforms between adjacent links can be computed, the coordinate frames of the joint links on which the transforms are applied must be defined. The [Denavit–Hartenberg (DH) parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters) are four parameters describing the rotations and translations between adjacent links. The definition of these parameters constitutes a convention for assigning coordinate reference frames to the links of a robotic manipulator. Figure 3.8 shows the so-called *modified* convention of DH parameters as defined by \[Craig, JJ. (2005)].

<p align="center">
<img src="figure/3-theory/mod_dh_params_labeled_4.png" alt="" width="65%">
<br>
<sup><b>Fig. 3.7&nbsp;&nbsp;The four parameters of the Modified DH convention</b></sup>
<br>
<sup>[Source: Modified from Wikipedia Commons]</sup>
</p>

The parameters are defined as follows:

* α<sub>i-1</sub>: twist angle between the z-axes of links *i-1* and *i* (measured about *x<sub>i-1</sub>* in a right-hand sense)
* ɑ<sub>i-1</sub>: link distance between the z-axes of links *i-1* and *i*  (measured *x<sub>i-1</sub>*)
* d<sub>i</sub>: link offset signed distance between the x-axes of links  *i-1* and *i*  (measured along *z<sub>i</sub>*)
* θ<sub>i</sub>: joint angle between the x-axes of links  *i-1* and *i* (measured about *z<sub>i</sub>* in a right-hand sense)

Note:

* The origin of a frame *i* is defined by the intersection of *x<sub>i</sub>* and *z<sub>i</sub>*
* The x-axes define the common normals between *z<sub>i-1</sub>* and *z<sub>i</sub>*

Recall that to compute the position of the end-effector w.r.t. the base or world reference frame, transforms between adjacent links are composed as follows:

<p align="center">
<img src="figures/3-theory/dh_eq_1.png" alt="" width="55%">
</p>

where the base frame is denoted by *0* and the end-effector's frame denoted by *N*. Thus,  <sup>0</sup><sub>N</sub>T defines the homogeneous transformation that projects frame *N* onto frame *0*. More specifically, a single transform between links *i-1* and *i* is given by

<p align="center">
<img src="figures/3-theory/dh_eq_2.png" alt="" width="55%">
</p>

<p align="center">
<img src="figures/3-theory/dh_eq_3_v2.png" alt="" width="55%">
</p>

and is made up up of two rotations *R* of magnitudes α and θ, and two displacements *D* of magnitudes ɑ and d.

The parameter assignment process for open kinematic chains with n degrees of freedom (i.e., joints) is summarized as:

1. Label all joints from {1, 2, … , n}.
2. Label all links from {0, 1, …, n} starting with the fixed base link as 0.
3. Draw lines through all joints, defining the joint axes.
4. Assign the Z-axis of each frame to point along its joint axis.
5. Identify the common normal between each frame *Z*<sub>i-1</sub> and  *Z*<sub>i</sub>
6. The endpoints of *intermediate links* (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the *X*<sub>i</sub> to be ...
	1. For skew axes, along the normal between *Z*<sub>i</sub> and *Z*<sub>i+1</sub> and pointing from {i} to {i+1}.
	2. For intersecting axes, normal to the plane containing *Z*<sub>i</sub> and *Z*<sub>i+1</sub>.
	3. For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.
7. For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable (θ<sub>1</sub> 
​​  or d<sub>1</sub>) is equal to zero. This will guarantee that α<sub>0</sub> = a<sub>0</sub> = 0, and, if joint 1 is a revolute, d<sub>1</sub> = 0. If joint 1 is prismatic, then θ<sub>1</sub> = 0.
8. For the end effector frame, if joint n is revolute, choose *X*<sub>n</sub> to be in the direction of *X*<sub>n−1</sub>
​​  when θ<sub>n</sub>​ = 0 and the origin of frame {n} such that d<sub>n</sub> = 0.

Special cases involving the *Z*<sub>i-1</sub> and *Z*<sub>i</sub> axes:

* collinear lines: alpha = 0 and a = 0
* parallel lines: alpha = 0 and a ≠ 0
* intersecting lines: alpha ≠ 0 and a = 0
* If the common normal intersects *Z*<sub>i</sub> at the origin of frame i, then d​<sub>i</sub> is zero.

Once the frame assignments are made, the DH parameters are typically presented in tabular form (below). Each row in the table corresponds to the homogeneous transform from frame {i} to frame {i+1}.

<p align="center">
<img src="figures/3-theory/mod_dh_table_2.png" alt="" width="58%">
<br>
<sup><b>Table 3.1&nbsp;&nbsp;The four parameters of the Modified DH convention</b></sup>
</p>

#### 3.7 Forward and Inverse Kinematics
Forward Kinematics is the process of computing a manipulator's end-effector position in Cartesian coordinates from its given joint angles. This can be achieved by a composition of homogeneous transformations that map the base frame onto the end-effector's frame, taking as input the joint angles. The end-effector's coordinates can then be extracted from the resulting composite transform matrix.

The relationship between Forward and Inverse Kinematics is depicted in figure 3.9,

<p align="center">
<img src="figures/3-theory/fk_ik_3.png" alt="" width="43%">
<br>
<sup><b>Fig. 3.9&nbsp;&nbsp;Relationship between Forward and Inverse Kinematics</b></sup>
</p>

Inverse Kinematics is the reverse process where the EE position is known and a set of joint angles that would result in that position need to be determined. This is a more complicated process than FK as multiple solutions can exist for the same EE position. However, no joint angle solutions exist for any EE position outside the manipulator's workspace. There are two main approaches to solve the IK problem: numerical and analytical. The later approach is used in this project.

#### 3.8 Differential Drive Robot

The robot we use is a two-wheeled (motorized) robot. Each of the wheels
can rotate back and forth separately.
Let’s define the dynamical model:

* (x,y) - the position of the robot
* φ - the orientation of the robot
* v- measured linear velocity
* ω - measured angular velocity
let's write the derivatives of the position and the orientation,

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{x}=v\cos(\phi)" title="\Large \dot{x}=v\cos(\phi)" />
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{y}=v\sin(\phi)" title="\Large \dot{y}=v\sin(\phi)" />
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{\phi}=\omega " title="\Large \dot{\phi}=\omega " />
</p>

In the robot we control at, their only the velocity of the two wheels
Let’s define it in our model:

* R - the radios of the wheel
* L - the distance between the wheels
* vr - right wheel velocity
* vl - left wheel velocity
so,

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{x}=\frac{R}{2}(v_r+v_l)\cos(\phi)" title="\Large \dot{x}=\frac{R}{2}(v_r+v_l)\cos(\phi)" />
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{y}=\frac{R}{2}(v_r+v_l)\sin(\phi)" title="\Large \dot{y}=\frac{R}{2}(v_r+v_l)\sin(\phi)" />
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\dot{\phi}=\frac{R}{L}(v_r-v_l) " title="\Large \dot{\phi}=\frac{R}{L}(v_r-v_l) " />
</p>

If we compare coefficients 
 we can express the measured linear <img src="https://latex.codecogs.com/svg.latex?\small&space;(\dot{x},\dot{y},\dot{\phi}) " title="\small (\dot{x},\dot{y},\dot{\phi}) " />
and angular velocities depending on <img src="https://latex.codecogs.com/svg.latex?\small&space;(v_r,v_l) " title="\small (v_r,v_l) " />:

so,

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;v=\frac{R}{2}(v_r+v_l)\cos(\phi)" title="\Large  v=\frac{R}{2}(v_r+v_l)\cos(\phi)" />
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space;\omega=\frac{R}{L}(v_r-v_l) " title="\Large \omega=\frac{R}{L}(v_r-v_l)" />
</p>

hence,


<p align="center">
<img src="https://latex.codecogs.com/svg.latex?\Large&space; $$v_r=\frac{2v+\omega L}{2R}$$" title="\Large $$v_r=\frac{2v+\omega L}{2R}$$" />
</p>

<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=v_l=\frac{2v-\omega L}{2R}">
</p>

#### 3.9 Robot Operating System (ROS)

##### ROS_MASTER:
* manager of the ROS *Nodes*
* allows _Nodes to locate one another and communicate
* also hosts the _Parameter_Server so running _Nodes can lookup parameter and configuration values e.g. wheel radius
* More Explanation:
* maintains a registry of all the active nodes on a system. It then allows these nodes to discover other nodes to establish lines of communications between nodes i.e. the nodes themselves don't have to communicate to other nodes through the ROS master; they can do it directly once the ROS master has allowed them to discover other relavant nodes for communication.

##### Communication between Nodes:
- Through _Topics or _Services
- Pub-Sub comm pattern vs, Req-Res comm pattern
- Pub-Sub comm pattern:
	- _Nodes can also share data (i.e. publish and subsrcribe _Messages) 
	  amongst themselves via nameds buses called _Topics. 
	  (Recall that the only stuff that nodes and ROS master share 
	  are Parameter values)
	- Nodes publish and subscribe _Messages over _Topics
- Req-Res comm architecture:
	- Like _Topics, _Services allow the passing of message 
	  betwee nodes

##### Environment Setup:
- Before we can launch and use a ROS package like turtlesim, 
  we must first ensure that all of the ROS environment variables 
  have been correctly set.
- Your ROS distro e.g. Kinetic provides a bash script called
  setup.bash to ensure this.
- This script can be run with the bash command: source
- Among other things, the ROS environment variables tell our 
  bash shell where ROS commands and packages can be found.

##### roscore
- rosout node is responsible for aggregating, filtering and
  recording log messages to a text file.

##### catkin
- a powerful build and package management system provided
  by ROS.
- a catkin workspace is a direcoty where catkin packages
  are built, modified and installed
- all ROS software components are organized into and 
  distributed as catkin packages
- catkin packages contain resources like source code for
  nodes, useful scripts, config file etc.

##### roslaunch
- Launch ROS Master and multiple nodes with one simple command
- Set default parameters on the parameter server
- Automatically re-spawn processes that have died

##### Rosplan





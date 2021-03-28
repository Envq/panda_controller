# **Panda Controller**
This is a set of utilities and examples for control Panda Franka Emika using ROS and c++.



## **Table of Contents**
* [Getting started](#getting-started)
    * [FAQ](#faq)
    * [Dependencies](#dependencies)
    * [Building from source](#building-from-source)
* [Files](#files)
    * [data_manager](#data_manager)
    * [panda](#panda)
    * [exceptions](#exceptions)
    * [colors](#colors)
* [Data](#data)
* [Examples](#examples)
    * [console](#console)
    * [pick_place](#pick_place)
    * [teleop](#teleop)
* [Visual Studio Code](#vscode)
* [Author](#author)
* [License](#license)



---
## **Getting Started**
This package was tested:
- Without real robot and:
    - ROS melodic
    - Linux Mint Tricia (compatible with Ubuntu Bionic)
    - franka_ros f7f00b6d9678e59e6f34ccc8d7aad6491b42ac80
    - panda_moveit_config 5c97a61e9e8a02ca7f1fe08df48ac4ff1b03871a

- With real robot and:
    - ROS kinetic
    - Ubuntu Xenial
    - franka_ros 0.6.0
    - (panda_moveit_config included in franka_ros)

### **- FAQ**
- The 'simulation' vscode task not control real panda, it is only used for test project.
- The 'controller' vscode task control real panda, but it call another file unavailable. So edit it with your launch file.
- Gripper use franka_gripper action server that is available only with real panda arm.
- If you not have real panda use the kinetic-devel branches of franka_ros and panda_moveit_config repositories, else follow the bottom instructions.



### **- Dependencies**
**NOTE:** If you want to use the real robot replace the word "melodic" with "kinetic".

- Install ROS:
http://wiki.ros.org/melodic/Installation/Ubuntu


- Remove possible conflicts: 
~~~
sudo apt remove ros-melodic-franka* ros-melodic-panda* *libfranka*
~~~


- Update system:
~~~
rosdep update

sudo apt-get update

sudo apt-get dist-upgrade
~~~


- Install c++ tools:
~~~
sudo apt install clang-format cloc doxygen
sudo apt install python-pip
pip3 install --user lizard
~~~


- Install catkin the ROS build system and cmake:
~~~
sudo apt install cmake ros-melodic-catkin python-catkin-tools
~~~


- Install moveit:
~~~
sudo apt install ros-melodic-moveit
~~~


- Install libfranka:
~~~
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

mkdir -p ~/github

cd ~/github

git clone https://github.com/frankaemika/libfranka -b master

mkdir -p ~/github/libfranka/build

cd ~/github/libfranka/build

cmake -DCMAKE_BUILD_TYPE=Release ..

cmake --build .
~~~



### **- Building from source**
**Create Workspace:**
~~~
mkdir -p ~/panda_ws/src

cd ~/panda_ws/src

git clone https://github.com/Envq/panda_controller.git

git clone https://github.com/frankaemika/franka_ros.git
~~~

- If you work WITH real robot:
~~~
cd ~/panda_ws/src/franka_ros

git checkout 0.6.0
~~~

- If you work WITHOUT real robot:
~~~
git clone https://github.com/ros-planning/panda_moveit_config.git

cd ~/panda_ws/src/franka_ros

git checkout f7f00b6d9678e59e6f34ccc8d7aad6491b42ac80

cd ~/panda_ws/src/panda_moveit_config

git checkout 5c97a61e9e8a02ca7f1fe08df48ac4ff1b03871a
~~~

Add to franka_ros/franka_description/robots/panda_arm.xacro:
~~~
<link name="${arm_id}_gripper_center"/>
<joint name="${arm_id}_gripper_center_joint" type="fixed">
    <parent link="${arm_id}_link8"/>
    <child link="${arm_id}_gripper_center"/>
    <origin xyz="0.0 0.0 0.1035" rpy="0 ${pi} -${pi/4}"/>
    <axis xyz="0 0 0"/>
</joint>
~~~

Replace to franka_ros/franka_description/mesh/collision/finger.stl with panda_controller/data/finger.stl for better collisions


**Configure Workspace**
~~~
# If you use a different linux distribution based on bionic like Linux Mint
echo 'export ROS_OS_OVERRIDE=ubuntu:18.04:bionic' >> ~/.bashrc

# To fix a bug with move-it
echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc

echo 'source ~/panda_ws/devel/setup.bash' >> ~/.bashrc

source ~/.bashrc

cd ~/panda_ws/

rosdep install -y --from-paths src --ignore-src --rosdistro melodic --skip-keys libfranka

catkin config -j$(nproc) --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/github/libfranka/build

catkin init
~~~


### **- Build Workspace**
~~~
catkin build
~~~


---
## **Files:**

### **data_manager**
- This file contains all the functions to communicate with the stored data.

### **panda**
- This file is a wrapper for easy use of moveit to control the Panda robot.

### **exceptions**
- All exceptions used are defined in this file.

### **colors**
- This file contains the definitions to modify the colors and format of the stdout. Wrap your output string with them.



---
## **Data**
This folder contains:
- poses.json: 
    - **Description**: A Collection of position and orientation of a reference link.
    - **Formant**: "name_pose" : {eef, orientation, position}.
    - **Doc**: Look [here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) for more informations on **Pose**.
- scenes.json:
    - **Description**: A collection of Scene.
    - **Formant1**: "name_scene" : {name, color, position, orientation, type, dimensions}.
    - **Formant2**: "name_scene" : {name, color, position, orientation, type=mesh, file, scale}. 
        - Specify the format (.stl) in the "file" field.
        - Check the position of the origin on cad programs.
        - Use the scale field if the file is exported to a unit other than meters.
    - **Doc**:
        - Click [here](http://docs.ros.org/melodic/api/shape_msgs/html/msg/SolidPrimitive.html) for more informations on **SolidPrimite**.
        - Click [here](http://docs.ros.org/kinetic/api/std_msgs/html/msg/ColorRGBA.html) for more informations on **ColorRGBA**.

---
## **Examples**
A brief description of the example files available:

Note: all launch files contain the "gripper_is_active" parameter which, if set to false, disables the gripper_action_client allowing to work without the real robot
![simulation](screenshot/simulation.png?raw=true "simulation")


### **console**
This node is a console for performing these simple tasks:
- **quit:** to close the node.
- **help:** to see the list of commands.
- **info:** to print robot info.
- **scene [name:str]:** to load specified scene.
- **scene reset:** to reset scene.
- **speed arm [speed:float]:** to set the arm speed value.
- **eef set [name:str]:** to set end effector link name.
- **eef get:** to get end effector link name.
- **speed gripper [speed:float]:** to set the gripper speed value.
- **save [(name:str)]:** to save the current pose with the specified name.
- **move joint [joint:int, val:double]:** to move the specified joint by an integer of the specified degree.
- **move offset [x:double y:double z:double]:** to move the arm along the x,y,z specified directions in meters.
- **move pose [name:str]:** to move the arm on the specified pose saved in database.
- **move gripper [width:double]:** to move the gripper fingers with the specified speed on the specified width from center.
- **homing arm:** to perform the homing of the arm.
- **homing gripper:** to perform the homing of the gripper.
- **grasp [width:double (force:double epsilon_inner:double epsilon_outer:double)]:** to perform the grasping.

**Leggends:**
- [] indicates a parameter.
- () indicates a optional parameter with default.

![console](screenshot/console.png?raw=true "console")



 ### **pick_place**

This node execute the "pick and place" task. Two objects are moved in this task.
In the launch file you can modify the following parameters:
- **[ scene:=string ]:** to specify the name of scene to load. (Is always loaded also "base" scene)
- **[ object1:=string ]:** to specify the name of object1 to pick.
- **[ pick_pose1:=string ]:** to specify the name of the pick pose 1.
- **[ place_pose1:=string ]:** to specify the name of the place pose 1.
- **[ object2:=string ]:** to specify the name of object2 to pick.
- **[ pick_pose2:=string ]:** to specify the name of the pick pose 2.
- **[ place_pose2:=string ]:** to specify the name of the place pose 2.
- **[ arm_speed:=float ]:** to specify the factor scale of the arm speed.
- **[ gripper_speed:=float ]:** to specify the gripper speed.
Values for grasp:
- **[ grasp_width:=float ]:** to specify the width of the grasp.
- **[ grasp_force:=float ]:** to specify the force of the grasp.
- **[ grasp_epsilon_inner:=float ]:** to specify the epsilon inner of the grasp.
- **[ grasp_epsilon_outer:=float ]:** to specify the epsilon outer of the grasp.

![pick_place](screenshot/pick_place.png?raw=true "pick_place")


### **teleop**

The "teleop_listener" node read from the topic "teleop" a vector message for moving the arm in cartesian mode.
The "teleop_talker" node read stdin for sending a vector message on the topic "teleop".

For control "teleop_talker":
- **GENERIC COMMANDS:**
    - **QUIT:** ESC
    - **HELP:** 'h'
    - **MODE:** 'm' There are 3 modes: position, orientation, gripper
- **POSITIONAL COMMANDS:**
    -  **X_POS:** 'w'
    -  **X_NEG:** 's'
    -  **Y_POS:** 'a'
    -  **Y_NEG:** 'd'
    -  **Z_POS:** 'i'
    -  **Z_NEG:** 'k'
    -  **INCREASE_POSITION:** 'l'
    -  **DECREASE_POSITION:** 'j'
- **ORIENTATION COMMANDS:**
    -  **ROLL_POS:** 'd'
    -  **ROLL_NEG:** 'a'
    -  **PITCH_POS:** 'w'
    -  **PITCH_NEG:** 's'
    -  **YAW_POS:** 'q'
    -  **YAW_NEG:** 'e'
    -  **INCREASE_ORIENTATION:** 'l'
    -  **DECREASE_ORIENTATION:** 'j'
- **GRIPPER COMMANDS:**
    -  **GRIPPER_HOMING:** 'o'
    -  **GRIPPER_GRASP:** 's'
    -  **GRIPPER_WIDTH_POS:** 'd'
    -  **GRIPPER_WIDTH_NEG:** 'a'
    -  **INCREASE_GRIPPER_WIDTH:** 'l'
    -  **DECREASE_GRIPPER_WIDTH:** 'j'
- **HOMING:**
    -  **const int GRIPPER_HOMING:** 'o';
    -  **const int ARM_HOMING:** 'o';

Behavior:
- Use Mode to switch mode.
- Use INCREASE and DESCREASE KEYS to change the value of DELTA (= displacement to be applied upon reception of the command).
- Use launch file for set the START_DELTA and the RESOLUTION (= the variation of the DELTA value).
- When your DELTA is the desired value, press the key to active the move.

![teleop](screenshot/teleop.png?raw=true "teleop")

---
## **VSCode**
I used these extensions:
- **c/c++** by microsoft
- **c/c++ snippets** by harsh
- **c++ intellisense** by austin
- **cmake** by twxs
- **doxygen** by christoph schlosser
- **clang-format** by xaver
- **doxgen documentation** by christoph schlosser
- **python** by microsoft
- **ros** by microsoft
- **urdf** by smilerobotics
- **git graph** by mhutchie
- **gruvbox mirror** by adamsome
- **vscode-icons** by icons for visual studio code
- **xml format** by Mike Burgh
- **git graph** by mhutchie


The following commands are available:
- **build** : builds this package.
- **buildAndRun**: builds this package and run the actual developing node.
- **simulation**: starts the node running RVIZ.
- **controller**: starts the node running robot controller. NOTE: update with your controller launch.
- **console**: starts the node that allows the Panda robot to perform simple tasks.
- **pick_place**: starts the node that perform pick-and-place task.
- **format**: formats the sources with clang-format.
- **doxygen**: builds the documentation.
- **cloc**: returns line of code.
- **lizard-cyclo**: returns Cyclomatic Complexity.
- **teleop_listener**: starts the node that reads teleop panda_controller/topic to perform robot moviment.
- **teleop_talker**: starts the node that converts keyboard input in teleop_panda msg.
- **build-all**: builds all packages.
- **clean-all**: cleans all packages.


---
## **Author**

**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq).


## **License**

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details.
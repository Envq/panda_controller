# **Panda Controller**
This is a set of utilities to control Panda Franka Emika using ROS and c++.



## **Table of Contents**
* [Getting started](#getting-started)
    * [FAQ](#faq)
    * [Dependencies](#dependencies)
    * [Building from source](#building-from-source)
* [Files](#files)
    * [data_manager](#data_manager)
    * [panda](#panda)
    * [excepions](#excepions)
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
This package was tested in:
- ROS melodic running under Linux Mint Tricia (compatible with Ubuntu Bionic).
- ROS kinetic running under Ubuntu Xenial.
- With real Panda robot.

### **- FAQ**
- The 'simulation' vscode task not control real panda, it is only used for test project.
- Gripper use franka_gripper action server that is available only with real panda arm.
- For simulation with the 'simulation' vscode task use the kinetic-devel branches of franka_ros and panda_moveit_config repositories, else follow the bottom instructions.



### **- Dependencies**
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
sudo apt install clang-format cmake
~~~


- Install catkin the ROS build system:
~~~
sudo apt install ros-melodic-catkin python-catkin-tools
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
**Create Workspace**
~~~
mkdir -p ~/workspace/panda_ws/src

cd ~/workspace/panda_ws/src

git clone https://github.com/Envq/panda_controller.git

git clone https://github.com/frankaemika/franka_ros.git

cd franka_ros

git checkout 0.6.0
~~~


**Configure Workspace**
~~~
# If you use a different linux distribution based on bionic like Linux Mint
echo 'export ROS_OS_OVERRIDE=ubuntu:18.04:bionic' >> ~/.bashrc

# To fix a bug with move-it
echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc

echo 'source ~/workspace/panda_ws/devel/setup.bash' >> ~/.bashrc

source ~/.bashrc

cd ~/workspace/panda_ws/

rosdep install -y --from-paths src --ignore-src --rosdistro melodic --skip-keys libfranka

catkin config -j$(nproc) --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/github/libfranka/build

catkin init
~~~


### **- Build Workspace**
~~~
catkin build
~~~


---
## **Files**

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
    - **Description**: A Collection of position and orientation of the link6 (before Panda's hand).
    - **Formant**: "name_pose" : {orientation, position}.
    - **Doc**: Look [here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) for more informations on **Pose**.
- scenes.json:
    - **Description**: A collection of Scene.
    - **Formant**: "name_scene" : {name, type, color, dimensions, position, orientation}.
    - **Doc**:
        - Click [here](http://docs.ros.org/melodic/api/shape_msgs/html/msg/SolidPrimitive.html) for more informations on **SolidPrimite**.
        - Click [here](http://docs.ros.org/kinetic/api/std_msgs/html/msg/ColorRGBA.html) for more informations on **ColorRGBA**.



---
## **Examples**
A brief description of the launch files available:


### **- simulation**
This node launches rviz with the panda arm (gripper action not work without real panda).

![simulation](screenshot/simulation.png?raw=true "simulation")


### **- console**
This node is a console for performing these simple tasks:
- **help:** to see the list of commands.
- **quit:** to close the node.
- **scene [name:str]:** to load specified scene.
- **scene reset:** to reset scene.
- **speed arm [speed:float]:** to set the arm speed value.
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



 ### **- pick_place**

This node execute the "pick and place" task with 3 methods.
- **[ method:=int ]:** to specify the method of pick-and-place (values: 1, 2, 3)
- **[ scene:=string ]:** to specify the name of scene to load.
- **[ object:=string ]:** to specify the name of object to pick.
- **[ pick_pose:=string ]:** to specify the name of the pick pose.
- **[ place_pose:=string ]:** to specify the name of the place pose.
- **[ arm_speed:=float ]:** to specify the factor scale of the arm speed.
- **[ gripper_speed:=float ]:** to specify the gripper speed.
Values for grasp:
- **[ grasp_width:=float ]:** to specify the width of the grasp.
- **[ grasp_force:=float ]:** to specify the force of the grasp.
- **[ grasp_epsilon_inner:=float ]:** to specify the epsilon inner of the grasp.
- **[ grasp_epsilon_outer:=float ]:** to specify the epsilon outer of the grasp.
Values for approch of method 2:
- **[ pre_grasp_approch_x:=float ]:** to specify the pre-grasp x.
- **[ pre_grasp_approch_y:=float ]:** to specify the pre-grasp y.
- **[ pre_grasp_approch_z:=float ]:** to specify the pre-grasp z.
- **[ post_grasp_retreat_x:=float ]:** to specify the post-grasp x.
- **[ post_grasp_retreat_y:=float ]:** to specify the post-grasp y.
- **[ post_grasp_retreat_z:=float ]:** to specify the post-grasp z.
- **[ post_place_retreat_x:=float ]:** to specify the post place x.
- **[ post_place_retreat_y:=float ]:** to specify the post place y.
- **[ post_place_retreat_z:=float ]:** to specify the post place z.

![pick_place](screenshot/pick_place.png?raw=true "pick_place")


### **- teleop**

The "teleop_listener" node read from the topic "teleop" a vector message for moving the arm in cartesian mode.
The "teleop_talker" node read stdin for sending a vector message on the topic "teleop"
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
    -  **GRIPPER_WIDTH_POS:** 'i'
    -  **GRIPPER_WIDTH_NEG:** 'k'
    -  **INCREASE_GRIPPER_WIDTH:** 'l'
    -  **DECREASE_GRIPPER_WIDTH:** 'j'


![teleop](screenshot/teleop.png?raw=true "teleop")

---
## **VSCode**
I used these extensions:
- **c/c++** by microsoft
- **c/c++ snippets** by harsh
- **c++ intellisense** by austin
- **cmake** by twxs
- **clang-format** by xaver
- **doxgen documentation** by christoph schlosser
- **python** by microsoft
- **ros** by microsoft
- **urdf** by smilerobotics
- **git graph** by mhutchie
- **gruvbox mirror** by adamsome
- **vscode-icons** by icons for visual studio code
- **xml format** by Mike Burgh


The following commands are available:
- **build** : builds this package
- **buildAndRun**: builds this package and run the actual developing node
- **simulation**: starts the node running RVIZ
- **console**: starts the node that allows the Panda robot to perform simple tasks
- **pick_place**: starts the node that perform pick-and-place task
- **format**: formats the sources with clang-format
- **build_all**: builds all packages
- **clean_all**: cleans all packages


---
## **Author**

**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq).


## **License**

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details.
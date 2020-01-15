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
- **load [scene]:** to load specified scene.
- **load reset:** to reset scene.
- **speed arm [value]:** to set the arm speed value.
- **speed gripper [value]:** to set the gripper speed value.
- **save [(name)]:** to save the current pose with the specified name.
- **move offset [x y z]:** to move the arm along the x,y,z specified directions in meters.
- **move pose [name]:** to move the arm on the specified pose saved in database.
- **move gripper [width (speed)]:** to move the gripper fingers with the specified speed on the specified width from center.
- **homing arm:** to perform the homing of the arm.
- **homing gripper:** to perform the homing of the gripper.
- **grasp [width (speed force epsilon_inner epsilon_outer)]:** to perform the grasping.

**Leggends:**
- [] indicates a parameter.
- () indicates a optional parameter with default.

![console](screenshot/console.png?raw=true "console")



 ### **- pick_place**

This node execute the "pick and place" task
- **[ scene:=string ]:** to specify the name of scene to load (default is "pick_place").
- **[ object:=string ]:** to specify the name of object to pick (default is "object").
- **[ pick_pose:=string ]:** to specify the name of the pick pose (default is "pick").
- **[ place_pose:=string ]:** to specify the name of the place pose (default is "place").
- **[ speed:=float ]:** to specify the factor scale of the arm speed (default is 1.0).

![pick_place](screenshot/pick_place.png?raw=true "pick_place")


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
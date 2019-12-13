# Panda Controller

This is a task for the the Panda arm (Franka Emika) using ROS



## Table of Contents

* [Getting started](#getting-started)
  * [Dependencies](#dependencies)
  * [Building from source](#building-from-source)
* [Database](#database)
* [Nodes](#nodes)
  * [controller](#controller)
  * [current_pose](#current_pose)
  * [relative_move](#relative_move)
  * [absolute_move](#absolute_move)
  * [gripper_move](#gripper_move)
  * [pick_place](#pick_place)
* [VSCode](#vscode)
* [Author](#author)
* [License](#license)



---
## Getting Started

This package was tested in:
- ROS melodic running under Linux Mint Tina (compatible with Ubuntu Bionic)
- ROS kinetic running under Ubuntu Xenial
- with real panda arm

### FAQ:
- controller vscode task not control real panda, it is used only for simulation
- gripper use franka_gripper action server that is available only with real panda arm
- for simulation with controller vscode task use kinetic-devel branches



## Dependencies:

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



## Building from source

### Create Workspace
~~~
mkdir -p ~/workspace/panda_ws/src

cd ~/workspace/panda_ws/src

git clone https://github.com/Envq/panda_controller.git

git clone https://github.com/frankaemika/franka_ros.git

cd franka_ros

git checkout 0.6.0
~~~


### Configure Workspace
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


## Build Workspace
~~~
catkin build
~~~


---
## Database
This folder contains:
- poses.json: 
    - A Collection of pose and orientation of the link6 (before Panda's hand)
    - "name_pose" : {orientation, position}
    - Look [here](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html) for more informations on **Pose**
- scenes.json:
    - A collection of Scene
    - "name_scene" : {name, type, color, dimensions, position, orientation}
    - Click [here](http://docs.ros.org/melodic/api/shape_msgs/html/msg/SolidPrimitive.html) for more informations on **SolidPrimite**
    - Click [here](http://docs.ros.org/kinetic/api/std_msgs/html/msg/ColorRGBA.html) for more informations on **ColorRGBA**

look 


---
## Nodes
A brief description of the launch files available:


 ### **controller**

This node launches rviz with the panda arm (gripper action not work without real panda)

![controller](screenshot/controller.png?raw=true "controller")


 ### **current_pose**

This node takes current pose of the panda arm and saves it on poses.json.
- **[ name:=string ]:** to specify the pose name to save (default is "current")

![current_pose](screenshot/current_pose.png?raw=true "current_pose")


### **relative_move**

This node moves the arm along the x, y and z directions
- **[ x:=float ]:** to specify the offset on x in meters (default is 0.0)
- **[ y:=float ]:** to specify the offset on y in meters (default is 0.0)
- **[ z:=float ]:** to specify the offset on z in meters (default is 0.0)
- **[ speed:=float ]:** to specify the factor scale of the arm speed (default is 1.0)
- **[ plan_only:=bool ]:** to specify if you want move the real arm or not (default is false)


### **absolute_move**

This node moves the arm on the selected pose
- **[ pose:=string ]:** to specify the name of pose
- **[ speed:=float ]:** to specify the factor scale of the arm speed (default is 1.0)
- **[ plan_only:=bool ]:** to specify if you want move the real arm or not (default is false)


### **gripper_move**

This node moves the gripper
- **[ mode:=string ]:** to specify the mode between *homing*, *move* and *grasp*
    - **mode:=home** execute the homing of gripper
    - **mode:=move** use args **width** and **speed** for moving the gripper
    - **mode:=grasp** use args **width**, **speed**, **force**, **epsilon_inner** and **epsilon_outer** for execute the grasp
- **[ width:=float ]:** to specify the width of the gripper movement in meters (default 0.08 which corresponds to the max opening)
- **[ speed:=float ]:** to specify the factor scale of the arm speed in m/s (default is 0.5)
- **[ force:=float ]:** to specify the force in netwon of the grasp (default is 20.0)
- **[ epsilon_inner:=float ]:** to specify the epsilon inner of the grasp (default is 0.002)
- **[ epsilon_outer:=float ]:** to specify the epsilon outer of the grasp (default is 0.002)


 ### **pick_place**

This node execute the "pick and place" task
- **[ scene:=string ]:** to specify the name of scene to load (default is "pick_place")
- **[ object:=string ]:** to specify the name of object to pick (default is "object")
- **[ pick_pose:=string ]:** to specify the name of the pick pose (default is "pick")
- **[ place_pose:=string ]:** to specify the name of the place pose (default is "place")
- **[ speed:=float ]:** to specify the factor scale of the arm speed (default is 1.0)


---
## VSCode
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
- **build** : build this package
- **buildAndRun**: build this package and run the actual developing node
- **controller**: run the node that launch rviz
- **current_pose**: run the node that save current pose
- **relative_move**: run the node that execute relative move
- **absolute_move**: run the node execute abolute move
- **relative_move**: run the node execute gripper move
- **pick_place**: run the node that execute pick-and-place task
- **format**: format sources with clang-format
- **build_all**: build all packages
- **clean_all**: clean all packages


---
## Author

**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq)


## License

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details
# Panda Controller

This is a task for the the Panda arm (Franka Emika) using ROS



## Table of Contents

* [Getting started](#getting-started)
  * [Dependencies](#dependencies)
  * [Building from source](#building-from-source)
* [Nodes](#nodes)
  * [simulation](#simulation)
* [VSCode](#vscode)
* [Author](#author)
* [License](#license)



---
## Getting Started

This package was written for ROS melodic running under Ubuntu 18.04.



---
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



---
## Building from source

### Create Workspace
~~~
mkdir -p ~/panda_ws/src

cd ~/panda_ws/src

git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

git clone https://github.com/frankaemika/franka_ros.git -b melodic-devel

git clone https://github.com/Envq/panda_controller.git
~~~


### Configure Workspace
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


### Build Workspace
~~~
catkin build
~~~



---
## Nodes
A brief description of the nodes

### simulation
![simulation](screenshot/simulation.png?raw=true "simulation")

This node launch rviz with the panda arm


---
## VSCode
I used these extensions:
- **ros** by microsoft
- **c/c++** by microsoft
- **c/c++ snippets** by harsh
- **c++ intellisense** by austin
- **python** by microsoft
- **urdf** by smilerobotics
- **clang-format** by xaver
- **doxgen documentation** by christoph schlosser
- **cmake** by twxs
- **git history** by don jayamanne
- **gruvbox mirror** by adamsome
- **vscode-icons** by icons for visual studio code


The following commands are available:
- **build** : build this package
- **buildAndRun**: build this package and run the actual developing node
- **simulation**: run the node that launch rviz
- **format**: format sources with clang-format
- **build_all**: build all packages
- **clean_all**: clean all packages


---
## Author

**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq)

---
## License

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details
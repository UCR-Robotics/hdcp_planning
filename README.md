# hdcp_planning

## 1. Introduction

We propose an online Hex-Decomposed Coverage Planning (HDCP) algorithm that
- guarantees resolution-completeness coverage in unknown, cluttered environments,
- includes a variant HDCP-E to trade-off between exploration speed and coverage area,
- provides closed-form solutions for planning smooth paths that robots can follow at constant speed.

This public repository, organized as a ROS package, offers a complete implementation of the above algorithm.
It also provides basic building blocks, such as common operations in hex map and trajectory generation using Dubins paths, which we believe are helpful for future research and development.

**Authors:** Xinyue Kan, Hanzhe Teng, and Konstantinos Karydis from [ARCS Lab](https://sites.google.com/view/arcs-lab/) at [UC Riverside](https://www.ucr.edu/). 

**Videos:** Our presentation at IROS 2020 can be seen [here](https://youtu.be/-Kw2I0kJ-EM). 
A short 2-min supplementary video can be seen [here](https://youtu.be/T3ZazIgfVw8).

**Related Publications:**

X. Kan, H. Teng, and K. Karydis, "**Online Exploration and Coverage Planning in Unknown Obstacle-Cluttered Environments**", in *IEEE Robotics and Automation Letters*, vol. 5, no. 4, pp. 5969-5976, Oct. 2020 ([paper](https://ieeexplore.ieee.org/abstract/document/9144373), [preprint](https://arxiv.org/abs/2006.16460))

```latex
@article{kan2020online,
  title={Online Exploration and Coverage Planning in Unknown Obstacle-Cluttered Environments},
  author={Kan, Xinyue and Teng, Hanzhe and Karydis, Konstantinos},
  journal={IEEE Robotics and Automation Letters},
  volume={5},
  number={4},
  pages={5969--5976},
  year={2020},
  publisher={IEEE}
}
```

## 2. Installation

### 2.1 Prerequisites 
- Ubuntu 16.04 + ROS Kinetic (or Ubuntu 18.04 + ROS Melodic)
  - It is possible to run in Ubuntu 18.04, if we [build turtlebot2 from source](https://github.com/UCR-Robotics/Turtlebot2-On-Melodic) or switch to other robots equipped with the same sensors. See the last section for more information.
- [turtlebot2n_description](https://github.com/UCR-Robotics/turtlebot2n)
  - We use a customized turtlebot2 robot, equipped with RPLidar and Astra Pro camera, in both Gazebo simulation and experiments. This is a URDF description package that we developed to be compatible with the hardware. It depends on `turtlebot` meta-package (and `rplidar_ros` package if running hardware).
- [obstacle_detector](https://github.com/UCR-Robotics/obstacle_detector)
  - This is a ROS package that can extract line segments or circular obstacles from 2D laser range data. We forked from the [original package](https://github.com/tysik/obstacle_detector) to maintain consistency. (We thank the author for providing this package.)
  - [Armadillo library](http://arma.sourceforge.net/) is required by this obstacle detector. We will introduce how to install it in the following section.

### 2.2 Steps
We provide a step-by-step installation guideline, including all dependencies.

- Please follow ROS Wiki to [install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [create a ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). In the following, we assume the ROS workspace is at `~/catkin_ws`.

- Install `turtlebot` dependencies. Not all of them are required in simulation, but this is a complete list for hardware. (If running in Ubuntu 18, please skip this step and follow the instruction in [this repository](https://github.com/UCR-Robotics/Turtlebot2-On-Melodic) instead to install `turtlebot` dependencies.)
  ```
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
  sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
  ```

- Install Armadillo C++ linear algebra library for obstacle detector. You may use other folders other than `Downloads` as you like. (We provide a backup source in case the original one is expired.)
  ```
  cd ~/Downloads
  wget http://sourceforge.net/projects/arma/files/armadillo-9.700.2.tar.xz
  #wget https://github.com/UCR-Robotics/hdcp_planning/raw/armadillo/armadillo-9.700.2.tar.xz
  tar -xvf armadillo-9.700.2.tar.xz 
  cd armadillo-9.700.2
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```

- Git clone and compile ROS packages.
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/UCR-Robotics/obstacle_detector
  git clone https://github.com/UCR-Robotics/turtlebot2n
  git clone https://github.com/UCR-Robotics/hdcp_planning
  cd ~/catkin_ws
  catkin_make
  ```
Note: please double check if `hdcp.py` has executable permission to be run by roslaunch, though this should be already tracked by git.

## 3. Running HDCP

### 3.1 Basic Steps
Open three terminals and launch Gazebo simulator, obstacle detector and hdcp planner respectively.
```
roslaunch hdcp_planning gazebo.launch
roslaunch hdcp_planning obstacle_detector.launch
roslaunch hdcp_planning hdcp_planning.launch
```
Note: sequence does matter. Please wait for the previous one to be completed before launching the new one.

### 3.2 Optional Parameters
We offer the following roslaunch parameters to users:
- `gazebo.launch`
  - `x`: robot initial x coordinate (-10 to 10, default 0)
  - `y`: robot initial y coordinate (-10 to 10, default 0)
  - `world`: simulation world file (empty, cross, row, random)
- `hdcp_planning.launch`
  - `x`: robot initial x coordinate (-10 to 10, default 0)
  - `y`: robot initial y coordinate (-10 to 10, default 0)
  - `hex_radius`: radius or side length of each hexagon (default 1.0m)
  - `turning_radius`: robot turning radius when following circular trajectory in each hexagon (default 0.5m)
  - `data_folder`: folder to save hex map and trajectory data (hdcp, hdcp_e or others)
  - `exploration_mode`: run HDCP-E for fast exploration (true or false)
  - `debugging_mode`: show more details in visualization (true or false)
  - `obstacle_threshold`: to remove outlier (avoid false positive) in obstacle detection (recommended 50-200, default 100); use larger threshold for HDCP due to longer staying time in each hex, and smaller threshold for HDCP-E

More parameters not exposed to users are optimized internally (e.g., valid sensing range, control point offset, tracking controller param, etc.)

### 3.3 Sample Usage
#### run in an uniform environment
```
roslaunch hdcp_planning gazebo.launch world:=cross
roslaunch hdcp_planning obstacle_detector.launch
roslaunch hdcp_planning hdcp_planning.launch
```
#### spawn the robot at corner
```
roslaunch hdcp_planning gazebo.launch x:=-9 y:=-9
roslaunch hdcp_planning obstacle_detector.launch
roslaunch hdcp_planning hdcp_planning.launch x:=-9 y:=-9
```
#### run HDCP-E (enable exploration mode)
```
roslaunch hdcp_planning gazebo.launch
roslaunch hdcp_planning obstacle_detector.launch
roslaunch hdcp_planning hdcp_planning.launch exploration_mode:=true data_folder:=hdcp_e
```

### 3.4 Visualization Only
We also provide a python file that can be used offline to visualize previously saved data.
```
roscd hdcp_planning/scripts
python plot.py
```
By default, this will show an example set of data in `data/hdcp_e` folder that was captured by running HDCP-E algorithm from initial point [-9, -9] in random environment, with debugging option enabled.

## 4. Code Structure

### 4.1 Pipeline
- perception layer
    - read in point cloud data and register on hexmap
- hex and path planning
    - determine next hex to move according to hexmap
    - find inner/outer tangent points (start and end points)
- trajectory generation
    - calculate straight-line or circular trajectory
    - generate waypoints according to sampling period (discretization resolution)
- tracking controller
    - track waypoints according to current robot status
    - feedback linearization is used for handling non-holonomic kinematics

### 4.2 Files
- `scripts` folder
  - `hdcp.py` has the high-level implementation of the proposed HDCP algorithm,
    according to Algorithm 1 in our paper. 
  - `turtlebot.py` handles low-level perception, control and planning, while
    `hexmap.py` provides hex-decomposed map related operations.
  - `hexcell.py` and `vector2d.py` offer basic data structure used in the development.
  - `plot.py` shows an example of visualizing previously saved data.
  - All variables and functions are named explicitly to facilitate understanding. 
- `models`folder
  - We use a standard `pine_tree` model provided by Gazebo, and a customized `bare_tree` model provided in `models` folder. In the launch file, we provide `GAZEBO_MODEL_PATH` environment variable pointing to this folder, such that it can be found and loaded in Gazebo simulator. When you launch Gazebo simulation for the first time, it may take some time for Gazebo to download `pine_tree` model from its model library. 


## 5. Future Work
We acknowledge that some designs are not optimal for now. Future work can include
- adding a subclass of HexCell class to include status information (free, occupied, visited, etc.),
- using probabilistic update for observed obstacles,
- more efficient A* online replanning,
- reducing dependencies on turtlebot and supporting other robots.

Currently, `turtlebot` meta-package has not been released under ROS Melodic (not able to install by `sudo apt install ros-melodic-turtlebot`), due to deprecated dependencies. It is possible to build from source or switch to other robots with the same configuration. Please follow the instruction in [this repository](https://github.com/UCR-Robotics/Turtlebot2-On-Melodic) to install `turtlebot` in Ubuntu 18. 

**Our framework is compatible with any other robot (e.g., [ROSbot 2.0](https://store.husarion.com/products/rosbot)) that can provide 2D lidar point cloud and respond to `cmd_vel` (geometry_msgs/Twist) commands.** Though named after turtlebot, the `turtlebot.py` file by itself does not rely on any component in the turtlebot package. Only topic names in subscriber/publisher need to be changed.

You are very welcome to contact us or open a new issue in this repository if you have any questions/comments!


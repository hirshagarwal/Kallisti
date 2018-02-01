# Kallisti
System Design Project Repository

# Installation Instructions
These instructions are designed for Ubuntu 14. Any other operating system might work, but has not been tested.
### Update OS
~~~~
sudo apt-get update
sudo apt-get upgrade
~~~~
### Install Standard ROS, Extra Dependencies and Tools
~~~~
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install libgl1-mesa-dev-lts-utopic
sudo apt-get install ros-indigo-desktop-full 
sudo apt-get install python-rosinstall python-rosdep 
sudo rosdep init
rosdep update 
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
~~~~
### Start ROS Workspace
~~~~
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/src 
catkin_init_workspace 
cd ~/catkin_ws/ 
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
~~~~
### LSD Slam Install
~~~~
sudo apt-get install ros-indigo-libg2o ros-indigo-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev libx11-dev
sudo apt-get install python-rosinstall
mkdir ~/rosbuild_ws
cd ~/rosbuild_ws
rosws init . /opt/ros/indigo
mkdir package_dir
rosws set ~/rosbuild_ws/package_dir -t .
echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
bash
cd package_dir
git clone https://github.com/tum-vision/lsd_slam.git lsd_slam
~~~~
### ROSmake Repair
In order to succesfully run the ROSmake command two configuration files need the be changed:
* rosbuild_ws/package_dir/lsd_slam/lsd_slam_viewer/cfg/LSDSLAMViewerParams.cfg
* rosbuild_ws/package_dir/lsd_slam/lsd_slam_core/cfg/LSDParams.cfg
In each of these files replace all of the ' (single quotation marks) with \` (backtick) **except** in the first line.
~~~~
rosmake lsd_slam
~~~~

# Running LSD Slam
### Check Live Topics
~~~~
rostopic list
~~~~
### Install OpenCV Streamer
~~~~
sudo apt-get install ros-indigo-video-stream-opencv
~~~~

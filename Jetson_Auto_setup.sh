#!/bin/bash 

# InDro Robotics
# Austin Greisman - austin.greisman@indrorobotics.com

# Used for install ROS Melodic, Swift Nav, Teensy, and Agile X Software
echo "Starting Install..."

read -e -p "What is the sudo password?: " PASS
echo $PASS | sudo -S apt update 
sudo apt dist-upgrade -y
sudo apt install curl -y
sudo apt install git -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-melodic-desktop-full -y

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt install python-rosdep -y
sudo apt install python-pip python3-pip -y

sudo apt autoremove

sudo rosdep init
rosdep update

# Make your catkin workspace
echo "Creating Catkin Workspace"
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make

echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Git Pull the proper repos
echo "Pulling Git Repos for SwiftNav and Rosserial"
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git --branch melodic-devel
git clone https://github.com/austin-inDro/swift_pgm.git

echo "Installing necessary SwiftNav packages"
cd ~/catkin_ws/src/swift_pgm
python3 -m pip install -r requirements.txt
python3 -m pip install catkin_pkg # May be needed for Ros_Upstart
cd ~/catkin_ws/src/

# Installing Agile X Packages
echo "Installing Agile X Packages"
sudo apt install libasio-dev -y 
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard -y 
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui -y 
sudo apt install ros-$ROS_DISTRO-ros-controllers -y 

git clone https://github.com/agilexrobotics/ugv_sdk.git  
git clone https://github.com/agilexrobotics/scout_ros.git
git clone https://github.com/agilexrobotics/hunter_ros.git
git clone https://github.com/agilexrobotics/tracer_ros.git
git clone https://github.com/agilexrobotics/bunker_ros.git
git clone https://github.com/clearpathrobotics/robot_upstart.git --branch melodic-devel

cd ~/catkin_ws
catkin_make

#Installing Teensy Drivers
echo "Installing Teensy Drivers"
cd ~/Downloads
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/

rm 00-teensy.rules

# wget https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linuxarm
# sudo chmod 755 TeensyduinoInstall.linuxarm
# ./TeensyduinoInstall.linuxarm

source ~/.bashrc
echo "Attempting to enable the Can0 port. Ensure it's connected to the Jetson"
# Enable CAN-To-USB 
sudo modprobe gs_usb

echo "Running bring up commands..."
rosrun tracer_bringup setup_can2usb.bash
rosrun tracer_bringup bringup_can2usb.bash

# Setup Reboot commands
echo "Setting up system for automatic ROS boot"
cd ~/catkin_ws/src

catkin_create_pkg riab_startup std_msgs rospy
# vi ROS_boot.launch
echo "You still need to create the"
echo "rosrun robot_upstart install myrobot_bringup/launch/base.launch"

# Connecting Rocos
echo "Setting Up Rocos.."
echo "You will need to interact with the terminal"
echo "deb https://packages.rocos.io/apt stable main" | sudo tee -a /etc/apt/sources.list.d/rocos.list
curl https://packages.rocos.io/apt/docs/key.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install rocos-agent -y
sudo rocos-agent

echo "Attempting to start the rocos-agent for the first time"
# sudo systemctl start rocos-agent
echo "Rocos should be working... Moving on"

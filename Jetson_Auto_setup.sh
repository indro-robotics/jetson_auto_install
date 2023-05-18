#!/bin/bash 

# InDro Robotics
# Austin Greisman - austin.greisman@indrorobotics.com
# Arif Anjum  - arif.anjum@indrorobotics.com

# Used for install ROS Noetic, Swift Nav, Teensy, and Agile X Software

# INDROROBOTICS Background Installation
gsettings set org.gnome.desktop.background picture-uri "file:///home/$USER/jetson_auto_install/InDroRobotics.png"

# IndroRobotics Software Installation
echo "Starting Install..."
read -e -p "What is the sudo password?: " PASS
echo $PASS | sudo -S apt update 
sudo apt dist-upgrade -y
sudo apt install curl -y
sudo apt install git -y

# Robot Operating Sysytem Installation (ROS Noetic)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y
sudo apt install python3-pip -y

sudo apt autoremove -y

sudo rosdep init
rosdep update

source ~/.bashrc

# Make your catkin workspace
echo "Creating Catkin Workspace"
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
source ~/.bashrc
catkin_make

echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Git Pull the proper repos
echo "Pulling Git Repos for and Rosserial"
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git --branch noetic-devel
# git clone https://github.com/indro-robotics/swift_pgm.git
# echo "Installing necessary SwiftNav packages"
# cd ~/catkin_ws/src/swift_pgm
# python3 -m pip install -r requirements.txt
python3 -m pip install catkin_pkg # May be needed for Ros_Upstart
cd ~/catkin_ws/src/

# Installing Agile X Packages
echo "Installing Agile X Packages"
sudo apt install libasio-dev -y 
sudo apt install ros-noetic-teleop-twist-keyboard -y 
sudo apt install ros-noetic-joint-state-publisher-gui -y 
sudo apt install ros-noetic-ros-controllers -y 

git clone https://github.com/agilexrobotics/ugv_sdk.git  
git clone https://github.com/agilexrobotics/scout_ros.git
git clone https://github.com/agilexrobotics/hunter_ros.git
git clone https://github.com/agilexrobotics/tracer_ros.git
git clone https://github.com/agilexrobotics/bunker_ros.git
git clone https://github.com/clearpathrobotics/robot_upstart.git --branch noetic-devel


cd ~/catkin_ws
catkin_make

#Installing Teensy Drivers
echo "Installing Teensy Drivers"
cd ~/Downloads
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/

rm 00-teensy.rules


source ~/.bashrc
echo "Attempting to enable the Can0 port. Ensure it's connected to the Jetson"

# Enable kernel module: gs_usb
sudo modprobe gs_usb

# Bring up can interface
sudo ip link set can0 up type can bitrate 500000

# Install can utils
sudo apt install -y can-utils


#CAN BUS INSTALLATION
#bring Down can interface
sudo ip link set can0 down type can bitrate 500000

cd
cd ../..

#CANBUS Automatically Restart on Startup
sudo touch /etc/systemd/network/80-can.network && {
echo '[Match]'
echo 'Name=can0'
echo '[CAN]'
echo 'BitRate=500K'
} | sudo tee /etc/systemd/network/80-can.network

#SYSTEMD-Networkd setup for CANBUS Automatically Restart on Startup
sudo systemctl start systemd-networkd
sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd






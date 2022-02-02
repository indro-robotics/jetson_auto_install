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
sudo apt-get install -y setpriv

sudo apt autoremove -y

sudo rosdep init
rosdep update

source ~/.bashrc

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

# Installing Thermal Camera
git clone https://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@github.com/indro-robotics/WALL-E-Thermal-Camera.git
cd ~/catkin_ws/src/WALL-E-Thermal-Camera

sudo apt-get install libxml2-dev libxslt-dev python-dev -y
python3 -m pip install -r requirements.txt

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
cd ~/catkin_ws/src/riab_startup
mkdir launch
cd launch
touch boot.launch
echo "You will need to populate the boot.launch file with what you want to happen at boot..."
rosrun robot_upstart install riab_startup/launch/boot.launch --job ros_boot --symlink
sudo systemctl daemon-reload

# Adding CAN bus support into kernel for boot
sudo bash -c '

auto can0
  iface can0 inet manual
  pre-up /sbin/ip link set can0 type can bitrate 500000
  up /sbin/ifconfig can0 up
  down /sbin/ifconfig can0 downauto can0
  iface can0 inet manual
  pre-up /sbin/ip link set can0 type can bitrate 500000
  up /sbin/ifconfig can0 up
  down /sbin/ifconfig can0 down" >> /etc/network/interfaces'
  
sudo systemctl daemon-reload
sudo systemctl restart systemd-networkd
sudo systemctl start ros_boot.service

# Adding a time fix into network up...
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR
sudo chmod +x startup.sh
sudo mv startup.sh /etc/network/if-up.d/
sudo bash -c 'echo -e "[Unit]
# Type=simple|forking|oneshot|dbus|notify|idle
Description=Time Fix daemon
## make sure we only start the service after network is up
Wants=network-online.target
After=network.target

[Service]
## here we can set custom environment variables
Environment=AUTOSSH_GATETIME=0
Environment=AUTOSSH_PORT=0
ExecStart=/etc/network/if-up.d/startup.sh
# Useful during debugging; remove it once the service is working
StandardOutput=console

[Install]
WantedBy=multi-user.target" > /etc/systemd/system/timefix.service'

sudo systemctl daemon-reload
sudo systemctl start timefix
sudo systemctl enable timefix

cd ~

# Setting Up WireGuard VPN
sudo apt-get install wireguard openresolv -y

# Connecting Rocos
echo "Setting Up Rocos.."
echo "You will need to interact with the terminal"
echo "deb https://packages.rocos.io/apt stable main" | sudo tee -a /etc/apt/sources.list.d/rocos.list
curl https://packages.rocos.io/apt/docs/key.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install rocos-agent -y
sudo rocos-agent


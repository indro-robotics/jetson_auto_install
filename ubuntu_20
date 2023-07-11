#!/bin/bash

# INDROROBOTICS Background Installation
gsettings set org.gnome.desktop.background picture-uri "file://$(pwd)/InDroRobotics.png"

# IndroRobotics Software Installation
echo "Starting Install..."
read -e -p "What is the sudo password?: " PASS
sleep 5
echo $PASS | sudo -S apt update 
sudo apt dist-upgrade -y
sudo apt install curl -y
sudo apt install git -y



# ROS2 - Foxy installation:

echo "Installing ROS2 - Foxy"
sleep 5
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -y
sudo apt upgrade -y  
sudo apt install ros-foxy-desktop python3-argcomplete -y
sudo apt install ros-dev-tools -y
sudo apt install ros-foxy-rmw-cyclonedds-cpp

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

sleep 5
sudo apt install python3-pip -y
sudo apt autoremove -y

#setting up kernal buffer for cyclonedds
sudo touch /etc/sysctl.d/10-cyclone-max.conf && {
    echo 'net.core.rmem_max=2147483647'
} | sudo tee /etc/sysctl.d/10-cyclone-max.conf

#installing terminator
sleep 5
sudo apt install terminator -y

# Nvidia-Jetpack tools Installation
echo "Starting jepack installation..."
sleep 5
sudo apt-get install nvidia-jetpack

# Docker installation
echo "Starting docker installation:..."
sleep 5
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null



sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
sudo usermod -aG docker $USER

#installting Zerotier

echo "installing Zerotier"
sleep 5
curl -s 'https://raw.githubusercontent.com/zerotier/ZeroTierOne/master/doc/contact%40zerotier.com.gpg' | gpg --import && \
if z=$(curl -s 'https://install.zerotier.com/' | gpg); then echo "$z" | sudo bash; fi

echo "make sure to reboot the computer once this script has run succesfully"
#can-bus rules 

echo "Attempting to enable the Can0 port. Ensure it's connected to the Jetson"
sleep 5
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


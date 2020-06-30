#!/bin/bash
#Author: Eech Hsiao 

echo ""
echo "[INFO] Required OS Version >>> Ubuntu 18.04 (Bionic Beaver)"
echo "[INFO] Target ROS Version  >>> ROS Melodic Morena"
echo "[INFO] Catkin Workspace    >>> $HOME/catkin_ws"
echo "THIS INSTALL SCRIPT IS FOR DESIGNED SPECIFICALLY FOR McMASTER SIMULATIONS COURSE"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read -p "PRESS ENTER TO CONTINUE"

ROS_DISTRO=${ROS_DISTRO:="melodic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo "[INFO] INSTALLING ROS"
echo "[INFO] Written for McMaster University // Ubuntu 18.04 // ROS Melodic Morena"

echo "[INFO] Adding ROS Repository"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "[INFO] Downloading ROS Keys"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo "[INFO] Adding Intel RealSense Repository"
if [ ! -e /etc/apt/sources.list.d/danielrichter2007-ubuntu-grub-customizer-bionic.list ]; then
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
fi

echo "[INFO] Downloading Intel RealSense Keys"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

echo "[INFO] Update and Upgrade Packages"
sudo apt-get update -y
sudo apt-get dist-upgrade -y

echo "[INFO] Installing ROS and Other Packages"
sudo apt install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-ros-controllers \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-desktop-full

# Initialize and Update rosdep 
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "[INFO] Environment Setup"
source /opt/ros/$ROS_DISTRO/setup.sh
printf "\nsource /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc

echo "[INFO] Create Catkin Workspace"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin_make

echo "[INFO] Sourcing catkin_workspace"
printf "\nsource ~/$name_catkin_workspace/devel/setup.bash" >> $HOME/.bashrc

echo "[INFO] Exporting SVGA for VMware"
printf "\nexport SVGA_VGPU10=0" >> $HOME/.bashrc

echo "[INFO] Fixing Gazebo REST Request Error"
sed -i -e 's,https://api.ignitionfuel.org,https://api.ignitionrobotics.org,g' ~/.ignition/fuel/config.yaml

echo "[INFO] Finished Full Installation"

exit 0

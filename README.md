# iarc7-common

Contains common documents and notably the rosinstall to initialize the RAS IARC7 2017 ROS workspace.


## RAS IARC7 2017 Software Requirements

- Ubuntu 14.04
- ROS Jade
- Morse
- Wstool

### Ubuntu 14.04

	ROS Jade requires ubuntu 14.04, 14.10, or 15.04.

	If you don't have ubuntu installed, install 14.04 because its LTS (Long term service)

    http://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr

    Or make a chroot environment if you don't want to leave your current ubuntu distro.
    http://wiki.ros.org/ROS/Tutorials/InstallingIndigoInChroot

### ROS Indigo

    http://wiki.ros.org/indigo/Installation/Ubuntu

    Install ros-indigo-desktop-full

### Morse

    TBD

### wstool

After installing ROS

```bash
sudo apt-get install python-wstool
```

## Setting up a workspace

```bash
# Create workspace folder in your home directory
cd ~
mkdir -p iarc7/src

# Enter workspace
cd iarc7

# Clone iarc7 common for rosinstall file
git clone https://github.com/Pitt-RAS/iarc7-common.git

# Load rosinstall file to download iarc7 packages
wstool init src iarc7-common/main.rosinstall

# Create build and devel folders
catkin_make

# Source your environment variables
source devel/setup.bash

# If you want the enivornment sourced upon login
echo "source devel/setup.bash" >> ~/.bashrc
```
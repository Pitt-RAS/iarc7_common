# iarc7-common

Master repository for the Pitt RAS team competing in Mission 7 of the International Aerial Robotics Competition

For other IARC teams finding this repository, we've made it public intentionally. We would love for you to take a look around and send us any questions you have (or any bugfixes you develop). This code is open source and free to use under the GPL, but we do ask that other IARC teams using this code or ideas taken from it cite the Pitt Robotics and Automation Society and do not present the work or the ideas contained within it as their own.

[The wiki on this repository](https://github.com/Pitt-RAS/iarc7_common/wiki) contains a thorough description of our entire system, along with detailed information on various subsystems.  If you're interesed in learning more about our design, it's a good place to start.

This repository specifically contains common documents and files needed to set up our software systems.  Installation instructions can be found below.

Note: The default install instructions for these packages will not work.  You can get away with defaults for Ubuntu, ROS Kinetic, and gcc6, but the default install instructions will not work for wstool, Morse, or OpenCV.

## Table of Contents

### [RAS IARC7 2017 Software Requirements](#installation-instructions)

- [Ubuntu 16.04](#ubuntu-1604)
- [ROS Kinetic](#installing-ros-kinetic)
- [gcc6 and gcc5](#installing-gcc6-and-gcc5)
- [CUDA](#installing-cuda)
- [wstool](#setting-up-a-workspace-with-wstool)
- [Morse](#installing-morse)
- [OpenCV 2.4.13](#installing-opencv)

### [Misc Other Instructions](#miscellaneous-other-notes)

- [SSH Authentication With Workspaces](#ssh-authentication-for-github)
- [Updating on changed dependencies](#updating-on-changed-dependencies)

## Installation Instructions

### Ubuntu 16.04

ROS Kinetic requires ubuntu 16.04

If you don't have Ubuntu installed, install version 16.04 because it's LTS (Long term service). There's a useful tutorial [here](https://www.ubuntu.com/download/desktop/install-ubuntu-desktop).

If you already have Ubuntu, but a version different from 16.04, you can install Ubuntu 16.04 in a chroot environment under your existing operating system.  To do this, follow the instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingIndigoInChroot), but replace every occurence of "Indigo" with "Kinetic" and "trusty" with "xenial".

### Installing ROS Kinetic

Run the following (you can copy and paste the whole thing):

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116 && \
sudo apt-get update && \
sudo apt-get install ros-kinetic-ros-base && \
sudo rosdep init && \
rosdep update
```

### Installing gcc6

Run the following, you can copy and paste as one command:

    sudo apt-get update && \
    sudo apt-get install build-essential software-properties-common -y && \
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    sudo apt-get update && \
    sudo apt-get install gcc-snapshot -y && \
    sudo apt-get update && \
    sudo apt-get install gcc-6 g++-6 -y && \
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6 && \
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5;

The full instructions are here but you shouldn't need them: https://gist.github.com/application2000/73fd6f4bf1be6600a2cf9f56315a2d91

### Installing CUDA (NVIDIA cards only, non-NVIDIA users can skip this part)

Enable NVIDIA drivers (this depends on what GPU you have, you'll need to look up instructions online)

Select the gcc-5 compiler using the onscreen prompts

    sudo update-alternatives --config gcc

Download the CUDA 8 package

    cd ~/Downloads && \
    wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run && \
    mv cuda_8.0.61_375.26_linux-run.1 cuda_8.0.61_375.26_linux.run && \
    chmod +x cuda_8.0.61_375.26_linux.run && \
    sudo ./cuda_8.0.61_375.26_linux.run --silent --toolkit --samples --override

If continuing to install opencv do not switch back to gcc-6 using update-alternatives. Otherwise switch back to gcc-6

    sudo update-alternatives --config gcc

### Installing OpenCV

Select the gcc-5 compiler using the onscreen prompts

    sudo update-alternatives --config gcc

It is easiest to install OpenCV from source given that we are using a specific version. Use version 2.4.13 (this is the version that OpenCV4Tegra is based on).

Unzip this snapshot of source in your home directory:

    https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.13/opencv-2.4.13.zip/download

The instructions for install opencv are based off of this guide [here](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html).

Install packages

    sudo apt-get install build-essential -y && \
    sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y && \
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev -y;

Setup build directory

    cd ~/opencv-2.4.13 && \
    mkdir release && \
    cd release && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..;

Build, replace the number for with the number of cores you have (This will take a hot minute)

    make -j4

Install

    sudo make install

Remember to switch back to gcc6 once OpenCV is installed.

### Setting up a workspace with `wstool`

First, install ROS's tool for managing workspaces:

    sudo apt-get install python-wstool

Then, create a workspace.  This folder can be anywhere, but here's how to create one in your home directory:

    mkdir -p ~/iarc7/src

Clone the iarc7\_common repository:

    cd ~/iarc7
    git clone https://github.com/Pitt-RAS/iarc7_common.git

Source ros variables:

    source /opt/ros/kinetic/setup.bash

Download the repos:

    wstool init src iarc7_common/main.rosinstall

Install dependencies:

    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

Build for the first time

    catkin_make

And finally, make your ROS environment be set up automatically in the future:

    echo "source ~/iarc7/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc;

If you don't do this, you'll have to run `source ~/iarc7/devel/setup.bash` every time you open a new terminal.

### Installing Morse

Install required packages:

    sudo apt-get install cmake python3-dev python3-numpy

Download and unzip blender:
We need this specific version because it matches the system python version.

    cd ~ && \
    wget http://download.blender.org/release/Blender2.78/blender-2.78c-linux-glibc219-x86_64.tar.bz2 && \
    tar xvjf blender-2.78c-linux-glibc219-x86_64.tar.bz2;

Clone the latest morse and build/install it:

    git clone https://github.com/morse-simulator/morse.git && \
    cd morse && \
    mkdir build && cd build && \
    cmake .. && \
    sudo make install;

Add blender environment variable, assumes you installed blender in your home directory:

    echo "export MORSE_BLENDER="~/blender-2.78c-linux-glibc219-x86_64/blender" ">> ~/.bashrc && \
    source ~/.bashrc;

Check if everything is ok with:

    morse check

Install ROS support for MORSE (Note: DO NOT install `python3-rospkg` with `apt-get`)

    sudo apt-get install python3-pip && \
    pip3 install --ignore-installed --install-option="--prefix=~/blender-2.78c-linux-glibc219-x86_64/2.78/python" rospkg catkin_pkg

Download iarc7\_simulator (instructions copied from https://github.com/Pitt-RAS/iarc7_simulator)

    cd ~/iarc7 && /
    wstool merge -t src iarc7_common/simulator.rosinstall;

If this is your first time setting up an iarc workspace run

    wstool update -t src

Now import and compile the sim

    cd ~/iarc7 && \
    cd src/iarc7_simulator && \
    morse import sim && \
    cd ~/iarc7 && \
    catkin_make;

To launch the simulator (It won't do anything, just render the scene)

    roslaunch iarc7_simulator morse.launch

## Miscellaneous other notes

### SSH Authentication for GitHub

If you wish to use ssh keys instead of http authentication:

    cd ~/iarc7
    find . -path "./src/*/.git/config" | xargs -n 1 sed -i "s/https:\/\/github.com\//git@github.com:/"
    sed -i "s/https:\/\/github.com\//git@github.com:/" ./src/.rosinstall

### Updating on Changed Dependencies

When new packages are added to the `rosinstall` files in this repository, run
the following to update:

```sh
cd ~/iarc7/iarc7_common && \
git pull && \
cd ~/iarc7 && \
wstool merge -t src iarc7_common/main.rosinstall && \
wstool update -t src
```

If you're using ssh authentication, run the snippet under
[SSH Authentication for GitHub](#ssh-authentication-for-github) again after
running the above.

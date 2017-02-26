# iarc7-common

Master repository for the Pitt RAS team competing in Mission 7 of the International Aerial Robotics Competition

Contains common documents and notably the rosinstall to initialize the ROS workspace.

Note: The default install instructions for these packages will not work.  You can get away with defaults for Ubuntu, ROS Jade, and gcc6, but the default install instructions will not work for wstool, Morse, or OpenCV.

## RAS IARC7 2017 Software Requirements

- [Ubuntu 14.04](#ubuntu-1404)
- [ROS Jade](#installing-ros-jade)
- [gcc6](#installing-gcc6)
- [wstool](#setting-up-a-workspace-with-wstool)
- [Morse](#installing-morse)
- [OpenCV 2.4.13](#installing-opencv)

## Installation Instructions

### Ubuntu 14.04

ROS Jade requires ubuntu 14.04, 14.10, or 15.04.

If you don't have Ubuntu installed, install version 14.04 because it's LTS (Long term service). There's a useful tutorial [here](http://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr).

If you already have Ubuntu, but a version different from 14.04, you can install Ubuntu 14.04 in a chroot environment under your existing operating system.  To do this, follow the instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingIndigoInChroot), but replace every occurence of "Indigo" with "Jade".

### Installing ROS Jade

Run the following (you can copy and paste the whole thing):

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116 && \
sudo apt-get update && \
sudo apt-get install ros-jade-desktop-full && \
sudo rosdep init && \
rosdep update
```

If installing on ARM you will need to make this change to remove a warning that will fail the build.
The file affected is `/opt/ros/jade/include/ros/serialization.h`.
https://github.com/ros/roscpp_core/commit/4325fb7c9b31c739f4c86fe2a76a47055a5f56fa#diff-8fb8fffd5bd285b1d28f2d903953b067L204

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
    sudo apt-get install gcc-4.8 g++-4.8 -y && \
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8;

The full instructions are here but you shouldn't need them: https://gist.github.com/application2000/73fd6f4bf1be6600a2cf9f56315a2d91

### Setting up a workspace with `wstool`

First, install ROS's tool for managing workspaces:

    sudo apt-get install python-wstool

Then, create a workspace.  This folder can be anywhere, but here's how to create one in your home directory:

    mkdir -p ~/iarc7/src

Clone the iarc7\_common repository:

    cd ~/iarc7
    git clone https://github.com/Pitt-RAS/iarc7_common.git

Source ros variables:

    source /opt/ros/jade/setup.bash

Download the repos:

    wstool init src iarc7_common/main.rosinstall

Install dependencies:

    rosdep install --from-paths src --ignore-src --rosdistro=jade -y

Build for the first time

    catkin_make

And finally, make your ROS environment be set up automatically in the future:

    echo "source ~/iarc7/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

If you don't do this, you'll have to run `source ~/iarc7/devel/setup.bash` every time you open a new terminal.

### Installing Morse

Install required packages:

    sudo apt-get install cmake python3-dev python3-numpy

Download and unzip blender:
We need this specific version as in v2.77 they switched to a newer version of python that ubuntu 14.04 does not come with.

    cd ~
    wget http://download.blender.org/release/Blender2.76/blender-2.76b-linux-glibc211-x86_64.tar.bz2
    tar xvjf blender-2.76b-linux-glibc211-x86_64.tar.bz2

Clone the latest blender and build/install it:

    git clone https://github.com/morse-simulator/morse.git
    cd morse
    mkdir build && cd build
    cmake ..
    sudo make install

Add blender environment variable, assumes you installed blender in your home directory:

    echo "export MORSE_BLENDER="~/blender-2.76b-linux-glibc211-x86_64/blender" ">> ~/.bashrc

Disable python version checking. The closest we can get to matching the system python and blender python versions is 3.4.3 and 3.4.2 respectively without building blender from source:

    echo "export MORSE_SILENT_PYTHON_CHECK=1 ">> ~/.bashrc

Now close the terminal and open a new one.  Then you can see if everything is ok with:

    morse check

Install ROS support for MORSE (Note: DO NOT install `python3-rospkg` with `apt-get`)

    sudo apt-get install python3-pip
    pip3 install --ignore-installed --install-option="--prefix=~/blender-2.76b-linux-glibc211-x86_64/2.76/python" rospkg catkin_pkg

Make sure that the ROS integration was successful (if it was, the simulator should open with a demo scene.  It will crash if the installation failed.)

    morse run /usr/local/share/morse/examples/tutorials/tutorial-1-ros.py

Try a simulator!

    cd ~/iarc7
    mkdir morse
    cd morse
    morse create my_first_sim
    cd my_first_sim
    morse run my_first_sim

Use the arrow keys to drive around! http://www.openrobots.org/morse/doc/stable/quickstart.html for more info

Download iarc7\_simulator (instructions copied from https://github.com/Pitt-RAS/iarc7_simulator)

    cd ~/iarc7
    wstool merge -t src iarc7_common/simulator.rosinstall

If using ssh use this instead:
    wstool merge -t src iarc7_common/simulatorssh.rosinstall

If this is your first time setting up an iarc workspace run
    wstool update -t src

If your repositories are not on the branch master running wstool update can have undesirable results run one of these to manually clone the sim:

If using https

    cd src ; git clone https://github.com/Pitt-RAS/iarc7_simulator

If using ssh

    cd src ; git clone git@github.com:Pitt-RAS/iarc7_simulator.git

Now import and compile the sim

    cd src/iarc7_simulator
    morse import sim
    cd ~/iarc7
    catkin_make

To launch the simulator (make sure you've run `catkin_make` and sourced the correct setup script first, or else this won't work)

    roslaunch iarc7_simulator morse.launch

### Installing OpenCV

It is easiest to install OpenCV from source given that we are using a specific version. Use version 2.4.13 (this is the version that OpenCV4Tegra is based on).

Unzip this snapshot of source:
https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.13/

Switch to gcc4 for this (the above instructions made gcc4 and gcc6 available)

    sudo update-alternatives --config gcc

Select gcc4 using the printed out menu.

Finally, build and install OpenCV using [these instructions](http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html).

Remember to switch back to gcc6 once OpenCV is installed.

## Miscellaneous other notes

### SSH Authentication for GitHub

If you wish to use ssh keys instead of http authentication:

    cd ~/iarc7
    find . -path "./src/*/.git/config" | xargs -n 1 sed -i "s/https:\/\/github.com\//git@github.com:/"
    sed -i "s/https:\/\/github.com\//git@github.com:/" ./src/.rosinstall

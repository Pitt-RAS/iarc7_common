# iarc7-common

Contains common documents and notably the rosinstall to initialize the RAS IARC7 2017 ROS workspace.


## RAS IARC7 2017 Software Requirements

- Ubuntu 14.04
- ROS Jade
- Morse
- wstool

### Ubuntu 14.04

ROS Jade requires ubuntu 14.04, 14.10, or 15.04.

If you don't have ubuntu installed, install 14.04 because its LTS (Long term service)

http://howtoubuntu.org/how-to-install-ubuntu-14-04-trusty-tahr

Or make a chroot environment if you don't want to leave your current ubuntu distro.
http://wiki.ros.org/ROS/Tutorials/InstallingIndigoInChroot

### ROS Jade

Run the following:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get install ros-jade-desktop-full
    sudo rosdep init
    rosdep update

### Install gcc6

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

### Setting up a workspace

First, install ROS's tool for managing workspaces:

    sudo apt-get install python-wstool

Then, create a workspace.  This folder can be anywhere, but here's how to create one in your home directory:

    mkdir -p ~/iarc7/src

Clone the iarc7\_common repository:

    cd ~/iarc7
    git clone https://github.com/Pitt-RAS/iarc7_common.git

Initialize workspace:
    source /opt/ros/jade/setup.bash
    wstool init src iarc7_common/main.rosinstall
    catkin_make

And finally, make your ROS environment be set up automatically in the future:

    echo "source ~/iarc7/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

If you don't do this, you'll have to run `source ~/iarc7/devel/setup.bash` every time you open a new terminal.

### SSH-keys

If you wish to use ssh keys instead of http authentication.

    cd ~/iarc7
    find . -path "./src/iarc7_*/.git/config" | xargs -n 1 sed -i "s/https:\/\/github.com\//git@github.com:/"

### Morse

See <https://github.com/amiller27/iarc7-simulator> for install instructions.


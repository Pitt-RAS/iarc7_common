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

Source ros variables:

    source /opt/ros/jade/setup.bash

Download the repos:

    wstool init src iarc7_common/main.rosinstall

Use this if you want to use ssh instead:
    wstool init src iarc7_common/mainssh.rosinstall

Build for the first time

    catkin_make

And finally, make your ROS environment be set up automatically in the future:

    echo "source ~/iarc7/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

If you don't do this, you'll have to run `source ~/iarc7/devel/setup.bash` every time you open a new terminal.

### SSH-keys

If you wish to use ssh keys instead of http authentication after setting up through https

    cd ~/iarc7
    find . -path "./src/iarc7_*/.git/config" | xargs -n 1 sed -i "s/https:\/\/github.com\//git@github.com:/"
    find . -path "./src/.rosinstall" | xargs -n 1 sed -i "s/https:\/\/github.com\//git@github.com:/"

### Morse

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

Install ROS support for MORSE

    sudo apt-get install python3-pip
    pip3 install --install-option="--prefix=~/blender-2.76b-linux-glibc211-x86_64/2.76/python" rospkg catkin_pkg catkin

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

    cd src ; git clone git@github.com:Pitt-RAS/iarc7_simulator.git

If using ssh

    cd src ; git clone https://github.com/Pitt-RAS/iarc7_simulator

Now import and compile the sim

    cd src/iarc7_simulator
    morse import sim
    cd ~/iarc7
    catkin_make

To launch the simulator (make sure you've run `catkin_make` and sourced the correct setup script first, or else this won't work)

    roslaunch iarc7_simulator morse.launch

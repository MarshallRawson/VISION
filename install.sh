#!/bin/bash
GOODCOLOR='\033[1;36m'
WARNCOLOR='\033[1;31m'
NOCOLOR='\033[0m'
GOODPREFIX="${GOODCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"

# Sane installation defaults for no argument cases
CATKIN_DIR=~/repos/catkin_ws
REQUIRED_OS="trusty"
SEMAPHORE=false

instlog() {
    printf "$GOODPREFIX $@ $NOCOLOR\n"
}

instwarn() {
    printf "$WARNPREFIX $@ $NOCOLOR\n"
}

DTETCTED_OS="`lsb_release -sc`"

if [ $DTETCTED_OS != $REQUIRED_OS ]; then
    instwarn "The Sub requires Ubuntu 14.04 (trusty)"
    instwarn "Terminating installation due to incorrect OS (detected $DTETCTED_OS)"
    exit 1
fi

#======================#
# Script Configuration #
#======================#

while [ "$#" -gt 0 ]; do
  case $1 in
    -h) printf "\nUsage: $0 \n
    [-c] catkin_workspace (Recommend: ~/repos/catkin_ws)\n
    example: ./install.sh -c ~/repos/catkin_ws
    \n"; exit ;;
    # TODO: Use this to check if catkin ws already set up
    -c) CATKIN_DIR="$2"
        shift 2;;
    -?) instwarn "Option $1 is not implemented"; exit ;;
  esac
done

# Spooky activity to check if we are in semaphore
if ls ~/ | grep --quiet Sub8$; then
    instlog "Found Sub8 in HOME, Assuming we're in Semaphore"
    SEMAPHORE=true
    CATKIN_DIR=~/repos/catkin_ws
fi


#==================================#
# Repository and Dependancy Set Up #
#==================================#

# Make sure script dependancies are installed on bare bones installations
sudo apt-get update -qq
sudo apt-get install -qq wget aptitude git

# Add software repositories for ROS and Gazebo
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'

# Get the GPG signing keys for the above repositories
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Install ROS and other project dependancies
instlog "Installing ROS and Gazebo"
sudo apt-get update -qq
sudo apt-get install -qq ros-indigo-desktop python-catkin-pkg python-rosdep
sudo apt-get install -qq gazebo7

# Sources ROS configurations for bash on this user account
source /opt/ros/indigo/setup.bash
if ! cat ~/.bashrc | grep "source /opt/ros"; then
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
fi

# Get information about ROS versions
sudo rosdep init
rosdep update


#=====================================#
# Workspace and Sub Repository Set Up #
#=====================================#

# Set up catkin workspace directory
instlog "Generating catkin workspace (If needed) at $CATKIN_DIR"
mkdir -p "$CATKIN_DIR/src"
cd "$CATKIN_DIR/src"
catkin_init_workspace
catkin_make -C "$CATKIN_DIR"

# If we're in the Semaphore-ci, we should run catkin_make in the actual build thread
if  $SEMAPHORE; then
    mv ~/Sub8 "$CATKIN_DIR/src"
fi

# Sources the workspace's configurations for bash on this user account
source "$CATKIN_DIR/devel/setup.bash"
if ! cat ~/.bashrc | grep "grep source.*$CATKIN_DIR/devel/setup.bash"; then
    echo "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc
fi

# Check if the sub is set up; if it isn't, set it up
instlog "Looks like you don't have the sub set up, let's do that"
if ! ls "$CATKIN_DIR/src" | grep Sub8; then
    cd "$CATKIN_DIR/src"
    git clone -q https://github.com/uf-mil/Sub8.git
    cd Sub8
    git remote rename origin upstream
    instlog "Make sure you change your git to point to your own fork! (git remote add origin your_forks_url)"
fi

# Install external dependencies with another script
cd $CATKIN_DIR/src
$CATKIN_DIR/src/Sub8/scripts/get_dependencies.sh

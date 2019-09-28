sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-catkin python-catkin-tools
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
sudo apt-get install ros-kinetic-moveit-visual-tools
sudo apt-get install ros-kinetic-industrial-core
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers

#!/bin/bash

apt-get update && apt-get upgrade -y

source /opt/ros/melodic/setup.bash

apt-get install -y git curl wget apt-utils python-pip
apt-get install ros-melodic-rosbridge-server ros-melodic-map-server ros-melodic-cv-bridge

pip install opencv-python==4.2.0.32
mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src && catkin_init_workspace

cd /root/catkin_ws/src && git clone https://github.com/OkDoky/cai_msgs.git -b master
cd /root/catkin_ws/src && git clone https://github.com/OkDoky/moro_server_test.git -b master

apt-get update && rosdep update

cd /root/catkin_ws && rosdep install -y -r --from-paths src --ignore-src

cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE="Release"
source /root/catkin_ws/devel/setup.bash

# add alias for build & source bash
echo "alias sb='source ~/.bashrc'" >> /root/.bashrc
echo "alias cm='cd /root/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE="Release"'" >> /root/.bashrc
echo "alias cs='cd /root/catkin_ws/src'" >> /root/.bashrc
echo "alias cw='cd /root/catkin_ws'" >> /root/.bashrc

# add sources at bash
echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# add variables for ros
echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc
echo "export ROS_HOSTNAME=localhost" >> /root/.bashrc

source /root/.bashrc
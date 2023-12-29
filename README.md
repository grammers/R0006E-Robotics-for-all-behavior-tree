# R0006E-Robotics-for-all-behavior-tree
Code package for the behaviro tree lab in robotics for all.

## Install
The lab is tested is on ubuntu 20.04 and ros2 galactic. Other versions and os should work as.
###
Ros2 galactic folow instrctions at https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html.

Setup source for ros and workspace.
add
source /opt/ros/galactic/setup.bash
source ~/colcon_ws/install/setup.bash
to .bashrc

Create workspace
mkdir colcon_ws
cd colcon_ws
mkdir src
cd src

git clone 
grammers/R0006E-Robotics-for-all-behavior-tree

git clone https://github.com/LTU-RAI/Dsp.git
cd Dsp
git checkout ros2_msg
cd ..


    cd 
    git clone https://github.com/jhu-asco/dsl.git
    cd dsl
    git checkout 61cf588668309e87de209cd95f03a0f792a16c33
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    sudo make install
cd ../../../

sudo apt install ros-galactic-octomap*

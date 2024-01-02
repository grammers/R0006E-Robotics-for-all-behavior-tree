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
cd ../../

git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.cpp
mkdir build
cd build
sudo apt install libzmq3-dev libboost-coroutine-dev libncurses5-dev libncursesw5-dev 
cmake ..
make
sudo make install

cd ../../../

sudo apt install ros-galactic-octomap* ros-galactic-mavros-msgs*


colcon build

### install groot2
at 
https://www.behaviortree.dev/groot/
Downlowd linux installer

cd ~/Downloads/
sudo chmod +x Groot2-v1.5.0-linux-installer.run

## Run
restatr the terminal and run
ros2 launch robotics_for_all_BT_sim rviz_launch.py 

New terminal run
ros2 launch dsp dsp_PAC_launch.py 

New terminal run
ros2 launch dsp octomap_recorder_launch.py 

new terminal run 
ros2 run robotics_for_all_BT_sim drone_sim 
./Groot2-v1.5.0-linux-installer.run 
Follow instructions


new terminal run
ros2 run behavior_tree_ros behavior_tree_ros 

new termiunal run
ros2 launch behavior_tree_ros shafter_take_off_launcher.py

Run Groot

Load the prodject file located at /home/r0006e/colcon_ws/src/R0006E-Robotics-for-all-behavior-tree/behaviorTreeRos/trees

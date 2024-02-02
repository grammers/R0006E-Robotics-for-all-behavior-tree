# R0006E-Robotics-for-all-behavior-tree
Code package for the behavior tree lab in robotics for all.

## Install
The lab is tested on Ubuntu 20.04 and ros2 galactic. Other versions and OS should work as.

### Install soft simulation software
Ros2 galactic follow instructions at [ros.org](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

Set up source for ros and workspace.
add
```
source /opt/ros/galactic/setup.bash
source ~/colcon_ws/install/setup.bash
```
to `.bashrc`

Create workspace
```
mkdir colcon_ws
cd colcon_ws
mkdir src
cd src
```
Clone the used packages
```
git clone https://github.com/grammers/R0006E-Robotics-for-all-behavior-tree.git

git clone https://github.com/LTU-RAI/Dsp.git
cd Dsp
git checkout ros2_msg
cd ..
```

Install some librarys from src
```
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
```
Install some ros package that is needed.
```
sudo apt install ros-galactic-octomap* ros-galactic-mavros-msgs*
```
If you're not using the `galactic` version of ros 2 you need to change galactic in the commands above to the name of the ros version you are using, for example, `ros-iron-octomap*`

Now build the workspace
```
colcon build
```

### install groot2
To install the graphical editor groot2 [download Linux installer](https://www.behaviortree.dev/groot/).

Change the directory to the Downloads folder.
```
cd ~/Downloads/
```
Set the downloaded file to executable.
```
sudo chmod +x Groot2-v1.5.0-linux-installer.run
```
Execute the file.
```
./Groot2-v1.5.0-linux-installer.run
```

## Run
You will need multiple terminals, you can open more terminals `ALT+CTRL+t` or multiple tabs by pressing the '+' sign on top of the terminal window.

restart the terminal and run. To start rviz, the visualization tool to visualize data live in the ros framework.
```
ros2 launch robotics_for_all_BT_sim rviz_launch.py 
```

New terminal run, the D*+ path planner
```
ros2 launch dsp dsp_PAC_launch.py 
```

New terminal run, the map (note that the previous nodes rviz and DSP have to be started before this if you don't see a map in rviz or the DSP window don't print 'Building map' re-run this one.)
```
ros2 launch dsp octomap_recorder_launch.py 
```

new terminal run, the soft simulation software.
```
ros2 run robotics_for_all_BT_sim drone_sim 
```

In a new terminal run, the behavior tree.
```
ros2 launch behavior_tree_ros shafter_take_off_launcher.py
```

To run Groot
```
cd ~/Groot2/bin
./groot2
```


Load the `Project.btproj` file located at /home/r0006e/colcon_ws/src/R0006E-Robotics-for-all-behavior-tree/behaviorTreeRos/trees
Also, load the `behaviorTreeLab.xml` file (the file you should edit and hand in. Remember to change the name of it before handing it in).
Finally, load the `TakeOffTree.xml` and `disarm.xml` if you desire to use them as subtrees.


To send waypoints use `Publish Point` found in the top bar in rviz. Press the `Publish point` once and then on a square in the map to send a waypoint 1m above the clicked point. Do not pres on the top of the map because it will send the drone to the outside of the map and that will cause issues.

## Nots
If you install it on you're machine you will need to change a path in  `behaviorTreeRos/launch/shafter_take_off_launcher.py` on line 16 change tree_path to match the path on your machine.

To stop a program running in a terminal press `CTRL+c`

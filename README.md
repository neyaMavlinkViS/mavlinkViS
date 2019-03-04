#Installing Ros 2:

Link [here](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/)

notes: follow the instructions for setting locale on the link above, but then add the [ROS 2 apt](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/#linux-install-debians-setup-sources) repository to your system before following the rest of the instructions

#After Installing ROS 2

##Building QGroundControl in ROS 2 workspace:

Download QGroundControl directory from bitbucket()  
Place the QGroundControl directory in ~ros2_ws/src/ros2  
To build:         **colcon build --symlink-install --packages-select QGroundControl**  
Before running:         **. install/local_setup.bash**  
To run:         **./install/QGroundControl/bin/QgroundControl**  

##Building ViS in ROS 2 workspace:

Download diux and diux msgs directories from [Bitbucket](http://devtools1.neyasystems.local:7990/projects/DUIX/repos/mavlink_vis/browse)  
Place diux and diuxmsgs directories in ~ros2_ws/src/ros2  
Build diux msgs:        **colcon build --symlink-install --packages-select diux_msgs**  
Run:        **. install/local_setup.bash**  
Build diux:        **colcon build --symlink-install --packages-select diux**  
Run:        **. install/local_setup.bash**  
To run ViS:        **./install/diux/bin/ViS**  




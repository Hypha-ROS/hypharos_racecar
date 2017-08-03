# Hypha ROS RaceCar pkg
  
## About
FB Page: https://www.facebook.com/HyphaROS/  
Website: https://hypharosworkshop.wordpress.com/  
Contact: hypha.ros@gmail.com  
Author: HaoChih, LIN  
Date: 2017/08/04  
LICENSE: BSD  
  
## Hardware 
https://goo.gl/9bx72b  

## Software
### Windows 
64bit RAM >= 8G  
VMware Workstation Playe 12 64bit (http://www.vmware.com/products/player/playerpro-evaluation.html)  
VMWare Image：  
https://drive.google.com/drive/folders/0B-yq9HXd5lDkRGNJUm9KR0h0VEU  

### Ubuntu (16.04) 
64bit RAM > 4G  
Install ROS Kinetic - (Desktop-Full Install)   (http://wiki.ros.org/kinetic/Installation/Ubuntu)  

$ sudo apt-get install remmina synaptic gimp -y  
$ sudo apt-get install ros-kinetic-navigation -y  
$ sudo apt-get install ros-kinetic-hector-slam ros-kinetic-hector-mapping -y  
$ sudo apt-get install arduino ros-kinetic-geographic-msgs -y  
$ sudo apt-get install ros-kinetic-rosserial-arduino -y  
$ sudo apt-get install ros-kinetic-rosserial -y  
$ sudo apt-get install ros-kinetic-slam-gmapping -y  
$ sudo apt-get install ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d -y  

create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/cra-ros-pkg/robot_localization.git  
$ git clone https://github.com/ros-planning/navigation_tutorials.git  
$ git clone https://github.com/MAPIRlab/mapir-ros-pkgs.git  
$ git clone https://github.com/Hypha-ROS/hypha-racecar   
$ cd ..  
$ catkin_make  


# Hypha ROS RaceCar pkg

## Abstract
The MIT RACECAR project provides a fast (~4 m/s) and fully 
autonomous platform for engineering students to easily implement 
control and autonomy related algorithms, however , the total cost of the 
car is around 3,600 USD which is not affordable for general students. 
Hence, we demonstrate a low-cost version that is capable of reaching 
3 m/s but only costs about 600 USD (still onboard, fully autonomous).


## About us
FB Page: https://www.facebook.com/HyphaROS/  
Website: https://hypharosworkshop.wordpress.com/  
Youtube: https://www.youtube.com/watch?v=jgS6gpmWPoY (3 m/s)  
Youku: http://i.youku.com/hypharos  
Contact: hypha.ros@gmail.com  
Author: HaoChih, LIN  
Date: 2017/08/04  
License: LGPL  
  
## Hardware 
[google doc] https://goo.gl/9bx72b  
[baidu doc] http://pan.baidu.com/s/1c2ImUrQ  
Total Cost: < 600 USD  

## Software
### Windows 
64bit RAM >= 8G  
VMware Workstation Playe 12 64bit (http://www.vmware.com/products/player/playerpro-evaluation.html)  
VMWare Image：  
https://drive.google.com/drive/folders/0B-yq9HXd5lDkRGNJUm9KR0h0VEU  

### Ubuntu (16.04) 
64bit RAM > 4G  
Install ROS Kinetic - (Desktop-Full Install)   (http://wiki.ros.org/kinetic/Installation/Ubuntu)  

$ sudo apt-get install git ros-kinetic-rplidar-ros -y  
$ sudo apt-get install remmina synaptic gimp -y  
$ sudo apt-get install ros-kinetic-navigation -y  
$ sudo apt-get install ros-kinetic-hector-slam ros-kinetic-hector-mapping -y  
$ sudo apt-get install arduino ros-kinetic-geographic-msgs -y  
$ sudo apt-get install ros-kinetic-rosserial-arduino -y  
$ sudo apt-get install ros-kinetic-rosserial -y  
$ sudo apt-get install ros-kinetic-slam-gmapping -y  
$ sudo apt-get install ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d -y  

OR  
$ sudo apt-get install git ros-kinetic-rplidar-ros remmina synaptic gimp ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-hector-mapping arduino ros-kinetic-geographic-msgs ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-slam-gmapping ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d -y

create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/cra-ros-pkg/robot_localization.git  
$ git clone https://github.com/ros-planning/navigation_tutorials.git  
$ git clone https://github.com/MAPIRlab/mapir-ros-pkgs.git  
$ git clone https://github.com/Hypha-ROS/hypha-racecar   
$ cd ..  
$ catkin_make  

### Odroid
Image file for odroid.(with SD card >16G)  
[Google drive] https://drive.google.com/open?id=0B7f-a3PiitSec3RyQWRTQVZ1NzA


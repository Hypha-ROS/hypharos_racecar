# HyphaROS RaceCar pkg
![alt text](https://github.com/Hypha-ROS/hypha-racecar/blob/ver.1-release/document/logo/HyphaROS_logo_2.png)  

## Abstract
We present a low-cost (~600USD), high speed, fully autonomous 1/10 rc car platform,
called Hypha RaceCar. Instead of using high-end sensors/SBC like: hokuyo laser, 
Nvidia TX1, etc., our platform replace all expensive components with low-cost product 
(Odroid XU4, RPLidar A1, GY85 ...). The performance of high-end racecar, e.g. MIT RaceCar,
 is around 4 m/s (~3600USD), in contrast, Hypha RaceCar is able to achieve 3 m/s !   
[Official Slides] https://github.com/Hypha-ROS/hypha-racecar/blob/ver.1-release/document/HyphaROS_RaceCar_Project_released.pdf  
[3m/s Video] https://www.youtube.com/watch?v=jgS6gpmWPoY  
[![alt text](http://img.youtube.com/vi/jgS6gpmWPoY/0.jpg)](https://www.youtube.com/watch?v=jgS6gpmWPoY)  
 
## For Beginners
A step by step tutorial from hardware assembling to software installing and opration now is available:  
https://github.com/Hypha-ROS/hypha-racecar/blob/ver.1-release/document/Hypha-ROS-RaceCar-Tutorial_V1.1.pdf 

## About us
FB Page: https://www.facebook.com/HyphaROS/  
Website: https://hypharosworkshop.wordpress.com/  
Youtube: https://www.youtube.com/watch?v=jgS6gpmWPoY (3 m/s)  
Youku: http://i.youku.com/hypharos  
Contact: hypha.ros@gmail.com  
Developer:   
* HaoChih, LIN  
* KaiChun, Wu  

Date: 2017/09/29  
License: LGPL  

## Features
* onboard mapping (ICP, Gmapping)  
* EKF for odometry (Laser Odom + IMU, robot_localization pkg)  
* AMCL localization  
* L1 base controller  
* Dynamic obstacle avoidance  

## Roadmap
* Hypha RaceCar ver.2   
* Teb_local_planner  
* Autonomous parking  
* SVO for odometry estimation  
* Video tutorial  

## Hardware 
![alt text](https://github.com/Hypha-ROS/hypha-racecar/blob/ver.1-release/document/logo/HyphaRaceCar.jpg)  
[google doc] https://goo.gl/9bx72b  
[baidu doc] http://pan.baidu.com/s/1c2ImUrQ  
Total Cost: < 600 USD  

## Software
### Odroid Image
Image file for odroid.(with SD card >16G)  
[Google drive] https://drive.google.com/open?id=0B7f-a3PiitSec3RyQWRTQVZ1NzA  

### Desktop Windows 
64bit RAM >= 8G  
VirtualBox 64bit (https://www.virtualbox.org/wiki/Downloads)  
VirtualBox OVA file：  Will be released soon!

### Desktop Ubuntu (16.04) 
64bit RAM > 4G  
Install ROS Kinetic - (Desktop-Full Install)   (http://wiki.ros.org/kinetic/Installation/Ubuntu)  

$ sudo apt-get install git ros-kinetic-rplidar-ros remmina synaptic gimp ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-hector-mapping arduino ros-kinetic-geographic-msgs ros-kinetic-rosserial-arduino ros-kinetic-rosserial ros-kinetic-slam-gmapping ros-kinetic-mrpt-slam ros-kinetic-mrpt-icp-slam-2d ros-kinetic-robot-localization -y  

create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
$ cd catkin_ws/src  
$ git clone https://github.com/MAPIRlab/rf2o_laser_odometry  
$ git clone https://github.com/Hypha-ROS/hypha-racecar   
$ cd ..  
$ catkin_make  



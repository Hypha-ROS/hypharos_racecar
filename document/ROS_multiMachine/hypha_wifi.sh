#! /bin/bash
IP_ADDRESS_Local=`ip addr show wlan0 | awk '/inet/ {printf $2}' | cut -d/ -f1`
IP_ADDRESS_master="192.168.12.1"

export ROS_MASTER_URI=http://$IP_ADDRESS_master:11311
export ROS_HOSTNAMW=$IP_ADDRESS_Local
export ROS_IP=$IP_ADDRESS_Local

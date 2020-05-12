## About

This is a ROS package that helps integrate the batmon into ROS
It contains a ROS node that communicates with the Batmon via I2C 
and then publishes the information to /BatteryState_{Serial Number}
where {Serial Number} is the s/n of the device (i.e 11021)

## Install

For information reguarding installing the ROS package on a raspberry pi,
please reference the ros wiki:
https://wiki.ros.org/batmon-ros

## About

This is a ROS package that helps integrate the batmon into ROS
It contains a ROS node that communicates with the Batmon via I2C
and then publishes the information to /BatteryState_{Serial Number}
where {Serial Number} is the s/n of the device (i.e 11021)

## Install

#### OS Installation
This package was installed and tested on a Raspberry Pi 3
with Ubiquity's Ubuntu 16.04 image installed. 

The download for that
can be found here: https://downloads.ubiquityrobotics.com/pi.html

You may need to run this to turn off some of its scripts that autostart.

```sudo systemctl disable magni-base```

#### I2C software setup on the pi

Once the image is installed on the pi and it connected to the
internet. 

Run these commands to install I2C packages:

```sudo apt install i2c-tools```

```sudo apt-get install libi2c-dev```

Now by default the pi will have an I2C port at /dev/i2c-1. 

This port does not allow for clock stretching, 

so we need to disable it and then
 create a new one on /dev/i2c-3

 To disable the port run

 ```sudo raspi-config```

 A menu should appear, select option 3 for Interfacing Options

![Alt text](readme_images/menu1.png?raw=true "Menu1")

 On the new menu select P4 for I2C

![Alt text](readme_images/menu1.png?raw=true "Menu2")

 Then select No, and then Finish to exit.

 Now we need to add some commands to /boot/config.txt, run:

 ```sudo nano /boot/config.txt```

 Add the following line to the bottom of it to set the i2c bus speed to 100khz.

 ```dtparam=i2c_baudrate=100000```

 The I2C clock line is linked with the VPU core on the Pi, who's frequency
  can change with load. This can cause issues with communication so add the
   following line to set the core to a fixed frequency:
 
  ```core_freq=250```
 
  Now to create a new I2C port on /dev/i2c-3 add:
 
  ```dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=02,i2c_gpio_scl=03```
 
  By now you should have added three lines to the bottom of /boot/config.txt and it should look something like this. 

![Alt text](readme_images/boot.png?raw=true "Boot")
  Save the changes to the document and reboot.
 
  `sudo reboot now`
 
#### Hardware Setup for the Raspberry Pi
 
  Now connect the Batmon into the pins on the raspberry pi.
 
  The SDA line should be connected to GPIO3 and the SCL line should be plugged into GPIO5 on the board.
 
  Do not forget to add ground to one of the several ground pins on the board, GPIO9 works.
  Here is a reference for the pinout of the Raspberry Pi: https://pinout.xyz/#

![Alt text](readme_images/pinout.png?raw=true "Pinout") 
 
  Make sure those are connected to the correct lines on the batmon board. The pcb should
  have the output lines for I2C labeled.

![Alt text](readme_images/batmon.png?raw=true "Batmon") 
 
  If the Pi was hooked up correctly and the batmon is powered then running
  the following command should return a matrix of addresses.
 
  `i2cdetect -y 3`

![Alt text](readme_images/matrix.png?raw=true "Matrix") 
 
  Note depending on your batmon version the address could be either 0x0E or 0x0B.
 
  If you saw an address in the resulting matrix then the hardware was hooked up correctly
 
#### Ros Setup on the Pi
Ubiquity's image comes preinstalled with ROS, to install the batmon package clone it from this repo.

First change your directory, to the one in which you need to clone it to.

`cd catkin_ws/src/`

Then clone the GitHub repo:

```git clone https://github.com/rotoye/batmon-ros.git raspi_batmon```

Now edit /include/raspi_batmon/smbus.h to ensure your ROS node is reading from the correct address.

```nano ~/catkin_ws/src/raspi_batmon/include/raspi_batmon/smbus.h```

Edit the define for I2CADDRESS3 to match the address that appeared in ```i2cdetect -y 3```

Now if you start the node the data from the batmon should be published in /BatteryState_{SerialNumber}

e.x. /BatteryState_00001

You can start the node with ``rosrun raspi_batmon i2c_reader``

In a separate terminal, you can find the messages being published with `rostopic list`

and then you can see the data in the message with ``rostopic echo /BatteryState_{SerialNumber}``

i.e. ```rostopic echo /BatteryState_00001```

It should be displaying data similar to this: 

![Alt text](readme_images/ros.png?raw=true "Ros") 



# WARNING

This driver contains breaking changes compared to the original VESC driver. The main change is related to using RAD/S when handling speed commands. Please remember this and use with care!!!

# This is a copy of a MIT VESC repo

The code you can see here is taken from the pull request:
https://github.com/mit-racecar/vesc/pull/13 that adds support for firmware 3.38.

Remote from the pull request: git@github.com:erwincoumans/vesc.git

Please try to check it every now and then to check for bugfixes etc.

## Check this maybe
https://github.com/sbgisen/vesc


### Note
The `roslaunch` which executes *does not have its user's group memberships*. This means that it will not have access to serial ports with the `dialout` group, or locations in `/var/log` owned by root, etc. Any filesystem resources needed by your ROS nodes should be chowned to the same unprivileged user which will run ROS, or should set to world readable/writeable, for example using udev. 


# Upload firmware onto vesc
This guide is based on [http://vedder.se/2015/01/vesc-open-source-esc/]
Install toolchain to compile the firmware:
```
sudo apt-get remove binutils-arm-none-eabi gcc-arm-none-eabi
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update`
sudo apt-get install gcc-arm-none-eabi=4.9.3.2015q3-1trusty1
```

Install other dependencies
`sudo apt-get install build-essential  openocd git libudev-dev libqt5serialport5-dev`

Add user to the dialout group:
`sudo adduser $USER dialout`

Add udev rules to allow access to the programmer:
`wget vedder.se/Temp/49-stlinkv2.rules`
`sudo mv 49-stlinkv2.rules /etc/udev/rules.d/`

Restart pc

Download, compile and upload the bootloader
```
mkdir BLDC
cd BLDC
git clone https://github.com/vedderb/bldc-bootloader.git bldc-bootloader
cd bldc-bootloader
make upload
```

Download, compile and upload the firmware
```
cd ..
git clone https://github.com/vedderb/bldc.git bldc-firmware
cd bldc-firmware
```
Modify 'conf_general.h' to reflect the correct hardware version of the vesc
`make upload`


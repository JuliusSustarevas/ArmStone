# ArmStone

## as_bringup
Contains the as1 and as2 launch files. The two files are intended to be launched on the two intel nucs.
* as1 connects to the motors
* as1 launches the control stack ( motors and arm)
* as2 runs localisation, connects to lidars etc
* **Dont forget** udev rules on as1. See `as_bringup/config/udev`. The devices are named, so its important its all setup
 
## as_control
* **Remember** to initialise mecanum drive controller and vesc driver submodules
* hardware interface is setup so that it merges the motor and arm hw interfaces into a combined one. 

## as_description
* Pretty modular urdf setup
* robots folder contains all the entry points
* Otherwise I recommend just looking through how components are setup by reading the urdf folder contents.

## as_localisation
* has some launch files for slam_toolbox and some residue from experimenting with end-effector held realsense l515

## as_xarm
* stores the submodule for xarm drivers
  
## Original software setup
- Wifi Armstone5G
  - armstone@ArmStone1 (192.168.1.101) 
  - armstone@ArmStone2 (192.168.1.102)
- OS on nucs is u20
- docker container runs ubuntu 18 (for running ros melodic)
- DC control box IP 192.168.1.212 (For use with xarm studio)
- Setup Sofware
  - Start the docker container and open shell into it
  - git clone https://github.com/JuliusSustarevas/ArmStone.git
  - git submodule update --recursive
  - git submodule update --init --recursive
  - (in `~/armstone`) catkin build
  - (in `~/armstone`) source devel/setup.bash
  - ... Things are already setup till here in the `~/armstone` folder
  - edit as_bringup as1.launch to enable the controls/modes you want
    - Remember to turn arm on/off
  - roslaunch as_bringup as1.launch

### xARM
Note that xarm payload is calibrated for  Armstone Extruder and you should recalibrate as needed via xarm firmware software. 

### Docker
Armstone uses persistent docker containers. I.e. there is one docker container that is started/stopped as needed but its the same container. 
Original build and launch comments for the container are in as_docker

### bashrc
```
  export ROS_IP=192.168.1.101
  export ROS_MASTER_URI=http://192.168.1.101:11311  
  source /opt/ros/melodic/setup.bash  
  echo "CPU MODE:"
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor  
  #echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```
  

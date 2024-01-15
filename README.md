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
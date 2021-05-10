#include <iostream>
#include <ros/ros.h>
#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "combo_control_node");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    ROS_WARN("Attempting to load CombinedRobotHW");
    combined_robot_hw::CombinedRobotHW hw;
    ROS_WARN("CombinedRobotHW loaded");
    bool init_success = hw.init(nh,nh);

    ROS_WARN("Attempting to load controller manager");
    controller_manager::ControllerManager cm(&hw,nh);
    ROS_WARN("Controller manager loaded");

    ros::Rate rate(100); // 100Hz update rate

    ROS_INFO("multi_vesc_hw_interface started");
    while(ros::ok()){
        hw.read(ros::Time::now(), rate.expectedCycleTime());
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        hw.write(ros::Time::now(), rate.expectedCycleTime());
        rate.sleep();
    }

    spinner.stop();
return 0;
}
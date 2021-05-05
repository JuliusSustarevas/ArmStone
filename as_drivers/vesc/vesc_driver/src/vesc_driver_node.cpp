#include <ros/ros.h>

#include "vesc_driver/vesc_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_driver_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_driver::VescDriver vesc_driver(nh, private_nh);

  ros::Rate loop_rate(50);
  
  while (ros::ok())
  {
    //hack to keep time consistent even if it jumps back and forth
    //created an issue https://github.com/ros/ros_comm/issues/1558
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

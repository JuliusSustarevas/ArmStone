#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    pass
    # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")
    rospy.sleep(5);
    rospy.loginfo("Looking for client")
    # client = dynamic_reconfigure.client.Client("/as_sensors/camera/realsense2_camera_manager", timeout=5, config_callback=callback)
    # client = dynamic_reconfigure.client.Client("/as_sensors/camera/realsense2_camera", timeout=5, config_callback=callback)
    # client = dynamic_reconfigure.client.Client("/as_sensors/camera", timeout=5, config_callback=callback)
    client = dynamic_reconfigure.client.Client("/as_sensors/camera/l500_depth_sensor", timeout=30, config_callback=callback)
    rospy.loginfo("Connected to client")
    client.update_configuration({"visual_preset":5})
    # client.update_configuration({"visual_preset":4})
    rospy.loginfo("sent reconfigure")
    
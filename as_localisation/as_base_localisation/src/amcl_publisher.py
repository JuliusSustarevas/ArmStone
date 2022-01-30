#!/usr/bin/python
import rospy
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

map_frame="map"
odom_frame="odom"
robot_frame="base_link_footprint"


covariance=[]

def get_covar(msg):
    global covariance
    covariance=msg.pose.covariance


if __name__ == "__main__":
    rospy.init_node('tasker')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pwcs_pub = rospy.Publisher(
        "map_correction", PoseWithCovarianceStamped, queue_size=2)
    rate=rospy.Rate(10);
    sub=rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, get_covar)
    
    pose=PoseWithCovarianceStamped()
    pose.header.frame_id=odom_frame   
    global covariance 

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(map_frame, robot_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        if len(covariance)>0:
            pose.pose.covariance=covariance        
            pose.pose.pose.position.x=trans.transform.translation.x
            pose.pose.pose.position.y=trans.transform.translation.y
            pose.pose.pose.position.z=trans.transform.translation.z
            pose.pose.pose.orientation.x=trans.transform.rotation.x
            pose.pose.pose.orientation.y=trans.transform.rotation.y
            pose.pose.pose.orientation.z=trans.transform.rotation.z
            pose.pose.pose.orientation.w=trans.transform.rotation.w
            pwcs_pub.publish(pose)
        rate.sleep()

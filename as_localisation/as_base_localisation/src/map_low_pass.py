#!/usr/bin/python
import rospy
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from slam_toolbox_msgs.srv import Pause
from std_msgs.msg import Float64


# /slam_toolbox/pause_new_measurements

#     (roll, pitch, yaw) = euler_from_quaternion(transform.transform.orientation.x,
# AttributeError: 'Transform' object has no attribute 'orientation'

class MovingAvg():
    def __init__(self):
        self.fake_map_frame = "map"
        self.map_frame = "ma_map"
        self.odom_frame = "odom"
        self.filter_len_s = 3
        self.hz = 20
        self.values = []
        self.transform = TransformStamped()
        self.transform.header.frame_id = self.odom_frame 
        self.transform.child_frame_id = self.map_frame

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(self.hz)

    def add_new(self, transform):
        # convert to xyz rpy and store up to length.

        (roll, pitch, yaw) = euler_from_quaternion([transform.transform.rotation.x,
                                                   transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        new_value = [transform.transform.translation.x, transform.transform.translation.y,
                     transform.transform.translation.z, roll, pitch, yaw]
        if abs(roll) > 1 or abs(pitch) > 1 or abs(yaw) > 1:
            # this is just so i keep an eye out. cuz if it begins to flip to pi the interpolations will be wrong.
            rospy.logerr("MAP --> ODOM rotaitons are high!! ")

        if len(self.values) == self.hz*self.filter_len_s:
            self.values.pop(-1)  # kick last only if full
        self.values.insert(0, new_value)

    def get_avg(self):
        avg = [0]*6
        for val in self.values:
            for ii in range(6):
                avg[ii] += val[ii]/(self.hz*self.filter_len_s)
        return avg

    def xyzrpyToTransform(self, xyzrpy):

        q = quaternion_from_euler(*xyzrpy[3:])
        self.transform.header.stamp = rospy.Time.now()
        self.transform.transform.translation.x = xyzrpy[0]
        self.transform.transform.translation.y = xyzrpy[1]
        self.transform.transform.translation.z = xyzrpy[2]
        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]

        return self.transform

    def run(self):
        while not rospy.is_shutdown():
            try:
                    trans = self.tfBuffer.lookup_transform(
                       self.odom_frame , self.fake_map_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            self.add_new(trans)

            if len(self.values) == self.hz*self.filter_len_s:
                new_trans = self.xyzrpyToTransform(self.get_avg())
                self.broadcaster.sendTransform(new_trans)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('tasker')

    rospy.wait_for_service('/slam_toolbox/pause_new_measurements')
    pause_service = rospy.ServiceProxy('/slam_toolbox/pause_new_measurements', Pause)
    try:
       resp1    = pause_service()
    except rospy.ServiceException as exc:
        pass
    rospy.logwarn("Slam_toolbox exept new measuremtns set to: {}".format(resp1.status))

    movavg = MovingAvg()
    movavg.run()

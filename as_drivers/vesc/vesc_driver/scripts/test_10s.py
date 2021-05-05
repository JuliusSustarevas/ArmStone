#!/usr/bin/env python

import rospy
import math
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class MotorTester():
  def __init__(self, target_vel):
    self.last_speed = 0
    self.ramp_rate = 0.3
    self.prev_t = None
    self.duration = 10
    self.target_vel = target_vel
    self.test_finished = False
    self.test_started = False
    self.velocities = []
    self.positions = []
    self.start_time = None
    self.err = 0.005
    motor_topic = self.get_motor_topic()
    print(motor_topic)
    self.speed_pub = rospy.Publisher(motor_topic, Float64, queue_size=1)
    rospy.Subscriber("/vesc_driver/joint_state", JointState, self.joint_callback)

  def get_motor_topic(self):
    tp=rospy.get_published_topics()
    sensor_topic = "" # Always published
    node_name = ""
    for topic, msg_type in tp:
      if "/sensors" in topic:
        sensor_topic = topic
        break
    if sensor_topic == "":
      raise ValueError("Could not find a sensor topic. Is vesc_driver running?")
      rospy.shutdown()
    for elem in sensor_topic.split("/"):
      if "vesc_driver_node" in elem:
        node_name = elem
        break
    if node_name == "":
      raise ValueError("Could not find a correct vesc_node name name.")
      rospy.shutdown()

    motor_topic = "/vesc_driver/"+node_name+"/motor/speed_rads"
    return motor_topic

  def joint_callback(self, msg):
    # print("CALLBACK")

    if( abs(self.target_vel - msg.velocity[0]) <=  self.err and not self.test_started):
      self.test_started = True
      self.start_time = rospy.Time.now()
      print("STARTED")

    if(self.test_started and not self.test_finished):
      self.velocities.append(msg.velocity[0])
      self.positions.append(msg.position[0])

  def ramped_vel(self, v_prev, v_target, t_prev, t_now, ramp_rate):
    # compute maximum velocity step
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev)
    if error < step: # we can get there within this timestep. we're done.
      return v_target
    else:
      return v_prev + sign * step  # take a step towards the target

  def loop(self):
    rate = rospy.Rate(50)
    while not self.test_finished and not rospy.is_shutdown():
      now = rospy.Time.now()
      if(self.prev_t is None):
        self.prev_t = now
        continue
      if(self.start_time is not None):
        if((now-self.start_time).to_sec() >= self.duration):
          self.test_finished = True
          break
      v = self.ramped_vel(self.last_speed, self.target_vel, self.prev_t, now, self.ramp_rate)
      # print(v)
      self.prev_t = now
      self.last_speed = v
      msg = Float64()
      msg.data = v
      self.speed_pub.publish(msg)
      rate.sleep()
    #  After test finished
    mean = numpy.mean(self.velocities)
    print("MEAN SPEED [rad/s]:", mean)
    rad_diff = self.positions[len(self.positions) -1] - self.positions[0]
    print("POSITION CHANGE [rad]:", rad_diff)


if __name__ == '__main__':
  try:
    rospy.init_node('motor_tester', anonymous=False)
    tester = MotorTester(-1.5)
    tester.loop()
  except rospy.ROSInterruptException:
    pass

#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math

# TODO: modify these constants to make the car follow walls smoothly.
KP = rospy.get_param("/wall_following_pid/kp",-60.0)
KD = rospy.get_param("/wall_following_pid/kd",0.0)

# Set speed thresholds and speed values
MAX_SPEED = rospy.get_param("/wall_following_pid/max_speed",1.5)
MED_SPEED = rospy.get_param("/wall_following_pid/med_speed",1.0)
MIN_SPEED = rospy.get_param("/wall_following_pid/min_speed",0.5)
MAX_SPEED_THRESH = rospy.get_param("/wall_following_pid/max_speed_threshold",10*np.pi/180)
MED_SPEED_THRESH = rospy.get_param("/wall_following_pid/med_speed_threshold",20*np.pi/180)

# Persistent variables to hold previous error and time values for
# derivative calculation
previous_error = 0.0
previous_time = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Speed control algorithm based on steering angle.
# steer_angle: steer tire angle computed by PID controller, radians
# Outputs velocity in meters per second
def calculate_velocity(steer_angle):
  # Follow the step-like velocity thresholds as defined in the lab writeup
  if abs(steer_angle) < MAX_SPEED_THRESH:
    return MAX_SPEED
  elif abs(steer_angle) < MED_SPEED_THRESH:
    return MED_SPEED
  else:
    return MIN_SPEED

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):
  global previous_error
  global previous_time
  # Extract error and get current time for derivative calculation
  error = msg.data
  time = rospy.Time.now().to_sec()
  # Don't calculate derror_dt if no previous value was saved
  if previous_time == 0.0:
    derror_dt = 0
  else:
    derror_dt = (error - previous_error)/(time - previous_time)

  # Use PD control to determine steer tire angle
  pid_output = KP*error + KD*derror_dt
  angle = math.radians(pid_output)    #convert the angle to radians if not already in radians
  angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

  # Calculate velocity based on steer tire angle
  vel = calculate_velocity(angle)

  # Save error and time for next iteration
  previous_error = error
  previous_time = time

  msg = drive_param()
  msg.velocity = vel
  msg.angle = angle
  pub.publish(msg)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
  rospy.init_node('pid_controller_node', anonymous=True)
  rospy.Subscriber("pid_error", Float64, control_callback)
  rospy.spin()


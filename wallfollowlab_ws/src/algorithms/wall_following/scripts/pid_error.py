#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
ANGLE_INCREMENT = 0.00436515314505*180/math.pi
LOOKAHEAD = 0.5
THETA = 45
DESIRED_DISTANCE = 1.0

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
  # Make sure the input angle is within acceptable range
  if angle > MAX_ANGLE:
    angle = MAX_ANGLE
  elif angle < MIN_ANGLE:
    angle = MIN_ANGLE

  # Find the index where we should be looking in the LIDAR data for the
  # given angle
  offset_angle = angle - MIN_ANGLE
  range_idx = int(round(offset_angle/ANGLE_INCREMENT))
  # Make sure the index is actually within the length of the LIDAR ranges
  if range_idx < 0:
    range_idx = 0
  elif range_idx >= len(data.ranges):
    range_idx = len(data.ranges) - 1

  # Return the value at the given angle
  value = data.ranges[range_idx]
  if value > MAX_DISTANCE:
    value = MAX_DISTANCE
  elif value < MIN_DISTANCE:
    value = MIN_DISTANCE

  return value

# a: distance from LIDAR to wall at angle theta
# b: distance from LIDAR to wall at "zero" angle (depending on side)
# theta: angle between distances a and b, in degrees
# Outputs the angle alpha in degrees between the car's x-axis and the wall
def getAlpha(a,b,theta):
  # Calculate numerator and denominator in inverse tangent quantity
  num = a*np.cos(np.radians(theta)) - b
  den = a*np.sin(np.radians(theta))
  # Return alpha as defined by Equation (1) in the lab writeup
  return np.degrees(np.arctan(num/den))

# a: distance from LIDAR to wall at angle theta
# b: distance form LIDAR to wall at "zero" angle (depending on side)
# theta: angle between distances a and b, in degrees
# lookahead_distance: projected lookahead distance of car
# Output: D_t+1, or the future-projected distance from the wall
def getFutureDistance(a,b,theta,lookahead_dist):
  # Get the angle from the car's x-axis to the wall
  alpha = getAlpha(a,b,theta)
  # Return the estimated future distance
  return b*np.cos(np.radians(alpha)) + lookahead_dist*np.sin(np.radians(alpha))

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  # Length b will be found from the reading at 180 degrees
  # while length a is found at a reading (180-theta) degrees
  b = getRange(data,180)
  a = getRange(data,180-THETA)
  # Get the future-projected distance from the wall
  D_future = getFutureDistance(a,b,THETA,LOOKAHEAD)
  # Error for following on left is (desired_distance - D_future) such that
  # positive error encourages turning the steer tire right, or away from
  # the wall
  rospy.loginfo("Left reading: %f, Projected distance: %f", b, D_future)
  return desired_distance - D_future

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  # Length b will be found from the reading at 0 degrees
  # while length a is found at a reading theta degrees
  b = getRange(data,0)
  a = getRange(data,THETA)
  # Get the future-projected distance from the wall
  D_future = getFutureDistance(a,b,THETA,LOOKAHEAD)
  # Error for following on right is (D_future - desired_distance) such that
  # positive error encourages turning the steer tire right, or toward the
  # wall
  rospy.loginfo("Right reading: %f, Projected distance: %f", b, D_future)
  return D_future - desired_distance

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  # Determine distance from left wall
  b_left = getRange(data,180)
  a_left = getRange(data,180-THETA)
  D_left = getFutureDistance(a_left,b_left,THETA,LOOKAHEAD)

  # Determine distance from right wall
  b_right = getRange(data,0)
  a_right = getRange(data,THETA)
  D_right = getFutureDistance(a_right,b_right,THETA,LOOKAHEAD)

  # Reference is the average of these two values
  desired_distance = (D_left + D_right)/2.0

  # Error for center following is (D_right - desired_distance) such that
  # positive error encourages turning the steer tire right, or toward the
  # right wall
  rospy.loginfo("D_left: %f, D_right: %f",D_left, D_right)
  return D_right - desired_distance

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
  error = followCenter(data)

  msg = Float64()
  msg.data = error
  pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_error_node', anonymous = True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

#!/usr/bin/env python

"""
borrowed/ adapted from 
https://github.com/PacktPublishing/Learning-Robotics-using-Python-Second-Edition/blob/master/chapter_8_code/chefbot_bringup/scripts/diff_tf.py
"""
import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int64

# robot specifications
wheel_base_width = .26  # meters
ticks_per_rev = 800  # right now this is our best guess / estimate
wheel_diameter = 0.09  # meters
ticks_per_meter =  ticks_per_rev / (pi *wheel_diameter)  # TODO number of encoder ticks in one meter
max_ticks = 4294967296  # value set in tivac.h


# callbacks 

def lwheelCallback(msg):
    global left_ticks
    left_ticks = msg.data
    

def rwheelCallback(msg):
    global right_ticks
    right_ticks = msg.data 
   


# helper methods

def encoder_difference(curr_ticks, prev_ticks):
    """
    computes the change in ticks since the last time ticks were counted, accounting for wraparound
    """
    delta_ticks = curr_ticks - prev_ticks
    if abs(delta_ticks) > max_ticks:  # odds are the encoder ticks wrapped around with a difference this large.
        if curr_ticks > prev_ticks:  # wrap from low to high -> negative tick change
            ext_ticks = -max_ticks - (max_ticks - curr_ticks)  # ext_ticks = "extended ticks", a tick count that exceeds max or -max ticks
        else:                        # wrap from high to low -> positive tick change
            ext_ticks = max_ticks + (curr_ticks + max_ticks) 
        delta_ticks = ext_ticks - prev_ticks
    return delta_ticks
    
rospy.init_node("odom_core")

rospy.Subscriber("encoder_left", Int64, lwheelCallback)
rospy.Subscriber("encoder_right", Int64, rwheelCallback)
odomPub = rospy.Publisher("odom", Odometry,queue_size=10)
odomBroadcaster = TransformBroadcaster()

# global vars
left_ticks, right_ticks, left_ticks_last, right_ticks_last = 0, 0, None, None
delta_left, delta_right, delta_theta = 0, 0, 0
left_delta_ticks, right_delta_ticks = 0, 0
x, y, z = 0, 0, 0
rate = rospy.Rate(10)
then = rospy.Time.now().to_sec()

while not rospy.is_shutdown():
    rate.sleep()
    # calc change in time
    now = rospy.Time.now()
    elapsed_time = now.to_sec() - then
    then = now.to_sec()
    #elapsed_time = elapsed_time.to_sec()

    # approxomate distance traveled based on change in encoder values
    if left_ticks_last == None or right_ticks_last == None:
        delta_left = 0
        delta_right = 0
        left_ticks_last = left_ticks
        right_ticks_last = right_ticks
    else:
        # compute change in encoders
        left_delta_ticks = encoder_difference(left_ticks, left_ticks_last)
        right_delta_ticks = encoder_difference(right_ticks, right_ticks_last)
        # update previous values to be current values
        left_ticks_last = left_ticks
        right_ticks_last = right_ticks
        # update with values from wheel callbacks
        delta_left = left_delta_ticks / ticks_per_meter  # meters traveled = number of encoder ticks in the last time slice / number of encoder tick in a meter
        delta_right = right_delta_ticks / ticks_per_meter

    distance_traveled = (delta_left + delta_right) / 2  # average distance travelled of the two wheels 
    distance_turned = (delta_right - delta_left) / wheel_base_width  # book says this approximation only works for small angles, so IMU should probably be fused here. result is, somehow, in radians

    # velocity
    vx = distance_traveled / elapsed_time
    vr = distance_turned / elapsed_time

    if not distance_traveled == 0:
        # estimate change in x y position
        dx = cos(distance_turned) * distance_traveled
        dy = -sin(distance_turned) * distance_traveled
        # update x y position
        x = x + ( cos(z) * dx - sin(z) * dy)
        y = y + ( sin(z) * dx + cos(z) * dy)
    if not distance_turned == 0:
        # update z axis orientation
        z = z + distance_turned

    # Publish odom tf
    quat = Quaternion()
    quat.x = 0
    quat.y = 0
    quat.z = sin(z/2)
    quat.w = cos(z/2)
    odomBroadcaster.sendTransform(
        (x, y, 0), 
        (quat.x, quat.y, quat.z, quat.w),
        rospy.Time.now(), 
        'base_footprint',
        'odom'
        )

    # publish odom message
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = quat
    odom.child_frame_id = 'base_footprint'
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = vr
    odomPub.publish(odom)

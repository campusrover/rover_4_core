#!/usr/bin/env python
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
    global left_ticks_last, left_delta_ticks
    ticks = msg.data
    if left_ticks_last == None:
        left_ticks_last = ticks
    left_delta_ticks = ticks - left_ticks_last
    if left_delta_ticks > max_ticks:  # odds are the encoder ticks wrapped around with a difference this large.
        if ticks > left_ticks_last:  # wrap from low to high -> negative
            ext_ticks = -max_ticks - (max_ticks - ticks)
        else:                        # wrap from high to low -> positive
            ext_ticks = max_ticks + (ticks + max_ticks) 
        left_delta_ticks = ext_ticks - left_ticks_last
    left_ticks_last = ticks

def rwheelCallback(msg):
    global right_ticks_last, right_delta_ticks
    ticks = msg.data 
    if right_ticks_last == None:
        right_ticks_last = ticks
    right_delta_ticks = ticks - right_ticks_last
    if left_delta_ticks > max_ticks:  # odds are the encoder ticks wrapped around with a difference this large.
        if ticks > right_ticks_last:  # wrap from low to high -> negative
            ext_ticks = -max_ticks - (max_ticks - ticks)
        else:                        # wrap from high to low -> positive
            ext_ticks = max_ticks + (ticks + max_ticks) 
        right_delta_ticks = ext_ticks - right_ticks_last
    right_ticks_last = ticks
    
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
then = rospy.Time.now()

while not rospy.is_shutdown():
    rate.sleep()
    # calc change in time
    now = rospy.Time.now()
    elapsed_time = now - then
    then = now
    elapsed_time = elapsed_time.to_sec()

    # approxomate distance traveled based on change in encoder values
    if left_ticks_last == None or right_ticks_last == None:
        delta_left = 0
        delta_right = 0
    else:
        # update with values from wheel callbacks
        delta_left = left_delta_ticks / ticks_per_meter
        delta_right = right_delta_ticks / ticks_per_meter

    distance_traveled = (delta_left + delta_right) / 2
    distance_turned = (delta_right - delta_left) / wheel_base_width  # book says this approximation only works for small angles, so IMU should probably be fused here

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

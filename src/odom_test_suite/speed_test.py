#!/usr/bin/env python
"""
a node that forces robot behavior to measure actual maximum linear and rotational speeds through odometry
"""
# ros imports
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# not ros imports
import matplotlib.pyplot as plt 
from pickle import dump
from os.path import dirname, realpath
from os import sep
import sys

def odom_cb(msg):
    global linear_speed, angular_speed
    linear_speed = msg.twist.twist.linear.x
    angular_speed = msg.twist.twist.angular.z

def get_nearby_file(filename):
    return dirname(realpath(sys.argv[0])) + sep + filename

rospy.init_node('speed_test')
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
# tuning 
magic_number = 10
step = 0.1
time = 10
# defaults
linear_speed = 0
angular_speed = 0
linear_data = [[],[]]
angular_data = [[],[]]

rate = rospy.Rate(magic_number)

cmd_instruction = Twist()

# do linear test
while len(linear_data[0]) < magic_number * time:
    linear_data[0].append(linear_speed)
    cmd_instruction.linear.x += step
    linear_data[1].append(cmd_instruction.linear.x)
    cmd_pub.publish(cmd_instruction)
    rate.sleep()

cmd_pub.publish(Twist())
for x in range(magic_number):
    rate.sleep()

cmd_instruction = Twist()
# do angular test
while len(angular_data[0]) < magic_number * time:
    angular_data[0].append(angular_speed)
    cmd_instruction.angular.z += step
    angular_data[1].append(cmd_instruction.angular.z)
    cmd_pub.publish(cmd_instruction)
    rate.sleep()

cmd_pub.publish(Twist())


with open(get_nearby_file('move_data.pickle'), 'wb') as outfile:
    all_data = [linear_data, angular_data]
    dump(all_data, outfile)
# plot results
plt.plot(linear_data[0], 'g+', linear_data[1], 'g--', angular_data[0], 'r+', angular_data[1], 'r--')
plt.show()
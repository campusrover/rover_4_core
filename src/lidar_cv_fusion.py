#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import PointCloud2

rospy.init_node("lidar_cv_fusion")

global previous_cv_pc
previous_cv_pc = None
global previous_lidar_pc
previous_lidar_pc = None

def cv_cb(msg):
    global previous_cv_pc
    previous_cv_pc = msg

def lidar_cb(msg):
    global previous_lidar_pc
    previous_lidar_pc = msg


r = rospy.Rate(1)
rospy.Subscriber("/camera/depth/points", PointCloud2, cv_cb)
rospy.Subscriber("/laser_pointcloud", PointCloud2, lidar_cb)
perception_pub = rospy.Publisher ("/perception_points", PointCloud2, queue_size=10)

def merge_pcs(pc1, pc2):
    print("MERGING")
    # print(type(pc1), type(pc2))
    # print(dir(pc1))

    return None

while (True):

    # published merged pointclouds
    if previous_cv_pc and previous_lidar_pc:
        perception_pub.publish(merge_pcs(previous_cv_pc, previous_lidar_pc))
    
    r.sleep()

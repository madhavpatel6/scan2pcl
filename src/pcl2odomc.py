#!/usr/bin/env python

import rospy
import numpy
import math
import message_filters
import sensor_msgs.point_cloud2 as PCL
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import time

def pointcloud2_to_list(cloud):
    gen = PCL.read_points(cloud, skip_nans=True, field_names=('x', 'y', 'z'))
    list_of_tuples = list(gen)
    return list_of_tuples


def list_to_pointcloud2(points):
    pcloud = PointCloud2()
    pcloud = PCL.create_cloud_xyz32(pcloud.header, points)
    pcloud.header.stamp = rospy.Time.now()
    pcloud.header.frame_id = '/odom'
    return pcloud

new_scan = None
new_odomc = None
is_running = False
pub = None

def callback(odomc, scan):
    #rospy.loginfo('Delta stamp: %ds %dns' % (odomc.header.stamp.secs - scan.header.stamp.secs, odomc.header.stamp.nsecs - scan.header.stamp.nsecs))
    t = [odomc.pose.pose.position.x, odomc.pose.pose.position.y, odomc.pose.pose.position.z]
    q = [odomc.pose.pose.orientation.w, odomc.pose.pose.orientation.x,
         odomc.pose.pose.orientation.y, odomc.pose.pose.orientation.z]
    transformed_scan = transform_scan(pointcloud2_to_list(scan), t, q)
    pub.publish(list_to_pointcloud2(transformed_scan))

def compute_rotation_matrix(q):
    return numpy.matrix([
        [q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2, 2 * (q[1] * q[2] - q[0] * q[3]),
         2 * (q[1] * q[3] + q[0] * q[2])],
        [2 * (q[1] * q[2] + q[0] * q[3]), q[0] ** 2 + q[2] ** 2 - q[1] ** 2 - q[3] ** 2,
         2 * (q[2] * q[3] - q[0] * q[1])],
        [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]),
         q[0] ** 2 + q[3] ** 2 - q[1] ** 2 - q[2] ** 2]
    ])

def transform_scan(scan, t, q):
    new_scan = []
    rotation_matrix = compute_rotation_matrix(q)
    translation_vector = numpy.matrix(t).getT()
    for i in range(len(scan)):
        new_scan.append(list((rotation_matrix * numpy.matrix(scan[i]).getT() + translation_vector).flat))
    return new_scan

def main():
    rospy.init_node('Pcl2odomc')
    odom_sub = message_filters.Subscriber('/odomc', Odometry)
    scan_sub = message_filters.Subscriber('/cloud_out', PointCloud2)
    global pub
    pub = rospy.Publisher('/icp/scan', PointCloud2, queue_size=10)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub], 50, 0.5)
    ts.registerCallback(callback)
    global new_scan
    global new_odomc
    global is_running
    while not rospy.is_shutdown():
        '''if new_scan is not None and new_odomc is not None:
            print('Running:  ' + str(is_running))
            is_running = True
            scan = new_scan
            odomc = new_odomc
            t = [odomc.pose.pose.position.x, odomc.pose.pose.position.y, odomc.pose.pose.position.z]
            q = [odomc.pose.pose.orientation.w, odomc.pose.pose.orientation.x,
                 odomc.pose.pose.orientation.y, odomc.pose.pose.orientation.z]
            transformed_scan = transform_scan(pointcloud2_to_list(scan), t, q)
            pub.publish(list_to_pointcloud2(transformed_scan))
            new_scan = None
            new_odomc = None
            is_running = False
            print('Completed: ' + str(is_running))'''
        pass
if __name__ == "__main__":
    main()
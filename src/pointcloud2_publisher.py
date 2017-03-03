#!/usr/bin/env python
import rospy
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import random
import math

def talker():

    pub = rospy.Publisher('/cloud_out', PointCloud2, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    pcloud = PointCloud2()
    cloud = read_coordinate_file('diamond_shape_3d.txt')
    cloud = translate_2d(cloud, 10, 10)
    cloud = rotate_2d((0,0), cloud, 10)
    pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
    pub.publish(pcloud)

def read_coordinate_file(filename):
    points = []
    try:
        f = open(str(filename), 'r')
    except:
        sys.exit('Check the file name.')
    for line in f:
        if len(line.split()) == 3:
            x, y, z = float(line.split()[0]), float(line.split()[1]), float(line.split()[2])
            points.append([x, y, z])
        elif len(line.split()) == 2:
            x, y = float(line.split()[0]), float(line.split()[1])
            points.append([x, y])
        else:
            sys.exit('File format is incorrect.')
    random.shuffle(points)
    return points

def translate_2d(points, x, y):
    for i in range(len(points)):
        points[i][0] += x
        points[i][1] += y
    return points


def rotate_2d(origin, points, angle):
    ox, oy = origin
    rad_angle = angle*math.pi/180.0

    for i in range(len(points)):
        qx = ox + math.cos(rad_angle) * (points[i][0] - ox) - math.sin(rad_angle) * (points[i][1] - oy)
        qy = oy + math.sin(rad_angle) * (points[i][0] - ox) + math.cos(rad_angle) * (points[i][1] - oy)
        points[i] = [qx, qy, points[i][2]]
    return points


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

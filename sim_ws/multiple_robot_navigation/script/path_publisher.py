#!/usr/bin/python
import os
import platform
import sys
import shutil
import rospy
import sys
import csv
import argparse
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from path_converter import read_robot_path

def main(args):
    rospy.init_node('publicar_pose')
    filename = rospy.get_param("filename")
    robotid = rospy.get_param("robotid")
    metric_path = rospy.get_param("metricpath")
    pose_publisher = rospy.Publisher("robot" + str(robotid) + "_pose_publisher", PoseStamped, queue_size=300)
    path_publisher = rospy.Publisher("robot" + str(robotid) + "_visualization_marker", Marker, queue_size=300)
    rate=rospy.Rate(100)
    time.sleep(2)
    print("Metric path:" + str(metric_path))


    robot_path = read_robot_path(filename)
    if  metric_path == True:
        prev_pose = np.array((float(robot_path[0][0]),float(robot_path[0][1])))
    else:
        prev_pose = np.array((float(robot_path[0][0])*0.05-13.8,-float(robot_path[0][1])*0.05+12.2+15.5,0.0))
    first = True
    while not rospy.is_shutdown():

        count = 0
        if first:
            for i,point in enumerate(robot_path):
                #  print(point)
                #  print(msg.pose.pose.position)
                pose = PoseStamped()


                if  metric_path == True:
                    pose_x = float(point[0])
                    # We need to flip axis due matplotlib-opencv axis notation
                    pose_y = float(point[1])
                else:
                    pose_x = float(point[0])*0.05-13.8
                    # We need to flip axis due matplotlib-opencv axis notation
                    pose_y = -float(point[1])*0.05+12.2+15.5

                pose.header.seq = count
                pose.header.stamp= rospy.Time.now()
                pose.header.frame_id = "/map"
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                pose.pose.position.x = pose_x
                pose.pose.position.y = pose_y
                pose.pose.position.z = 0.0
                count += 1

                #  print("Cantidad de marcadores:", count)
                #  print("publish")
                pose_publisher.publish(pose)


                actual_pose = np.array((pose_x,pose_y,0.0))
                marcador = Marker()
                marcador.header.frame_id = "/map"
                marcador.header.stamp = rospy.Time.now()
                marcador.ns = "marcador_namespace";
                marcador.id = count;
                marcador.type = marcador.LINE_STRIP
                marcador.action = marcador.ADD
                marcador.scale.x = 0.03
                marcador.scale.y = 0.03
                marcador.scale.z = 0.03
                marcador.color.a = 1.0
                marcador.color.r = 0.0
                marcador.color.g = 0.0
                marcador.color.b = 1.0
                marcador.pose.orientation.x = 0.0
                marcador.pose.orientation.y = 0.0
                marcador.pose.orientation.z = 0.0
                marcador.pose.orientation.w = 1.0
                marcador.pose.position.x = 0.0
                marcador.pose.position.y = 0.0
                marcador.pose.position.z = 0.0
                count += 1

                # marker line points
                marcador.points = []
                MARKERS_MAX = 100

                # first point
                first_line_point = Point()
                first_line_point.x = pose_x
                first_line_point.y = pose_y
                first_line_point.z = 0.0
                marcador.points.append(first_line_point)
                # second point
                second_line_point = Point()
                second_line_point.x = prev_pose[0]
                second_line_point.y = prev_pose[1]
                second_line_point.z = 0.0
                marcador.points.append(second_line_point)
                path_publisher.publish(marcador)
                prev_pose = actual_pose
                rate.sleep()
            first = False
            robot_path.reverse()

if __name__ == '__main__':

    main(sys.argv)

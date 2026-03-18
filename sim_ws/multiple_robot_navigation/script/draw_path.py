import os
import platform
import sys
import shutil
import rospy
import sys
import csv
import argparse
import time
import yaml
import numpy as np
import cv2 as cv
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from path_converter import read_robot_path

def main(filename, yamlfile, pgmfile, metric):

    if metric == "0":
        ismetric = False
    else:
        ismetric = True

    robot_count = 2
    image_map = cv.imread(pgmfile)
    cv.imshow('frame',image_map)
    cv.waitKey(0)

    f = open(yamlfile)
    dataMap = yaml.safe_load(f)
    xcoordrobot1 = dataMap["xoriginr1"]
    ycoordrobot1 = dataMap["yoriginr1"]
    xcoordrobot2 = dataMap["xoriginr2"]
    ycoordrobot2 = dataMap["yoriginr2"]
    try:
        xcoordrobot3 = dataMap["xoriginr3"]
        ycoordrobot3 = dataMap["yoriginr3"]
        robot_count = 3
    except:
        print("Robot 3 not found, working with only 2")

    resolution = dataMap["resolution"]
    origin = dataMap["origin"]
    origin_pixels = [0,0]

    print("Resolution:" + str(resolution))
    origin_pixels[0] = origin[0] / resolution
    origin_pixels[1] = origin[1] / resolution
    print(origin_pixels)

    # Green color in BGR
    color = (255, 0, 0)

    # Line thickness
    thickness = 2

    robot_path = read_robot_path(filename)

    if ismetric:
        pose = (int(-origin_pixels[0] + float(robot_path[0][0])/resolution),\
                int(origin_pixels[1])+image_map.shape[0]-int(float(robot_path[0][1])/resolution))
    else:
        pose = [float(robot_path[0][0])/resolution, float(robot_path[0][1])/resolution]

    prev_pose = np.array((pose[0], pose[1]))

    start_point = (int(pose[0]), int(pose[1]))
    for i,point in enumerate(robot_path):
        if ismetric:
            pose = (int(-origin_pixels[0] + float(point[0])/resolution),\
                    int(origin_pixels[1])+image_map.shape[0]-int(float(point[1])/resolution))
        else:
            pose = [ float(point[0])/resolution, float(point[1])/resolution]
        end_point = (int(pose[0]), int(pose[1]))
        image_map = cv.line(image_map, start_point, end_point, color, thickness)
        start_point = end_point

    radius = 10
    center_coordinates = (int(-origin_pixels[0] + xcoordrobot1/resolution),\
                          int(origin_pixels[1])+image_map.shape[0]-int(ycoordrobot1/resolution))
    print(center_coordinates)
    cv.circle(image_map, center_coordinates, radius, color, thickness)

    center_coordinates = (int(-origin_pixels[0] + xcoordrobot2/resolution),\
                          int(origin_pixels[1])+image_map.shape[0]-int(ycoordrobot2/resolution))
    print(center_coordinates)
    cv.circle(image_map, center_coordinates, radius, color, thickness)

    if robot_count == 3:
        center_coordinates = (int(-origin_pixels[0] + xcoordrobot3/resolution),\
                              int(origin_pixels[1])+image_map.shape[0]-int(ycoordrobot3/resolution))
        print(center_coordinates)
        cv.circle(image_map, center_coordinates, radius, color, thickness)

    center_coordinates = (int(-origin_pixels[0] + 0.0),\
                          int(origin_pixels[1])+image_map.shape[0]-int(0.0))
    print(center_coordinates)

    color = (0, 255, 0)
    cv.circle(image_map, center_coordinates, radius, color, thickness)
    cv.imshow('frame',image_map)
    cv.waitKey(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument("-f","--filename", help="Path to roamap plan", default="foo")
    parser.add_argument("-y","--yamlfile", help="Path to map yaml", default="foo")
    parser.add_argument("-p","--pgmfile", help="Path to map image", default="foo")
    parser.add_argument("-m","--metric", help="Path is in metric coords", default="0")

    args = parser.parse_args()

    main(args.filename, args.yamlfile, args.pgmfile, args.metric)

import os
import platform
import sys
import argparse
import shutil
import csv
import random

def read_robot_path(csv_name):
    """Read csv file with the robot's initial position"""
    with open(csv_name, mode='r') as csv_file:
        robot_path = []
        csv_reader = csv.DictReader(csv_file, delimiter='\t')
        for row in csv_reader:
                    robot_path.append([float(row["x"]), float(row["y"]), float(row["z"])])
        return robot_path

# return [[x1,y1,z1,x2,zy2,z2,....],[Y1,Y2,....]]


def read_converted_waypoints(robot_path):

    #--------------------create nav simulation file--------------------
    plan_coords = []
    plan_yaw = []
    for row in robot_path:
        # Plan coords to world convert
        pose_x = float(row[0])*0.05-13.8
        # We need to flip axis due matplotlib-opencv axis notation
        pose_y = -float(row[1])*0.05+12.2+15.5

        plan_coords.append(pose_x)
        plan_coords.append(pose_y)
        plan_coords.append(0.0) # z is hardcoded to 0
        plan_yaw.append(float(row[2]))
    return plan_coords, plan_yaw

def read_waypoints(robot_path):

    #--------------------create nav simulation file--------------------
    plan_coords = []
    plan_yaw = []
    for row in robot_path:
        # Plan coords to world convert
        pose_x = float(row[0])
        pose_y = float(row[1])

        plan_coords.append(pose_x)
        plan_coords.append(pose_y)
        plan_coords.append(0.0) # z is hardcoded to 0
        plan_yaw.append(float(row[2]))
    return plan_coords, plan_yaw

def main():
    robot_path = read_robot_path("../config/robots_path_boxes2/map_boxes2_robot1.txt")
    plan_coords, plan_yaw = read_converted_waypoints(robot_path)
    print(plan_coords)
    print(plan_yaw)

    # Convertir de coordenadas pixel a coordenadas world

if __name__ == "__main__":
    main()

#!/usr/bin/env python

#  Base code from https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md

# Edit: Martin Nievas
#   Some modifications were added to work correctly in the current simulation setup.

import rospy
import math
import argparse
import sys
import time
from math import pow, atan2, sqrt
from path_converter import read_converted_waypoints
from path_converter import read_robot_path
from path_converter import read_waypoints

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self, filename, robotid, ismetric):

        rospy.init_node("move_base" + str(robotid) + "_sequence")
        self.plan_manifest = filename
        self.plan_coords = []
        self.plan_yaw = []
        self.topic = "/robot" + str(robotid) + "/robot" + str(robotid) + "_odom"

        self.pose_subscriber = rospy.Subscriber(self.topic,
                                                Odometry, self.update_pose)
        self.elapsed_time_publisher = rospy.Publisher("robot" + str(robotid) + "_elapsed_time", Float32, queue_size=1)
        self.path_done_publisher = rospy.Publisher("done", Bool, queue_size=1)
	
        self.odom = Odometry()
	
	self.velocity_publisher = rospy.Publisher("/robot"+ str(robotid) +"/cmd_vel", Twist,queue_size=10)
	self.vel_msg = Twist()

        self.robotid = robotid
        self.waypoint_radius = 0.5
        self.inittime = 0.0
        self.endtime = 0.0
        self.elapsed_time = 0.0
        self.done = False

        # Read (x1,y1,z1,x2,y2...) coords and (Y1, Y2,...) yaw angles
        #  self.plan_coords, self.plan_yaw = read_waypoints(self.plan_manifest)
        robot_path = read_robot_path(filename)
        if ismetric == True:
            self.plan_coords, self.plan_yaw = read_waypoints(robot_path)
        else:
            self.plan_coords, self.plan_yaw = read_converted_waypoints(robot_path)

        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0

        for yawangle in self.plan_yaw:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [self.plan_coords[i:i+n] for i in range(0, len(self.plan_coords), n)]
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        #Create action client
        self.move_base_topic = "/robot" + str(robotid) + "/move_base"
        self.client = actionlib.SimpleActionClient(self.move_base_topic, MoveBaseAction)
        wait = True
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        while not wait:
            rospy.loginfo("Reconect...Waiting for move_base action server...")
            wait = self.client.wait_for_server(rospy.Duration(0.5))

        if not wait:
            rospy.logerr("Action server for robot" +str(robotid) +" not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        # Start time measurment
        self.inittime = time.time()
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.odom.pose.pose.position.x = round(self.odom.pose.pose.position.x, 4)
        self.odom.pose.pose.position.y = round(self.odom.pose.pose.position.y, 4)


    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.position.x - self.odom.pose.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.odom.pose.pose.position.y), 2))


    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt)+" received")
        #  rospy.loginfo("pose:"+ str(self.pose_seq[self.goal_cnt].position.x))

        if self.euclidean_distance(self.pose_seq[-1]) <= 3.0 and\
                self.goal_cnt < len(self.pose_seq):
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime)
                rospy.loginfo("Robot "+ str(self.robotid) + " elapsed time: !" + str(self.elapsed_time))
                self.elapsed_time_publisher.publish(self.elapsed_time)
                self.done = True
                self.path_done_publisher.publish(self.done)
                self.goal_cnt = len(self.pose_seq)


        if self.goal_cnt < (len(self.pose_seq) - 1) and \
            self.euclidean_distance(self.pose_seq[self.goal_cnt]) <= self.waypoint_radius:
                rospy.loginfo("Goal pose "+str(self.goal_cnt) + " reached by distance limit")
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime)
                self.elapsed_time_publisher.publish(self.elapsed_time)
                self.path_done_publisher.publish(self.done)
		self.goal_cnt += 1
		contador = self.goal_cnt + 4               
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
		if contador < (len(self.pose_seq)-1) and self.pose_seq[contador] == self.pose_seq[contador+1]:
			time.sleep(2)		
			#self.wait_a_moment()
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
        elif self.goal_cnt == (len(self.pose_seq) - 1) and \
            self.euclidean_distance(self.pose_seq[self.goal_cnt]) <= 0.5:
                rospy.loginfo("Goal pose "+str(self.goal_cnt) + " reached by distance limit")
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime)
                rospy.loginfo("Robot "+ str(self.robotid) + " elapsed time: !" + str(self.elapsed_time))
                self.elapsed_time_publisher.publish(self.elapsed_time)
                self.done = True
                self.path_done_publisher.publish(self.done)
                #  rospy.signal_shutdown("Final goal pose reached!")
                self.goal_cnt += 1
        elif self.goal_cnt == len(self.pose_seq):
            next_goal = MoveBaseGoal()
            next_goal.target_pose.header.frame_id = "map"
            next_goal.target_pose.header.stamp = rospy.Time.now()
            final_pose = Pose()
            final_pose.position.x = -2.0
            final_pose.position.y = -0.5
            final_pose.position.z = 0.0
            final_pose.orientation.x = 0.0
            final_pose.orientation.y = 0.0
            final_pose.orientation.z = 0.0
            final_pose.orientation.w = 1.0

            next_goal.target_pose.pose = final_pose
            rospy.loginfo("Sending 'go away!' goal pose to Action Server")
            self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            self.goal_cnt += 1
        elif self.goal_cnt == (len(self.pose_seq)+1):
            final_pose = Pose()
            final_pose.position.x = -2.0
            final_pose.position.y = -0.5
            final_pose.position.z = 0.0
            final_pose.orientation.x = 0.0
            final_pose.orientation.y = 0.0
            final_pose.orientation.z = 0.0
            final_pose.orientation.w = 1.0
            if self.euclidean_distance(final_pose) <= self.waypoint_radius:
                rospy.signal_shutdown("Final goal pose reached!")
                return


    def done_cb(self, status, result):
        self.goal_cnt += 1
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html

        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                self.stop_robot()
                rospy.signal_shutdown("Final goal pose reached!")

                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()
#nuvo stop para robots
    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z =0.0
        for _in range(5):
            self.velocity_publisher.publish(stop)
            porpy.sleep(0.05)


    def wait_a_moment(self):
	self.vel_msg.linear.x = 0
	self.vel_msg.linear.y = 0
	self.vel_msg.linear.z = 0
	self.vel_msg.angular.x = 0
	self.vel_msg.angular.y = 0
	self.vel_msg.angular.z = 0
	for i in range(1,5):	
		self.velocity_publisher.publish(self.vel_msg)
		time.sleep(1)

def main(args):

    filename = rospy.get_param("filename")
    robotid = rospy.get_param("robotid")
    ismetric = rospy.get_param("metricpath")

    try:
        MoveBaseSeq(filename, robotid, ismetric)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

if __name__ == '__main__':

    main(sys.argv)

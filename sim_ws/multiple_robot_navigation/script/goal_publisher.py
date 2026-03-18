#!/usr/bin/env python

#  Base code from https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md
#  Edited to fix final-goal stopping and other bugs.

import rospy
import math
import sys
import time
from math import sqrt, pow
from path_converter import read_converted_waypoints, read_robot_path, read_waypoints

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from tf.transformations import quaternion_from_euler


class MoveBaseSeq(object):

    def __init__(self, filename, robotid, ismetric):
        rospy.init_node("move_base" + str(robotid) + "_sequence")
        self.robotid = int(robotid)
        self.plan_manifest = filename
        self.waypoint_radius = rospy.get_param("~waypoint_radius", 0.5)

        base = "robot" + str(self.robotid)
        self.topic = "/{0}/{0}_odom".format(base)
        self.pose_subscriber = rospy.Subscriber(self.topic, Odometry, self.update_pose)
        self.elapsed_time_publisher = rospy.Publisher(base + "_elapsed_time", Float32, queue_size=1)
        self.path_done_publisher = rospy.Publisher(base + "_path_done", Bool, queue_size=1)
        self.velocity_publisher = rospy.Publisher("/" + base + "/cmd_vel", Twist, queue_size=10)

        self.odom = None
        self.inittime = 0.0
        self.endtime = 0.0
        self.elapsed_time = 0.0
        self.done = False

        # Read path
        robot_path = read_robot_path(filename)
        if ismetric:
            plan_coords, plan_yaw = read_waypoints(robot_path)
        else:
            plan_coords, plan_yaw = read_converted_waypoints(robot_path)

        # build pose sequence
        n = 3
        points = [plan_coords[i:i+n] for i in range(0, len(plan_coords), n)]
        self.pose_seq = []
        for i, pt in enumerate(points):
            x = float(pt[0])
            y = float(pt[1])
            z = float(pt[2]) if len(pt) > 2 else 0.0
            yaw = 0.0
            if i < len(plan_yaw):
                yaw = float(plan_yaw[i])
            q = Quaternion(*quaternion_from_euler(0, 0, yaw))
            p = Pose(position=Point(x, y, z), orientation=q)
            self.pose_seq.append(p)

        if not self.pose_seq:
            rospy.logwarn("No poses loaded for robot %d" % self.robotid)
            self._publish_done("no_poses")
            return

        # action client (respect namespace)
        self.move_base_topic = rospy.resolve_name("move_base")
        self.client = actionlib.SimpleActionClient(self.move_base_topic, MoveBaseAction)
        wait = True
        rospy.loginfo("Waiting for move_base action server %s ..." % self.move_base_topic)
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        while not wait:
            rospy.logerr("Reconect... Waiting for move_base action server...")
            wait = self.client.wait_for_server(rospy.Duration(5.0))
            
        if not wait:
            rospy.logerr("Action server for robot"+ str(robotid)+"not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        rospy.loginfo("Connected to move_base for robot %d" % self.robotid)
        self.goal_idx = 0
        self.inittime = time.time()
        self._send_goal(self.goal_idx)

    def update_pose(self, data):
        self.odom = data

    def _dist_to(self, pose):
        if self.odom is None:
            return float('inf')
        dx = pose.position.x - self.odom.pose.pose.position.x
        dy = pose.position.y - self.odom.pose.pose.position.y
        return sqrt(dx * dx + dy * dy)

    def _send_goal(self, idx):
        if idx < 0 or idx >= len(self.pose_seq) or self.done:
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[idx]
        self.client.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        rospy.loginfo("robot%d: sent goal %d" % (self.robotid, idx))

    def _active_cb(self):
        rospy.loginfo("robot%d: goal %d active" % (self.robotid, self.goal_idx))

    def _feedback_cb(self, feedback):
        if self.done:
            return
        # check distance to current goal
        if self.goal_idx < len(self.pose_seq):
            dist = self._dist_to(self.pose_seq[self.goal_idx])
            if dist <= self.waypoint_radius:
                rospy.loginfo("robot%d: reached waypoint %d (dist %.3f)" % (self.robotid, self.goal_idx, dist))
                # if last waypoint -> finish and stop
                if self.goal_idx == len(self.pose_seq) - 1:
                    self.endtime = time.time()
                    self.elapsed_time = self.endtime - self.inittime
                    rospy.loginfo("robot%d: final goal reached, elapsed %.2f" % (self.robotid, self.elapsed_time))
                    self.client.cancel_all_goals()
                    self._stop_robot()
                    self._publish_done("success")
                else:
                    # advance to next
                    self.goal_idx += 1
                    self._send_goal(self.goal_idx)

    def _done_cb(self, status, result):
        if self.done:
            return
        # status codes: 3 == SUCCEEDED
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("robot%d: action succeeded for goal %d" % (self.robotid, self.goal_idx))
            # if it was final goal index -> finish
            if self.goal_idx >= len(self.pose_seq) - 1:
                self.endtime = time.time()
                self.elapsed_time = self.endtime - self.inittime
                self._stop_robot()
                self._publish_done("success")
            else:
                # otherwise advance (safety) - some servers increment differently
                self.goal_idx += 1
                self._send_goal(self.goal_idx)
        else:
            rospy.logwarn("robot%d: goal %d finished with status %d" % (self.robotid, self.goal_idx, status))
            self.client.cancel_all_goals()
            self._stop_robot()
            self._publish_done("aborted")

    def _stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        for _ in range(5):
            try:
                if not rospy.is_shutdown():
                    self.velocity_publisher.publish(stop)
                    rospy.sleep(0.05)
            except Exception:
                pass

    def _publish_done(self, reason):
        if self.done:
            return
        self.done = True
        try:
            self.path_done_publisher.publish(True)
            self.elapsed_time_publisher.publish(self.elapsed_time)
        except Exception:
            pass
        rospy.loginfo("robot%d finished: %s, elapsed: %.2f" % (self.robotid, reason, self.elapsed_time))
        # request shutdown of this node
        try:
            rospy.signal_shutdown("finished:%s" % reason)
        except Exception:
            pass


def main(argv):
    filename = rospy.get_param("filename")
    robotid = int(rospy.get_param("robotid"))
    ismetric = rospy.get_param("metricpath")
    try:
        MoveBaseSeq(filename, robotid, ismetric)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)

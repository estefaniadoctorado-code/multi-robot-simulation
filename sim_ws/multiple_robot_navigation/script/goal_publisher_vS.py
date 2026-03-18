#!/usr/bin/env python
import rospy
import actionlib
import time
from math import sqrt, pow
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from path_converter import read_robot_path, read_converted_waypoints, read_waypoints


class MoveBaseSeq(object):
    def __init__(self, filename, robotid, ismetric):
        rospy.init_node("move_base" + str(robotid)+"_sequence")
        self.plan_manifest = filename
        self.robotid = int(robotid)
        self.filename = filename
        self.ismetric = bool(ismetric)

        # Parameters (configurable via rosparam)
        self.waypoint_radius = rospy.get_param("~waypoint_radius", 0.5)
        self.per_goal_timeout = rospy.get_param("~per_goal_timeout", 60.0)   # seconds per waypoint
        self.overall_timeout = rospy.get_param("~overall_timeout", 300.0)   # max time for whole path
        self.check_rate = rospy.get_param("~timeout_check_rate", 1.0)       # Hz

        # State
        self.inittime = 0.0
        self.last_goal_sent_time = None
        self.endtime = 0.0
        self.elapsed_time = 0.0
        self.done = False
        self.result_reason = "not_started"  # success | timeout_per_goal | timeout_overall | aborted | failed

        # Publishers
        base = "robot" + str(self.robotid)
        self.elapsed_time_publisher = rospy.Publisher(base + "_elapsed_time", Float32, queue_size=1)
        # self.path_done_publisher = rospy.Publisher(base + "_path_done", Bool, queue_size=1)
        self.path_done_publisher = rospy.Publisher("done", Bool, queue_siz    e=1)


        self.result_publisher = rospy.Publisher(base + "_result", String, queue_size=1)
        self.velocity_publisher = rospy.Publisher("/" + base + "/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()

        # Odometry subscriber
        self.topic_odom = "/" + base + "/" + base + "_odom"
        # self.odom = None
        self.odom = Odometry()
        self.pose_subscriber = rospy.Subscriber(self.topic_odom, Odometry, self.update_pose)

        # Read path and build poses
        robot_path = read_robot_path(self.filename)
        if self.ismetric:
            self.plan_coords, self.plan_yaw = read_waypoints(robot_path)
        else:
            self.plan_coords, self.plan_yaw = read_converted_waypoints(robot_path)

        self.pose_seq = []
        n = 3
        points = [self.plan_coords[i:i+n] for i in range(0, len(self.plan_coords), n)]
        for i, pt in enumerate(points):
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            pose.position.z = float(pt[2])
            yaw = 0.0
            if i < len(self.plan_yaw):
                yaw = float(self.plan_yaw[i])
            q = quaternion_from_euler(0, 0, yaw)
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.pose_seq.append(pose)

        # Action client (resolve name in namespace)
        # self.move_base_topic = rospy.resolve_name("move_base")
        self.move_base_topic = "/"+ base + "/move_base"
        self.client = actionlib.SimpleActionClient(self.move_base_topic, MoveBaseAction)
        wait = True
        rospy.loginfo("Waiting for move_base action server %s ..." % self.move_base_topic)
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        while not wait:    
            rospy.loginfo("Reconect ...Waiting for move_base action server... %s" % self.move_base_topic)
            wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server for" + base +"not available!")
            rospy.signal_shutdown("Action server not available!")

            # self.result_reason = "no_action_server"
            # self.publish_final_state()
            return

        # Start time measurment

        self.inittime = time.time()
        rospy.loginfo("Connected to move_base server for robot %d" % self.robotid)
        rospy.loginfo("Starting goals achievements ...")
        self.goal_cnt = 0
        if len(self.pose_seq) > 0:
            self.send_goal(self.goal_cnt)
            # Timer to check timeouts
            rospy.Timer(rospy.Duration(1.0 / max(self.check_rate, 0.1)), self._timeout_monitor)
        else:
            rospy.logwarn("No poses loaded for robot %d" % self.robotid)
            self.result_reason = "no_poses"
            self.publish_final_state()

    def update_pose(self, data):
        self.odom = data

    def euclidean_distance(self, goal_pose):
        if self.odom is None:
            return float('inf')
        return sqrt(pow((goal_pose.position.x - self.odom.pose.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.odom.pose.pose.position.y), 2))

    def active_cb(self):
        rospy.loginfo("Goal %d is being processed by Action Server for robot %d" % (self.goal_cnt, self.robotid))

    def feedback_cb(self, feedback):
        # Called frequently by actionlib; use it to check arrival by distance as well
        if self.done:
            return
        if self.goal_cnt < len(self.pose_seq):
            dist = self.euclidean_distance(self.pose_seq[self.goal_cnt])
            if dist <= self.waypoint_radius:
                rospy.loginfo("Goal %d reached (dist %.3f) robot %d" % (self.goal_cnt, dist, self.robotid))
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime)
                self.elapsed_time_publisher.publish(self.elapsed_time)
                self.goal_cnt += 1
                if self.goal_cnt < len(self.pose_seq):
                    self.send_goal(self.goal_cnt)
                else:
                    # success: cancel remaining and stop robot
                    self.result_reason = "success"
                    self.client.cancel_all_goals()
                    self._stop_robot()
                    self.publish_final_state()

    def done_cb(self, status, result):
        if self.done:
            return
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal %d succeeded (status=%d) robot %d" % (self.goal_cnt, status, self.robotid))
            # If last goal succeeded -> success
            if self.goal_cnt >= (len(self.pose_seq) - 1):
                self.result_reason = "success"
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime)
                self._stop_robot()
                self.publish_final_state()
            else:
                # else, advance to next (safety)
                self.goal_cnt += 1
                if self.goal_cnt < len(self.pose_seq):
                    self.send_goal(self.goal_cnt)
        else:
            rospy.logwarn("Goal %d finished with status %d (robot %d)" % (self.goal_cnt, status, self.robotid))
            if status in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                self.result_reason = "aborted"
                self.endtime = time.time()
                self.elapsed_time = float(self.endtime - self.inittime) if self.inittime else 0.0
                self.client.cancel_all_goals()
                self._stop_robot()
                self.publish_final_state()

    def send_goal(self, idx):
        if idx < 0 or idx >= len(self.pose_seq) or self.done:
            return
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[idx]
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        self.last_goal_sent_time = time.time()
        rospy.loginfo("Sent goal %d for robot %d" % (idx, self.robotid))

    def _timeout_monitor(self, event):
        if self.done:
            return
        now = time.time()
        # overall timeout
        if self.inittime and (now - self.inittime) > self.overall_timeout:
            rospy.logwarn("Overall timeout for robot %d exceeded (%.1fs > %.1fs)" % (self.robotid, now - self.inittime, self.overall_timeout))
            self.result_reason = "timeout_overall"
            self.endtime = now
            self.elapsed_time = float(self.endtime - self.inittime)
            self.client.cancel_all_goals()
            self._stop_robot()
            self.publish_final_state()
            return
        # per-goal timeout
        if self.last_goal_sent_time and (now - self.last_goal_sent_time) > self.per_goal_timeout:
            rospy.logwarn("Per-goal timeout for robot %d on goal %d (%.1fs > %.1fs)" % (self.robotid, self.goal_cnt, now - self.last_goal_sent_time, self.per_goal_timeout))
            self.result_reason = "timeout_per_goal"
            self.endtime = now
            self.elapsed_time = float(self.endtime - self.inittime) if self.inittime else 0.0
            self.client.cancel_all_goals()
            self._stop_robot()
            self.publish_final_state()
            return

    def _stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        for _ in range(5):
            self.velocity_publisher.publish(stop)
            rospy.sleep(0.05)

    def publish_final_state(self):
        if self.done:
            return
        self.done = True
        self.path_done_publisher.publish(True)
        self.elapsed_time_publisher.publish(self.elapsed_time)
        self.result_publisher.publish(String(self.result_reason))
        rospy.loginfo("Robot %d finished: %s, elapsed: %.2f" % (self.robotid, self.result_reason, self.elapsed_time))

    def send_final_status_now(self):
        # helper to force publish state (used on early failures)
        self.publish_final_state()

def main():
    # rospy.init_node("move_base_sequence_node", anonymous=True)
    filename = rospy.get_param("filename")
    robotid = rospy.get_param("robotid")
    ismetric = rospy.get_param("metricpath")
    try:
        seq = MoveBaseSeq(filename, robotid, ismetric)
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveBaseSeq interrupted")


if __name__ == '__main__':
    main()

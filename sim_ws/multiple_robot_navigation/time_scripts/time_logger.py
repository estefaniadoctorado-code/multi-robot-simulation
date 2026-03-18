#!/usr/bin/env python
import rospy, time
import datetime

from std_msgs.msg import Float32, Bool



class time_logger():

    def __init__(self):
        self.robot1_times = []
        self.robot2_times = []
        self.robot3_times = []
        self.robot4_times = []
        self.robot1_status_done = False
        self.robot2_status_done = False
        self.robot3_status_done = False
        self.robot4_status_done = False
        self.currentDT = datetime.datetime.now()
        self.filename = str(self.currentDT.strftime("%Y-%m-%d-%H:%M:%S"))
        print("filename: " + self.filename)

    def log_write(self):
        self.f.write()

    def check_and_save(self):
        if self.robot1_status_done == True and self.robot2_status_done == True and self.robot3_status_done == True and self.robot4_status_done == True:
            print("saving logs")
            f = open("../time_logs/" + self.filename,"w")

            f.write("R1: " + str(self.robot1_times[-1]) + "\n")
            f.write("R2: " + str(self.robot2_times[-1]) + "\n")
            f.write("R3: " + str(self.robot3_times[-1]) + "\n")
            f.write("R4: " + str(self.robot4_times[-1]) + "\n")

            f.write("robot1\n")
            for data in self.robot1_times:
                f.write(str(data) + "\n")

            f.write("robot2\n")
            for data in self.robot2_times:
                f.write(str(data) + "\n")

            f.write("robot3\n")
            for data in self.robot3_times:
                f.write(str(data) + "\n")

            f.write("robot4\n")
            for data in self.robot4_times:
                f.write(str(data) + "\n")

            f.close()
            rospy.signal_shutdown("All logs saved")
            return


    def robot1_log(self,msg):
        print("robot1 node "+str(len(self.robot1_times))+" time: " + str(msg.data))
        self.robot1_times.append(float(msg.data))

    def robot2_log(self,msg):
        print("robot2 node "+str(len(self.robot2_times))+" time: " + str(msg.data))
        self.robot2_times.append(float(msg.data))

    def robot3_log(self,msg):
        print("robot3 node "+str(len(self.robot3_times))+" time: " + str(msg.data))
        self.robot3_times.append(float(msg.data))

    def robot4_log(self,msg):
        print("robot4 node "+str(len(self.robot4_times))+" time: " + str(msg.data))
        self.robot4_times.append(float(msg.data))

    def robot1_done(self,msg):
        self.robot1_status_done = msg.data
        if msg.data == True:
            print("Robot1 Done!")
            self.check_and_save()

    def robot2_done(self,msg):
        self.robot2_status_done = msg.data
        if msg.data == True:
            print("Robot2 Done!")
            self.check_and_save()

    def robot3_done(self,msg):
        self.robot3_status_done = msg.data
        if msg.data == True:
            print("Robot3 Done!")
            self.check_and_save()

    def robot4_done(self,msg):
        self.robot4_status_done = msg.data
        if msg.data == True:
            print("Robot4 Done!")
            self.check_and_save()

if __name__=='__main__':

    logger = time_logger()
    rospy.init_node("time_logger")
    rospy.loginfo("Loggin data")

    sub_robot1_time = rospy.Subscriber("/robot0/robot0_elapsed_time", Float32, logger.robot1_log)
    sub_robot1_done = rospy.Subscriber("/robot0/done", Bool, logger.robot1_done)
    rospy.loginfo("robot 1 logger")
    sub_robot2_time = rospy.Subscriber("/robot1/robot1_elapsed_time", Float32, logger.robot2_log)
    sub_robot2_done = rospy.Subscriber("/robot1/done", Bool, logger.robot2_done)
    rospy.loginfo("robot 2 logger")
    sub_robot3_time = rospy.Subscriber("/robot2/robot2_elapsed_time", Float32, logger.robot3_log)
    sub_robot3_done = rospy.Subscriber("/robot2/done", Bool, logger.robot3_done)
    rospy.loginfo("robot 3 logger")
    sub_robot4_time = rospy.Subscriber("/robot3/robot3_elapsed_time", Float32, logger.robot4_log)
    sub_robot4_done = rospy.Subscriber("/robot3/done", Bool, logger.robot4_done)
    rospy.loginfo("robot 4 logger")

    rospy.spin()

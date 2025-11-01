#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, ast, math
from std_msgs.msg import String
from nav_msgs.msg import Odometry

current_pose = (0.0, 0.0)
target_pose = None

def euclidean(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def odom_cb(msg):
    global current_pose
    current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

def segment_cb(msg):
    global target_pose
    seg = ast.literal_eval(msg.data)
    target_pose = tuple(seg['end'])
    rospy.loginfo("Monitoring segment toward %s", target_pose)

def monitor():
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if target_pose:
            dist = euclidean(current_pose, target_pose)
            if dist < 0.3:
                rospy.loginfo("✅ Reached target %s", target_pose)
            else:
                rospy.loginfo_throttle(3, "Distance to target: %.2f m" % dist)
        rate.sleep()

def main():
    rospy.init_node('execution_monitor')
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.Subscriber('/path_segments', String, segment_cb)
    monitor()

if __name__ == '__main__':
    main()


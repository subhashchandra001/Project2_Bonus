#!/usr/bin/env python
import rospy, math, ast
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def compute_path(curr_pose, next_pose):
    dx, dy = next_pose[0]-curr_pose[0], next_pose[1]-curr_pose[1]
    dist = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)
    return angle, dist

def callback(msg):
    waypoints = ast.literal_eval(msg.data)
    rospy.loginfo("Received path plan with %d waypoints", len(waypoints))
    pub = rospy.Publisher('/path_segments', String, queue_size=10)

    for i in range(len(waypoints)-1):
        a, d = compute_path(waypoints[i], waypoints[i+1])
        segment = {'start': waypoints[i], 'end': waypoints[i+1],
                   'angle': a, 'distance': d}
        rospy.loginfo("Segment %d: %s", i+1, segment)
        pub.publish(str(segment))
        rospy.sleep(0.5)

def main():
    rospy.init_node('path_planner')
    rospy.Subscriber('/planned_tasks', String, callback)
    rospy.spin()

if __name__ == '__main__':
    main()


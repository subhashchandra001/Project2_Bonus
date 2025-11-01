#!/usr/bin/env python
import rospy
import itertools
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def plan_tasks(tasks, start=(0,0)):
    """Return optimal visiting order minimizing straight-line distance."""
    valid_orders = []
    for perm in itertools.permutations([p for t in tasks for p in t]):
        ok = True
        for s, d in tasks:
            if perm.index(s) > perm.index(d):
                ok = False
                break
        if ok:
            valid_orders.append(list(perm))

    best_order = min(valid_orders,
                     key=lambda o: sum(distance(o[i], o[i+1]) for i in range(len(o)-1)))
    rospy.loginfo("Planned order: %s", best_order)
    return best_order

def main():
    rospy.init_node('task_planner')
    pub = rospy.Publisher('/planned_tasks', String, queue_size=10)
    rospy.loginfo("Enter tasks line by line as ((x1,y1),(x2,y2)) then blank line:")

    lines = []
    while not rospy.is_shutdown():
        try:
            line = raw_input()
        except EOFError:
            break
        if not line.strip():
            break
        lines.append(eval(line))

    order = plan_tasks(lines)
    pub.publish(str(order))
    rospy.loginfo("Published planned task order to /planned_tasks")
    rospy.spin()

if __name__ == '__main__':
    main()


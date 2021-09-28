#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


target = 1


def movebase_client():
    global target
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    if target == 1:
        goal.target_pose.pose.position.x = 5.49
        goal.target_pose.pose.position.y = -4.42
        goal.target_pose.pose.orientation.w = -0.07
    if target == 2:
        goal.target_pose.pose.position.x = -0.77
        goal.target_pose.pose.position.y = -7.99
        goal.target_pose.pose.orientation.w = 0.21
    if target == 3:
        goal.target_pose.pose.position.x = -10.19
        goal.target_pose.pose.position.y = -2.60 #-1.5
        goal.target_pose.pose.orientation.w = 0.84 #-0.2
    if target == 4:
        goal.target_pose.pose.position.x = 2.05
        goal.target_pose.pose.position.y = 2.31
        goal.target_pose.pose.orientation.w = -0.68
    if target ==5:
        goal.target_pose.pose.position.x = 7.01
        goal.target_pose.pose.position.y = 8.10
        goal.target_pose.pose.orientation.w = 0.99

   
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    while target <=5:
        try:
            rospy.init_node('waypoint',anonymous=False)
            result = movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
                target += 1
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

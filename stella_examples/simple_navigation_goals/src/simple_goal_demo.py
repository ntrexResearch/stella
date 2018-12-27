#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(index):

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	
	if index % 2 == 0:
		goal.target_pose.pose.position.x = 0.885164
		goal.target_pose.pose.position.y = -1.04856
		goal.target_pose.pose.position.z = 0.0

		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = -0.61786687
		goal.target_pose.pose.orientation.w = 0.786282720
	else:	
		goal.target_pose.pose.position.x = -4.70164 
		goal.target_pose.pose.position.y = -4.91629
		goal.target_pose.pose.position.z = 0.0

		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = -0.5054096
		goal.target_pose.pose.orientation.w = 0.8628795
		
	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		return client.get_result()

def shutdown_hook():
	global ctrl_c
	ctrl_c = True
	rospy.loginfo("Shutdown")

if __name__ == '__main__':
	try:
		rospy.init_node('movebase_client_py')
		rate = rospy.Rate(0.1)
		index = 0
		count = 0
		rospy.on_shutdown(shutdown_hook)
		ctrl_c = False
		while not ctrl_c:
			result = movebase_client(index)
			if result:
				if index == 1:
					# The robot traveled back to the destination 0 
					count += 1
		    		rospy.loginfo("Goal execution done to the destination %d for %d of times!", index, count)
				index = (index + 1) % 2
				#rospy.sleep(5000)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")


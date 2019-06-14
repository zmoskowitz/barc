#!/usr/bin/env python
import rospy

def helloworld():

	#Initialize node with a default name
	rospy.init_node('default_node_name', anonymous=True)

	#Prints to INFO log
	rospy.loginfo("HELLOOO WOOORRLD")

if __name__ =='__main__':
	try:
		helloworld()
	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python
import rospy
from package_name.msg import new_message

def callback_function(data):
	print(data)

def subscriber_name():
	# Initialize node
	rospy.init_node('subscriber_name', anonymous=True)

	# Create Subscriber object
	rospy.Subscriber('this_is_the_topic_name', new_message,
callback_function)

	# spin() keeps python running until node is shut down
	rospy.spin()

if __name__ == '__main__':
	try:
		subscriber_name()
	except rospy.ROSInterruptException:
		pass

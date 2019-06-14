#!/usr/bin/env python
import rospy
from package_name.msg import new_message
import random
# publisher node
def publisher_name():
	# initialize node
	rospy.init_node('publisher_name', anonymous=True)
	# create global variables
	global pubname , msg
	# Create publisher object
	pubname = rospy.Publisher('this_is_the_topic_name',new_message,
queue_size = 10)
	# defines the "msg" variable as an instance our new message data type "new_message"
	msg = new_message()

	# set node rate
	loop_rate 	= 2
	rate 		= rospy.Rate(loop_rate)

	random.seed(a=0)
	
	while not rospy.is_shutdown():
		msg.floatymcfloatface = random.random();
		if (msg.floatymcfloatface>0.5):
			msg.stringcheese = "Greater than 0.5";
			msg.sales_staff.salesrep_1 = "Jim";
			msg.sales_staff.salesrep_2 = "Dwight";
		else:
			msg.stringcheese = "Less than 0.5";
			msg.sales_staff.salesrep_1 = "Phyllis";
			msg.sales_staff.salesrep_2 = "Stanley";
			#print(msg)
		# Publish message
		pubname.publish(msg)
		# Wait until it's time for another iteration
		rate.sleep()
if __name__ == '__main__':
	try:
		publisher_name()
	except rospy.ROSInterruptException:
		pass
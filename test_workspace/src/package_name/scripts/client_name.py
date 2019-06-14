#!/usr/bin/env python
import sys
import rospy
from package_name.srv import service_name

def client_name(x,y,z) :
	global clientname
	print("Sending request: [%s, %s, %s]" % (x,y,z))
	rospy.wait_for_service('this_is_what_Im_calling_the_service')
	try:
	
		# Create Client object
		clientname = rospy.ServiceProxy('this_is_what_Im_calling_the_service', service_name)
		
		# Calls the service with arguments
		resp = clientname(x,y,z)
		print("Response received: [%s, %s]"%(resp.result_1,resp.result_2))
		return 0
	except rospy.ServiceException, e:
		print "Error: %s"%e
if __name__ == '__main__':
	try:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		z = float(sys.argv[3])
		client_name(x,y,z)
	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python
import rospy
from package_name.srv import service_name
# callback function
def callbackFunction(req):
	print "Receiving request: [%s, %s, %s]"%(req.x, req.y, req.z)
	resp1 = req.x + req.y
	resp2 = resp1 - req.z
	print("%s + %s is %s, minus %s that's %s , quick maths"%(req.x,
req.y,resp1,req.z,resp2))
	print "Sending response: [%s, %s]"%(resp1, resp2)
	return resp1,resp2

def server_name():
	global resp1,resp2
	rospy.init_node('server_name')
	# Create Server object
	servername = rospy.Service('this_is_what_Im_calling_the_service',
service_name, callbackFunction)
	print "Mans not hot."
	rospy.spin()

if __name__ == '__main__':
	try:
		server_name()
	except rospy.ROSInterruptException:
		pass

#! /usr/bin/env python
import rospy
import os
import sys
from paramiko import BadHostKeyException, AuthenticationException, SSHException, SSHClient, AutoAddPolicy
import time
import socket
from std_msgs.msg import Empty

pi_name = "vigirLPi"

def pi_bringup_cb(msg):
	rospy.loginfo("Operator reboot received")
	rospy.sleep(rospy.Duration(15))
	pi_bringup()

def pi_bringup():
	global pi_name
	wait_for_pi_ssh(pi_name)
	os.system("roslaunch hullify pi_onboard.launch " + (" ".join(sys.argv[1:])))
	

def wait_for_pi_ssh(pi_name):
	ssh = SSHClient()
	ssh.set_missing_host_key_policy(AutoAddPolicy())

	wait_interval = 1
	while not rospy.is_shutdown():
		try:
			ssh.connect(pi_name, username="vigir")
			ssh.close()
			rospy.loginfo("Pi connection can be established.")
			return True
		except (BadHostKeyException, AuthenticationException, SSHException, socket.error) as e:
			print "Pi connection exception: ", e
			time.sleep(wait_interval)
	
	return False

def set_pi_name():
	global pi_name
	#print os.environ["ROS_NAMESPACE"]
	try:
		if os.environ["ROS_NAMESPACE"] == "/l_pi":
			pi_name = "vigirLPi"
		else:
			pi_name = "vigirRPi"
	except KeyError:
		rospy.logerr("Cannot determine which namespace we are in, and thus which pi to launch.")
	finally:
		rospy.loginfo("Launching " + pi_name)


if __name__ == "__main__":
	rospy.init_node("launch_pi")
	#print "sys.argv: ", sys.argv

	set_pi_name()
	
	pi_launch_sub = rospy.Subscriber("operator_pi_reboot", Empty, pi_bringup_cb)

	pi_bringup()
	rospy.spin()

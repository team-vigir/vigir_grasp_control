import rospy
import socket
import numpy
import cPickle as pickle
import os
import sys

MAX_SOCKET_PAYLOAD = 10*1024*1024
pickle_protocol = pickle.HIGHEST_PROTOCOL
length_msg_str = "len:"
len_type = numpy.uint32
msg_size_pickle_str_len = len(pickle.dumps(len_type(0), pickle.HIGHEST_PROTOCOL))
GOOD = 0
UNRESPONSIVE = 1

class subprocess_comm_master:
	def __init__(self, master_socket):
		global GOOD
		self.establish_child_connection(master_socket)
		self.conn_state = GOOD

	def establish_child_connection(self, master_socket):
		subprocess_sock_and_addr = master_socket.accept()
		self.pipe_to_child = subprocess_sock_and_addr[0]
		self.pipe_to_child_addr = subprocess_sock_and_addr[1]
	
	def get_child_addr(self):
		return self.pipe_to_child_addr
	
	def get_sock(self):
		return self.pipe_to_child

	def read_max_payload(self):
		global MAX_SOCKET_PAYLOAD
		payload = self.pipe_to_child.recv(MAX_SOCKET_PAYLOAD)
		return payload

	# Returns: False on failure and True on success
	def send_grasping_request(self, grasp_params):
		global MAX_SOCKET_PAYLOAD
		global pickle_protocol

		# Make the grasp parameter string
		grasp_eval_request_string = pickle.dumps(grasp_params, pickle_protocol)
		if len(grasp_eval_request_string) > MAX_SOCKET_PAYLOAD:
				rospy.logfatal("Grasp parameters to send to OpenRAVE subprocesses exceeds maximum socket payload: %d", len(grasp_eval_request_string))
				return False

		msg_size = len(grasp_eval_request_string)
		try:
			write_msg_len(self.pipe_to_child, msg_size)
		except:
			rospy.logerr("Cannot write length message to child in send_grasping_request.")
			return False

		# Write the message
		try:
			num_bytes_sent = self.pipe_to_child.send(grasp_eval_request_string, socket.MSG_DONTWAIT)	# Request non-blocking IO
		except:
			rospy.logerr("Cannot write grasp param message to child in send_grasping_request.")
			return False

		if num_bytes_sent != len(grasp_eval_request_string):
			print "Could not send all grasp evaluation parameters to the subprocess ", self.processes[idx][0]
			print "Sent ", num_bytes_sent, " out of ", len(grasp_eval_request_string)
			return False
		else:
			print "Grasp request sent!"
		return True

	#def is_empty(self):
	#	result_str = self.pipe_to_child.empty()
	#	if result_str == None or result_str == "":
	#		return True
	#	else:
	#		return False

	def get_results(self):
		global pickle_protocol		
		try:
			msg_len = read_msg_len(self.pipe_to_child)
			grasp_results_str = self.pipe_to_child.recv(msg_len, socket.MSG_WAITALL)
			if len(grasp_results_str) != msg_len:
				rospy.logerr("Received %d bytes when expecting %d bytes.", len(grasp_results_str), msg_len)
				return None
		except:
			rospy.logerr("Reading results from subprocess grasp evaluation yielded exception.")
			return None

		grasp_results = pickle.loads(grasp_results_str)
		return grasp_results

	def dump_buffer(self):
		global pickle_protocol
		global MAX_SOCKET_PAYLOAD
		try:
			msg_len = read_msg_len(self.pipe_to_child)
			msg = self.pipe_to_child.recv(MAX_SOCKET_PAYLOAD, socket.MSG_DONTWAIT)
			if len(msg) != msg_len:
				rospy.logerr("Subprocess sent incomplete grasping results. Possibly died.")
				return None
		except:
			rospy.logerr("Trouble child connection. Socket should be closed.")
			return None

		return True



	def set_state(self, state):
		self.conn_state = state

	def get_state(self):
		return self.conn_state

	def shutdown(self):
		try:
			self.pipe_to_child.shutdown(socket.SHUT_RDWR)
		except:
			rospy.loginfo("Exception thrown in master to child socket shutdown. Probably trouble on the child side.")

class subprocess_comm_slave:
	def __init__(self, master_addr):
		self.pipe_to_parent = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.pipe_to_parent.connect(master_addr)
		if self.pipe_to_parent is not None:
			print "\tConnection to parent process achieved. Socket: ", self.pipe_to_parent.getsockname()
			pid = os.getpid()			
			self.pipe_to_parent.send(str(pid))
		else:
			rospy.logfatal("Connection to master grasping process not achieved. Terminating...")
			sys.exit(1)

	# Assumptions: The grasp request coming through the socket
	#	will not exceed the maximum payload specified.
	def listen_for_grasp_request(self):
		msg_len = read_msg_len(self.pipe_to_parent)	
		try:
			new_grasping_task_str = self.pipe_to_parent.recv(msg_len, socket.MSG_WAITALL)
			if len(new_grasping_task_str) != msg_len:
				rospy.logerr("Expected message of length %d, but got %d.", msg_len, len(new_grasping_task_str))
				sys.exit(1)
		except:
			rospy.loginfo("Subprocess param socket threw exception. If main process just closed, it is normal.")
			sys.exit(1)
		
		print "Length of parameters: ", len(new_grasping_task_str)
		if len(new_grasping_task_str) <= 0:
			rospy.loginfo("Exiting OpenRAVE subprocess. Empty socket recv() implies socket has closed.")
			sys.exit(1)

		try:
			new_grasping_task = pickle.loads(new_grasping_task_str)
		except:
			rospy.logerr("Could not unpickle grasp params appropriately. Writing data to file. Skipping evaluation...")
			rospack = rospkg.RosPack()
			rave_to_moveit_path = rospack.get_path('rave_to_moveit')
			err_file = open(rave_to_moveit_path + "/logs/unpickleable_data.pkl", "w")
			err_file.write(new_grasping_task_str)
			err_file.close()
			return None

		return new_grasping_task

	def send_results(self, grasps):
		global pickle_protocol
		result_string = pickle.dumps(grasps, pickle_protocol)
		num_bytes_to_send = len(result_string)		
		if num_bytes_to_send > MAX_SOCKET_PAYLOAD:
			rospy.logfatal("Pickled grasp results from subprocess exceed max socket payload length: %d", num_bytes_to_send)
			sys.exit(1)

		write_msg_len(self.pipe_to_parent, num_bytes_to_send)
	
		num_bytes_sent = self.pipe_to_parent.send(result_string)
		if num_bytes_sent != num_bytes_to_send:
			rospy.logerr("Could not send all result bytes through the socket (%d/%d).", num_bytes_sent, num_bytes_to_send)
			sys.exit(1)


# Description: Will start the main controller's socket
#	for distributing parameters and listening for
#	children.
#	Binds to wildcard address and ephemeral port
def start_master_socket(max_conn_count):
	process_controller_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	process_controller_socket.bind(('0.0.0.0', 0))
	process_controller_socket.listen(max_conn_count)
	process_socket_addr = process_controller_socket.getsockname()
	print "OpenRAVE process controller socket initialized. Address: ", process_socket_addr
	return process_controller_socket, process_socket_addr

# Description: Will write length message header
#	and length to given socket.
# Returns: True on success
# Exceptions: raises an exception on failure.
def write_msg_len(sock, msg_len):
	global length_msg_str
	global len_type
	global pickle_protocol
	msg_size_np_str = pickle.dumps(len_type(msg_len), pickle_protocol)
	len_msg = length_msg_str + msg_size_np_str
	num_bytes_written = sock.send(len_msg, socket.MSG_DONTWAIT)

	if num_bytes_written != len(len_msg):
		rospy.logerr("Could not transmit length message.")
		raise Exception()
		
	return True

# Return: the length of the message to come
# Exceptions: Raises an exception if the data stream is not
#	preceded with the length message header.
def read_msg_len(sock):
	global length_msg_str
	global msg_size_pickle_str_len
	global len_type
	global pickle_protocol

	expected_len = len(length_msg_str) + msg_size_pickle_str_len
	#len_msg_arr = bytearray()
	#len_msg = memoryview(len_msg_arr)
	len_msg = sock.recv(expected_len, socket.MSG_WAITALL)
	if len(len_msg) != expected_len:
		rospy.logerr("Got %d out of %d bytes for length message.", len(len_msg), expected_len)
		raise Exception

	len_msg = len_msg.split(":")
	out_len = pickle.loads(len_msg[1])

	return out_len

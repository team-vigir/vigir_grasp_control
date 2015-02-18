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

class subprocess_comm_master:
	def __init__(self, master_socket):
		self.establish_child_connection(master_socket)

	def establish_child_connection(self, master_socket):
		subprocess_sock_and_addr = master_socket.accept()
		self.child_sock = subprocess_sock_and_addr[0]
		self.child_sock_addr = subprocess_sock_and_addr[1]

	def read_max_payload(self):
		global MAX_SOCKET_PAYLOAD
		bytes = self.child_sock.recv(MAX_SOCKET_PAYLOAD)
		return bytes

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
		write_msg_len(self.child_sock, msg_size)

		# Write the message
		num_bytes_sent = self.child_sock.send(grasp_eval_request_string, socket.MSG_DONTWAIT)	# Request non-blocking IO
		if num_bytes_sent != len(grasp_eval_request_string):
			print "Could not send all grasp evaluation parameters to the subprocess ", self.processes[idx][0]
			print "Sent ", num_bytes_sent, " out of ", len(grasp_eval_request_string)
			return False
		else:
			print "Grasp request sent!"
		return True

	def is_empty(self):
		result_str = self.child_sock.empty()
		if result_str == None or result_str == "":
			return True
		else:
			return False

	def get_results(self):
		global pickle_protocol		
		msg_len = read_msg_len(self.child_sock)
		try:
			grasp_results = self.child_sock.recv(msg_len, socket.MSG_WAITALL)
			if len(grasp_results) != msg_len:
				rospy.logerr("Received %d bytes when expecting %d bytes.", len(grasp_results), msg_len)
				return None
		except:
			rospy.logerr("Reading results from subprocess grasp evaluation yielded exception.")
			return None

		grasp_results = pickle.loads(result_buffer)
		return grasp_results

	def shutdown(self):
		self.child_sock.shutdown()

class subprocess_comm_slave:
	def __init__(self, master_addr):
		self.child_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.child_sock.connect(master_addr)
		if self.child_sock is not None:
			print "\tConnection to parent process achieved. Socket: ", self.child_sock.getsockname()
			pid = os.getpid()			
			self.child_sock.send(str(pid))
		else:
			rospy.logfatal("Connection to master grasping process not achieved. Terminating...")
			sys.exit(1)

	# Assumptions: The grasp request coming through the socket
	#	will not exceed the maximum payload specified.
	def listen_for_grasp_request(self):
		msg_len = read_msg_len(self.child_sock)	
		try:
			new_grasping_task_str = self.child_sock.recv(msg_len, socket.MSG_WAITALL)
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

		write_msg_len(self.child_sock, num_bytes_to_send)
	
		num_bytes_sent = cli_socket.send(result_string)
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

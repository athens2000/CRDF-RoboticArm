import numpy as np

class robotArm():

	def __init__(self, urdf_file_location):
		pass

	##API functions
	def parse_urdf(self):
		#return rpy_array, xyz_array, axis_array
		#size of each = doF
		

	#2 methods -> direct, DH paramteres
	#define 3 arrays
	def forward_kinematics(self, q_array, method):
		if method == 'direct':
			epsilon = 1e-3
			final = np.identity(4)

			#take care of gimbal lock
			for counter in range(len(q_array)):
				#rpy, xyz will get from urdf
				#helper funct to alter the order (order approach)
				#identify gimbal lock conditions: pi/2 change it to pi/2 - epsilon
				first_matrix = make_homogenpus_matrix(rpy_array[counter], xyz_array[counter]) 

				#axis will be from urdf, q is inout to the func
				second_matrix = make_axis_angle_matrix(axis_array[counter], q_array[counter])
				final = final @ first_matrix @ second_matrix

				rpy = self.get_rpy_from_matrix(final)
				if(r == pi/2 or p == pi/2 or y == 0):
					r += epsilon
					p += epsilon
					y += epsilon
			return final



		elif method == 'DH paramteres':
			pass
		#q_array input angles (in radians) array, size = doF
		#return rpy, xyz position od end effector. [origin in robot base]
		pass

	#3 methods -> jacobian transpose, pseudo inverse, hybrid
	def inverse_kinematics(self, rpy, xyz, method):
		#rpy, xyz of end effector
		#return q_array
		pass

	#helper functions -> should never be called by user
	def make_homogenpus_matrix(self, rpy, xyz):
		pass
		#return 4x4 matrix

	def make_axis_angle_matrix(self, axis, q):
		# assume translation = 0,0,0
		pass
		#return a 4x4 matrix



file = "test.urdf"
myArm = robotArm(file)
myArm.forward_kinematics()
myArm.parse_urdf()
myArm.inverse_kinematics()

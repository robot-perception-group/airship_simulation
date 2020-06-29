import numpy as np

W = 2 #[m]
L = 5 #[m]
H = 2 #[m]

class BlimpMixer:
	def __init__(self):
		self.PITCH_OFFSET = 0.06
		self.action = [0,0,0,0,0,0,0,0]
		pass

	def mix(self, rotor_cmd=np.array([0,0,0,0]), plane_cmd=np.array([0,0,0,0])):
		#rotor_cmd:= [p, q, r, thrust]
		r1 = rotor_cmd[0]
		r2 = rotor_cmd[1]
		r3 = rotor_cmd[2]
		r4 = rotor_cmd[3]

		#plane_cmd:= [p, q, r, thrust]
		p1 = plane_cmd[0]
		p2 = plane_cmd[1]
		p3 = plane_cmd[2]
		p4 = plane_cmd[3]

		#servo 
		r_servo = self.PITCH_OFFSET - 3*r2
		if r_servo > np.pi/2:
			r_servo = np.pi/2
		else:
			if r_servo < -np.pi/2:
				r_servo = -np.pi/2

		# self.action[0] = 40*r1 + 50*r2 + 2.0*r4
		# self.action[1] = -40*r1 + 50*r2 + 2.0*r4
		# self.action[2] = 70*r3
		# self.action[3] = r_servo
		# self.action[4] = 0
		# self.action[5] = 0
		# self.action[6] = 0
		# self.action[7] = 0

		self.action[0] = p4
		self.action[1] = p4
		self.action[2] = 0
		self.action[3] = np.pi/2
		self.action[4] = p1
		self.action[5] = p1
		self.action[6] = p2
		self.action[7] = p2	

		return self.action
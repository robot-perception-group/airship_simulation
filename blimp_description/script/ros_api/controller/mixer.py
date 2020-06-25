import numpy as np

W = 2 #[m]
L = 5 #[m]
H = 2 #[m]

class BlimpMixer:
	def __init__(self):
		self.counter = 0
		self.action = [0,0,0,0,0,0,0,0]
		pass

	def mix(self, rotor_cmd=np.array([0,0,0,0]), vtol_cmd=np.array([0,0,0,0])):
		#rotor_cmd:= [p, q, r, thrust]
		
		r1 = rotor_cmd[0]
		r2 = rotor_cmd[1]
		r3 = rotor_cmd[2]
		r4 = rotor_cmd[3]

		self.action[0] = 20*r1 + 20*r2 + 1*r4
		self.action[1] = -20*r1 + 20*r2 + 1*r4
		self.action[2] = 20*r3
		self.action[3] = abs(r3/(r2+1))
		self.action[4] = 0
		self.action[5] = 0
		self.action[6] = 0
		self.action[7] = 0

		return self.action
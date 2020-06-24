import numpy as np

W = 2 #[m]
L = 5 #[m]
H = 2 #[m]

class BlimpMixer:
	def __init__(self):
		self.counter = 0
		pass

	def mix(self, rotor_cmd=np.array([0,0,0,0]), vtol_cmd=np.array([0,0,0,0])):
		#rotor_cmd:= [p, q, r, thrust]
		action = [0,0,0,0,0,0,0,0]

		r1 = rotor_cmd[0]
		r2 = rotor_cmd[1]
		r3 = rotor_cmd[2]
		r4 = rotor_cmd[3]

		action[0] = 20*r1 + 20*r2 + 1*r4
		action[1] = -20*r1 + 20*r2 + 1*r4
		action[2] = 20*r3
		action[3] = 0
		action[4] = 0
		action[5] = 0
		action[6] = 0
		action[7] = 0

		return action
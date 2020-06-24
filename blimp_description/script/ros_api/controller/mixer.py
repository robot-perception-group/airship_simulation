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

		t1 = rotor_cmd[0]
		t2 = rotor_cmd[1]
		t3 = rotor_cmd[2]
		t4 = rotor_cmd[3]

		action[0] = 0*t1 - 100*t2 + 0*t4
		action[1] = -0*t1 - 100*t2 + 0*t4
		action[2] = 0*t3
		action[3] = 0
		action[4] = 0
		action[5] = 0
		action[6] = 0
		action[7] = 0

		return action
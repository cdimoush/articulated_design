#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import os
from shutil import copyfile

# Generate some test data


class Heatmap():
	x_list = []
	y_list = []
	t1_list = []
	t2_list = []

	stepper1_torque = 1.7658
	stepper2_torque = 0.22536
	
	def updatePos(self, pos):
		self.x_list.append(pos.poses[3].position.x)
		self.y_list.append(pos.poses[3].position.y)
		
	
	def torque1(self, t):
		self.t1_list.append(abs(t.data))

	def torque2(self, t):
		self.t2_list.append(abs(t.data))

	def makePlot(self, data):
		rospy.logerr("Plotting")
		x = np.array(self.x_list)
		y = np.array(self.y_list)
		t1 = np.array(self.t1_list)
		t2 = np.array(self.t2_list)


		cmap = matplotlib.cm.get_cmap('viridis')
		normalize1 = matplotlib.colors.Normalize(vmin=0, vmax=self.stepper1_torque)
		normalize2 = matplotlib.colors.Normalize(vmin=0, vmax=self.stepper2_torque*2)
		c1 = [cmap(normalize1(value)) for value in t1]
		c2 = [cmap(normalize2(value)) for value in t2]

		fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True)

		ax1.scatter(x, y, c=c1)
		ax1.set_title("Servo1 Torques")
		cax1, _ = matplotlib.colorbar.make_axes(ax1)
		cbar1 = matplotlib.colorbar.ColorbarBase(cax1, cmap=cmap, norm=normalize1)
		ax2.scatter(x, y, c=c2)
		ax2.set_title("Servo2 Torques")
		cax2, _ = matplotlib.colorbar.make_axes(ax2)
		cbar2 = matplotlib.colorbar.ColorbarBase(cax2, cmap=cmap, norm=normalize2)

		plt.show()

		package_path = os.path.expanduser('~/dojo_ws/src/articulated_design')
		config_path = package_path + '/config/mechanism.yaml'
		folder_path = package_path + '/src/design/output'

		
		i = 0
		while True:
			if not os.path.isdir(folder_path + str(i)):
				os.makedirs(folder_path + str(i))
				break
			else:
				i += 1

		folder_path += str(i)
		fig.savefig(folder_path + '/scatter.png')
		copyfile(config_path, folder_path + '/mechanism.yaml')



	def create(self):
		rospy.Subscriber("/arm/links", PoseArray, self.updatePos)
		rospy.Subscriber("/arm/servo1/torque", Float64, self.torque1)
		rospy.Subscriber("/arm/plot", Empty, self.makePlot)
		rospy.Subscriber("/arm/servo2/torque", Float64, self.torque2)
	
if __name__ == '__main__':
	rospy.init_node('test_pub', anonymous=True)
	Heatmap().create()
	rospy.spin()

#!/usr/bin/env python

from Tkinter import *
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import time
import math

class MechGui(object):
	
	scale = 2500

	def start(self):
		self.root = Tk()
		self.canvas = Canvas(self.root, width=1200, height = 1200)
		self.build()
		self.canvas.pack()
		while not rospy.is_shutdown():
			self.root.mainloop()

	def build(self):
		#place servos 
		s1_coord	 = [0, 0]
		s2_coord = [rospy.get_param('/arm/s2_x') * self.scale, rospy.get_param('/arm/s2_y') * self.scale]
		d = 10
		hub = rospy.get_param('/arm/s2_hub_rad') * self.scale
		s1 = [0] * 4
		s2 = [0] * 4
		s2_hub = [0] * 4
		for i in range(2):
			s1[i] = s1_coord[i] - d/2
			s1[i + 2] = s1_coord[i] + d/2

			s2[i] = s2_coord[i] - d/2
			s2[i + 2] = s2_coord[i] + d/2

			s2_hub[i] = s2_coord[i] - hub
			s2_hub[i + 2] = s2_coord[i] + hub



		#XY correction for GUI
		for i in range(2):
			i = i * 2
			s1[i] = s1[i] + 300
			s1[i+1] = -1*s1[i+1] + 700

			s2[i] = s2[i] + 300
			s2[i+1] = -1*s2[i+1] + 700

			s2_hub[i] = s2_hub[i] + 300
			s2_hub[i+1] = -1*s2_hub[i+1] + 700
		self.servo2_hub = self.canvas.create_oval(s2_hub, outline='black')
		self.servo1 = self.canvas.create_oval(s1, outline='black', fill='red')
		self.servo2 = self.canvas.create_oval(s2, outline='black', fill='red')
		

		#initialize links
		self.link1 = self.canvas.create_line(0, 0, 0, 0, width = 3, fill='black')
		self.link2 = self.canvas.create_line(0, 0, 0, 0, width = 3, fill='black')
		self.link3 = self.canvas.create_line(0, 0, 0, 0, width = 3, fill='black')
		



	def update(self, link_list):
		l1 = [link_list.poses[0].position.x, link_list.poses[0].position.y, link_list.poses[1].position.x, link_list.poses[1].position.y]
		l2 = [link_list.poses[2].position.x, link_list.poses[2].position.y, link_list.poses[3].position.x, link_list.poses[3].position.y]
		l3 = [link_list.poses[4].position.x, link_list.poses[4].position.y, link_list.poses[5].position.x, link_list.poses[5].position.y] 
		
		for i in range(4):
			l1[i] = l1[i] * self.scale
			l2[i] = l2[i] * self.scale
			l3[i] = l3[i] * self.scale
		
		#XY correction for GUI
		for i in range(2):
			i = i * 2
			l1[i] = l1[i] + 300
			l1[i+1] = -1*l1[i+1] + 700

			l2[i] = l2[i] + 300
			l2[i+1] = -1*l2[i+1] + 700

			l3[i] = l3[i] + 300
			l3[i+1] = -1*l3[i+1] + 700


		self.canvas.coords(self.link1, l1[0], l1[1], l1[2], l1[3])
		self.canvas.coords(self.link2, l2[0], l2[1], l2[2], l2[3])
		self.canvas.coords(self.link3, l3[0], l3[1], l3[2], l3[3])

		self.canvas.update()

	def add_text_display(self):
		self.canvas.create_line(800, 0, 800, 800, width=2)

		px, py = 820, 8
		self.canvas.create_text(px, py,text="Arm Mechanism: ", font=('Times', 32), anchor=NW)
		#Dimension
		###############################################
		#links dimensions
		py += 64
		l = (self.l1_length, self.l2_length, self.l3_length)
		for i in range(3):
			text = "Link " + str(i+1) + ": " + str(l[i])
			self.canvas.create_text(px, py, text=text, anchor=NW)
			py += 16
		#Servo 2 Disk
		text = "Disk Radius: " + str(self.s2_hub_rad)
		self.canvas.create_text(px, py, text=text, anchor=NW)
		py += 16
		#Links 2 Lever
		text = "Link 2 Lever: " + str(self.l2_x)
		self.canvas.create_text(px, py, text=text, anchor=NW)


if __name__ == '__main__':
	rospy.init_node('mech_gui_node', anonymous=True)
	gui = MechGui()
	rospy.Subscriber("/arm/links", PoseArray, gui.update)
	gui.start()

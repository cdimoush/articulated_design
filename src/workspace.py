#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray

class Controller():

	s1_angle = 0
	s2_angle = 0
	
	def updatePos(self, pos):
		pass
		
	def updateServo1Angle(self, a):
		pass

	def updateServo2Angle(self, a):
		pass
			
	def arm(self):
		
		self.pub1 = rospy.Publisher('arm/servo1', Float64, queue_size=1000)
		self.pub2 = rospy.Publisher('arm/servo2', Float64, queue_size=1000)
		self.pub3 = rospy.Publisher('arm/plot', Empty, queue_size=1000)
		rospy.Subscriber("/arm/links", PoseArray, self.updatePos)
		rospy.Subscriber("/arm/servo1/angle", Float64, self.updateServo1Angle)
		rospy.Subscriber("/arm/servo2/angle", Float64, self.updateServo2Angle)

		rate = rospy.Rate(100) # 10hz

		rate.sleep()
		self.pub1.publish(self.s1_angle)
		rate.sleep()
		for i2 in range(85):
			self.s2_angle -= 1;
			self.pub2.publish(-1)
			rate.sleep()
		for x in range(16):
			for i1 in range (85):
				self.s1_angle += 1 
				self.pub1.publish(1)
				rate.sleep()
			for i2 in range(5):
				self.s2_angle += 1
				self.pub2.publish(1)
				rate.sleep()
			for i1 in range (85):
				self.s1_angle -= 1 
				self.pub1.publish(-1)
				rate.sleep()
			for i2 in range(5):
				self.s2_angle += 1
				self.pub2.publish(1)
				rate.sleep()

		rospy.Rate(1).sleep()
		self.pub3.publish()
			
if __name__ == '__main__':
	rospy.init_node('workspace', anonymous=True)
	Controller().arm()
	rospy.spin()

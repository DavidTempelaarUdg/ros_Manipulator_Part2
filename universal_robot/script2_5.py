#!/usr/bin/env python
import rospy
import time

import sys  # command line arguments argv
import math  # atan2
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class pos_command(object):

	def __init__(self, position=[]):
		self.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		self.ant_position = [0,0,0,0,0,0]
		self.velocity = [0.5, 0.5,0.5,0.5,0.5,0.5]
		self.effort = []
		self.got_position = False
		self.equals = False
		self.discretization = 10

		rospy.init_node('pos_command')
		self.pub = rospy.Publisher("/joint_states_cmd", JointState, queue_size=10)
		self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)

		self.position = position

	def callback(self, msg):
		self.ant_position = msg.position
		self.got_position = True

	def equals_ant_position(self):
		if self.position[0]-0.0001<self.ant_position[0] and self.position[0]+0.0001>self.ant_position[0] and self.position[1]-0.0001<self.ant_position[1] and self.position[1]+0.0001>self.ant_position[1] and self.position[2]-0.0001<self.ant_position[2] and self.position[2]+0.0001>self.ant_position[2] and self.position[3]-0.0001<self.ant_position[3] and self.position[3]+0.0001>self.ant_position[3] and self.position[4]-0.0001<self.ant_position[4] and self.position[4]+0.0001>self.ant_position[4] and self.position[5]-0.0001<self.ant_position[5] and self.position[5]+0.0001>self.ant_position[5]:
			self.equals = True
		else:
			self.equals = False

	def iterate(self):
		if self.got_position:
			print("past:{}".format(self.ant_position))
			print("actual:{}".format(self.position))
			self.equals_ant_position()
			if self.equals:
				print("position reached, waiting for new position")
				x= input('Enter new joints positions. Ex: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]:')
				self.position = x

			else:
				
				pos_aux = [((self.position[0]-self.ant_position[0])/self.discretization),((self.position[1]-self.ant_position[1])/self.discretization),((self.position[2]-self.ant_position[2])/self.discretization),((self.position[3]-self.ant_position[3])/self.discretization),((self.position[4]-self.ant_position[4])/self.discretization),((self.position[5]-self.ant_position[5])/self.discretization)]
                
				for i in range(1,self.discretization):

					poscom = JointState()
					poscom.name = self.name
					poscom.position = [(pos_aux[0]*i+self.ant_position[0]),(pos_aux[1]*i+self.ant_position[1]),(pos_aux[2]*i+self.ant_position[2]),(pos_aux[3]*i+self.ant_position[3]),(pos_aux[4]*i+self.ant_position[4]),(pos_aux[5]*i+self.ant_position[5])]
					poscom.velocity = self.velocity
					poscom.effort = self.effort

					self.pub.publish(poscom)
					rospy.sleep(0.1)
				

if __name__ == '__main__':
	if not len(sys.argv) == 7:
		print('No points specified in commandline')
	else:
		position = [float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6])]
		node = pos_command(position)

		while not rospy.is_shutdown():
			node.iterate()

	print('\nROS shutdown')


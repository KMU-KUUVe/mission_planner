#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import roslib
import rospy
import smach
import smach_ros
import actionlib
from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal, CheckForObjectsResult, CheckForObjectsFeedback
from enum import Enum

#class Sign(Enum):
#	A = 1
#	B = 2


#define common state Mission
class CommonMission(smach.State):
	def __init__(self, client_name):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name

	def execute(self, userdata):
		rospy.loginfo('Executing state %s', self.client_name)
		client = actionlib.SimpleActionClient(self.client_name, MissionPlannerAction)
		client.wait_for_server()

		goal = MissionPlannerGoal()
		goal.mission = 1
		client.send_goal(goal)
		rospy.loginfo("send goal")

		client.wait_for_result()
		rospy.loginfo('%s finish'%self.client_name)
		return 'finish'

class LaneDetecting(smach.State):
	def __init__(self, client_name):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name
		self.publisher = rospy.Publisher('lane_detector', Int8, queue_size = 10)
		pub = self.publisher
	def execute(self, userdata):
		rospy.loginfo('Executing state %s', self.client_name)
		pub.publish(1)
		client = actionlib.SimpleActionClient(self.client_name, MissionPlannerAction)
	 	client.wait_for_server()

		goal = MissionPlannerGoal()
		goal.mission = 1
		client.send_goal(goal)
		rospy.loginfo("send goal")
		
		client.wait_for_result()
		pub.publish(0)
		rospy.loginfo('%s finish'%self.client_name)
		return 'finish'	

class Parking(smach.State):
	def __init__(self, client_name):
		smach.State.__init__(self, outcomes=['finish'], input_keys=['sign_ab_input'])
		self.client_name = client_name
		
	def execute(self, userdata):
		rospy.loginfo('Executing state %s', self.client_name)
		client = actionlib.SimpleActionClient(self.client_name, MissionPlannerAction)
		client.wait_for_server()
		
		goal = MissionPlannerGoal()
		goal.mission = userdata.sign_ab_input
		client.send_goal(goal)
		rospy.loginfo("send goal")
		
		client.wait_for_result()
		rospy.loginfo('%s finish'%self.client_name)
		return 'finish'	

class TrafficLight(smach.State):
	def __init__(self, client_name, prob):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name
		self.subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCallback)
		self.flag = 0
		self.min_prob = prob

	def imageCallback(self, data):
		client = actionlib.SimpleActionClient(self.client_name, MissionPlannerAction)
		client.wait_for_server()
		goal = CheckForObjectsGoal()
		goal.image = data
		#goal.id
		client.send_goal(goal)
		rospy.loginfo("send goal")

		client.wait_for_result()

		result = client.get_result()
		rospy.loginfo('%s get result'%self.client_name)

		bounding_box = result.bounding_boxes[0]
		probability = bounding_box.probability
		sign_type = bounding_box.Class
		if probability >= min_prob or sign_type == 'Go':
			self.flag = 1

	def execute(self, userdata):
		rospy.loginfo('executing trafficlight classification')
		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.flag == 1:
				rospy.loginfo('Go sign identified')
				return 'finish'
			r.sleep()
		
		
class SignAB(smach.State):
	def __init__(self, client_name, prob):
		smach.State.__init__(self, outcomes=['finish'], output_keys=['sign_ab_output'])
		self.client_name = client_name 
		self.subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCallback)
		self.flag = 0
		self.min_prob = prob

	def imageCallback(self, data):
		client = actionlib.SimpleActionClient(self.client_name, MissionPlannerAction)
		client.wait_for_server()
		goal = CheckForObjectsGoal()
		goal.image = data
		#goal.id
		client.send_goal(goal)
		rospy.loginfo("send goal")

		client.wait_for_result()

		result = client.get_result()
		rospy.loginfo('%s get result'%self.client_name)

		bounding_box = result.bounding_boxes[0]
		probability = bounding_box.probability
		sign_type = bounding_box.Class
		if probability >= min_prob or sign_type == 'Parking_A':
			#self.flag = Sign.A
			self.flag = 1
		elif probabilty >= min_prob or sign_type == 'Parking_B':
			#self.flag = Sign.B
			self.flag = 2

	def execute(self, userdata):
		rospy.loginfo('executing parking_sign classification')
		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.flag == 1:
				rospy.info('Parking_A sign identified')
				#userdata.sign_ab_output = Sign.A
				userdata.sign_ab_output = 1
				return 'finish'
			if self.flag == Sign.B:
				rospy.info('Parking_B sign identified')
				#userdata.sign_ab_output = Sign.B
				userdata.sign_ab_output = 2
				return 'finish'
			r.sleep()

# main
def main():
	rospy.init_node('smach_example_state_machine')

	probability = 70.0

	#create a smach state machine
	sm = smach.StateMachine(outcomes=['success'])
	sm.userdata.sm_counter = 0
	
	#open the container
	with sm:
		smach.StateMachine.add('mission_start',TrafficLight('mission_start', probability), transitions= 
{'finish':'narrow_path'})
		smach.StateMachine.add('narrow_path',CommonMission('narrow_path'), transitions = 
{'finish':'lane_detector'})
		smach.StateMachine.add('lane_detector',LaneDetecting('lane_detector'), transitions = 
{'finish':'u_turn'})
		smach.StateMachine.add('u_turn',CommonMission('u_turn'), transitions =
{'finish':'wait_traffic_light'})
		smach.StateMachine.add('wait_traffic_light',TrafficLight('darknet_ros', probability), transitions = {'finish':'car_tracking'})
		smach.StateMachine.add('car_tracking',CommonMission('car_tracking'), transitions = {'finish':'sign_ab'})
		smach.StateMachine.add('sign_ab',SignAB('darknet_ros', probability), transitions = 
{'finish':'parking'}, remapping = {'sign_ab_output':'sm_counter'})
		smach.StateMachine.add('parking',Parking('parking'), transitions = 
{'finish':'mission_finish'}, remapping	= {'sign_ab_input':'sm_counter'})
		smach.StateMachine.add('mission_finish',CommonMission('mission_finish'),transitions=
{'finish':'success'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	sis.start()

    	#Execute SMACH plan
    	outcome = sm.execute()

    	rospy.spin()
    	sis.stop()

if __name__ == '__main__':
	main()



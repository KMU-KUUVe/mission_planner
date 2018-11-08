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
	def __init__(self, client_name, prob, standard_count):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name
		self.flag = 0
		self.callback_flag = 1
		self.min_prob = prob
		self.traffic_light_count = standard_count
		self.cur_count = 0

	def imageCallback(self, data):
		if self.callback_flag == 1:
			client = actionlib.SimpleActionClient(self.client_name, CheckForObjectsAction)
			client.wait_for_server()
			goal = CheckForObjectsGoal()
			goal.image = data
			#goal.id
			client.send_goal(goal)
			rospy.loginfo("send goal")

			client.wait_for_result()

			result = client.get_result()
			rospy.loginfo('%s get result'%self.client_name)

			bounding_box = result.bounding_boxes.bounding_boxes[0]
			probability = bounding_box.probability
			sign_type = bounding_box.Class
			if probability > self.min_prob and sign_type == 'Go':
				self.flag = 1

	def execute(self, userdata):
		subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCallback)
		rospy.loginfo('executing trafficlight classification')
		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.flag == 1:
				self.cur_count = self.cur_count + 1
				rospy.loginfo('Go sign identified %dtimes'%self.cur_count)
				self.flag = 0
			if self.cur_count > self.traffic_light_count:
				rospy.loginfo('Go sign certainly identified')
				self.callback_flag = 0
				return 'finish'
			r.sleep()
		
		
class SignAB(smach.State):
	def __init__(self, client_name, prob, standard_count):
		smach.State.__init__(self, outcomes=['finish'], output_keys=['sign_ab_output'])
		self.client_name = client_name 
		self.flag = 0
		self.callback_flag = 1
		self.min_prob = prob
		self.sign_ab_count = standard_count
		self.a_count = 0
		self.b_count = 0

	def imageCallback(self, data):
		if self.callback_flag == 1:	
			client = actionlib.SimpleActionClient(self.client_name, CheckForObjectsAction)
			client.wait_for_server()
			goal = CheckForObjectsGoal()
			goal.image = data
			#goal.id
			client.send_goal(goal)
			rospy.loginfo("send goal")

			client.wait_for_result()

			result = client.get_result()
			rospy.loginfo('%s get result'%self.client_name)

			bounding_box = result.bounding_boxes.bounding_boxes[0]
			probability = bounding_box.probability
			sign_type = bounding_box.Class
			if probability > self.min_prob and sign_type == 'Parking_a':
				self.flag = 1
			elif probabilty > self.min_prob and sign_type == 'Parking_b':
				self.flag = 2

	def execute(self, userdata):
		self.subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.imageCallback)
		rospy.loginfo('executing parking_sign classification')
		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if self.flag == 1:
				self.a_count = self.a_count + 1
				rospy.loginfo('Parking_a sign identified %dtimes'%self.a_count)
				self.flag = 0
			elif self.flag == 2:
				self.b_count = self.b_count + 1
				rospy.loginfo('Parking_b sign identified %dtimes'%self.b_count)
				self.flag = 0
			if self.a_count > self.sign_ab_count:
				rospy.loginfo('Parking_a sign certainly identified')
				userdata.sign_ab_output = 1
				self.callback_flag = 0
				return 'finish'
			if self.b_count > self.sign_ab_count:
				rospy.loginfo('Parking_b sign certainly identified')
				userdata.sign_ab_output = 2
				self.callback_flag = 0
				return 'finish'
			r.sleep()

# main
def main():
	rospy.init_node('smach_example_state_machine')
	probability = rospy.get_param('~probability')
	standard_count = rospy.get_param('~standard_count')

	#create a smach state machine
	sm = smach.StateMachine(outcomes=['success'])
	sm.userdata.sm_counter = 0
	
	#open the container
	#change darknet_ros_light -> darknet_ros
	with sm:
		smach.StateMachine.add('mission_start',TrafficLight('darknet_ros_light', probability, standard_count), transitions= 
{'finish':'narrow_path'})
		smach.StateMachine.add('narrow_path',CommonMission('narrow_path'), transitions = 
{'finish':'lane_detector'})
		smach.StateMachine.add('lane_detector',CommonMission('lane_detector'), transitions = 
{'finish':'u_turn'})
		smach.StateMachine.add('u_turn',CommonMission('u_turn_and_crosswalk_stop'), transitions =
{'finish':'wait_traffic_light'})
		smach.StateMachine.add('wait_traffic_light',TrafficLight('darknet_ros_light', probability, standard_count), transitions = {'finish':'car_tracking'})
		smach.StateMachine.add('car_tracking',CommonMission('car_tracking'), transitions = {'finish':'sign_ab'})
		smach.StateMachine.add('sign_ab',SignAB('darknet_ros_ab', probability, standard_count), transitions = 
{'finish':'parking'}, remapping = {'sign_ab_output':'sm_counter'})
		smach.StateMachine.add('parking',Parking('parking'), transitions = 
{'finish':'success'}, remapping	= {'sign_ab_input':'sm_counter'})
		#smach.StateMachine.add('mission_finish',CommonMission('mission_finish'),transitions=
#{'finish':'success'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	sis.start()

    	#Execute SMACH plan
    	outcome = sm.execute()

    	rospy.spin()
    	sis.stop()

if __name__ == '__main__':
	main()

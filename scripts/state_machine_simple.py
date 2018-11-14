#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_ros')
import roslib
import rospy
import rospkg
import smach
import smach_ros
import actionlib
from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal, CheckForObjectsResult, CheckForObjectsFeedback

#define common state Mission
class CommonMission(smach.State):
	def __init__(self, client_name,  stage):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name
		self.stage = stage
	def execute(self, userdata):
		rospack = rospkg.RosPack()
		fr = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "r")
		cur_stage = fr.read()
		cur_stage_num = int(cur_stage[0])
		fr.close()

		if cur_stage_num == self.stage:
			rospy.loginfo('current stage is %d'%cur_stage_num)
			fw = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "w")
			fw.write(str(self.stage + 1))
			fw.close()
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
		else:
			rospy.loginfo('moving to interrupted state')
			return 'finish' 

class Parking(smach.State):
	def __init__(self, client_name, stage):
		smach.State.__init__(self, outcomes=['finish'], input_keys=['sign_ab_input'])
		self.client_name = client_name
		self.stage = stage

	def execute(self, userdata):
		rospack = rospkg.RosPack()
		fr = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "r")
		cur_stage = fr.read()
		cur_stage_num = int(cur_stage[0])
		fr.close()

		if cur_stage_num == self.stage:
			rospy.loginfo('current stage is %d'%cur_stage_num)
			fw = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "w")
			fw.write(str(self.stage + 1))
			fw.close()
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
		else:
			rospy.loginfo('moving to interrupted state')
			return 'finish'	

class TrafficLight(smach.State):
	def __init__(self, client_name, stage, prob, standard_count):
		smach.State.__init__(self, outcomes=['finish'])
		self.client_name = client_name
		self.flag = 0
		self.stage = stage
		self.callback_flag = 0
		self.is_callback = 0
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
				self.is_callback = 1

	def execute(self, userdata):
		start_time = rospy.Time.now()
		dt = rospy.Duration(secs=12)
		self.callback_flag = 1
		rospack = rospkg.RosPack()
		fr = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "r")
		cur_stage = fr.read()
		cur_stage_num = int(cur_stage[0])
		fr.close()
		
		if cur_stage_num == self.stage:
			rospy.loginfo('current stage is %d'%cur_stage_num)
			fw = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "w")
			fw.write(str(self.stage + 1))
			fw.close()
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
				end_time = rospy.Time.now()
				if end_time - start_time > dt:
					rospy.loginfo('cannot identify Go sign')
					self.callback_flag = 0
					return 'finish'
				r.sleep()
				
		else:
			rospy.loginfo('moving to interrupted state')
			return 'finish'
			
			
		
class SignAB(smach.State):
	def __init__(self, client_name, stage, prob, standard_count):
		smach.State.__init__(self, outcomes=['finish'], output_keys=['sign_ab_output'])
		self.client_name = client_name 
		self.flag = 0
		self.callback_flag = 0
		self.stage = stage
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

			bounding_box = result.bounding_boxes.bounding_boxes[0]
			probability = bounding_box.probability
			sign_type = bounding_box.Class
			if probability > self.min_prob and sign_type == 'Parking_a':
				self.flag = 1
			elif probability > self.min_prob and sign_type == 'Parking_b':
				self.flag = 2

	def execute(self, userdata):
		start_time = rospy.Time.now()
		dt = rospy.Duration(secs=12)
		self.callback_flag = 1
		rospack = rospkg.RosPack()
		fr = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "r")
		cur_stage = fr.read()
		cur_stage_num = int(cur_stage[0])
		fr.close()
		
		if cur_stage_num == self.stage:
			rospy.loginfo('current stage is %d'%cur_stage_num)
			fw = open(rospack.get_path('mission_planner') + '/scripts/state_stage.txt', "w")
			fw.write(str(self.stage + 1))
			fw.close()
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
				
				end_time = rospy.Time.now()
				
				if end_time - start_time > dt:
					rospy.loginfo('cannot identify ab sign')
					if self.a_count < self.b_count:
						userdata.sign_ab_output = 2
						rospy.loginfo('Parking_b sign uncertainly identified')
					else: 
						userdata.sign_ab_output = 1
						rospy.loginfo('Parking_a sign uncertainly identified')
					self.callback_flag = 0
					return 'finish'

				r.sleep()
		else:
			rospy.loginfo('moving to interrupted state')
			return 'finish'

# main
def main():
	rospy.init_node('smach_example_state_machine')
	probability = rospy.get_param('~probability')
	standard_count = rospy.get_param('~standard_count')
	mission_start_stage = rospy.get_param('~mission_start_stage')
	narrow_path_stage = rospy.get_param('~narrow_path_stage')
	lane_detector_stage = rospy.get_param('~lane_detector_stage')
	u_turn_stage = rospy.get_param('~u_turn_stage')
	wait_traffic_light_stage = rospy.get_param('~wait_traffic_light_stage')
	car_tracking_stage = rospy.get_param('~car_tracking_stage')
	sign_ab_stage = rospy.get_param('~sign_ab_stage')
	parking_stage = rospy.get_param('~parking_stage')
	
	#create a smach state machine
	sm = smach.StateMachine(outcomes=['success'])
	sm.userdata.sm_counter = 0
	
	#open the container
	#change darknet_ros_light -> darknet_ros
	with sm:
		smach.StateMachine.add('mission_start',TrafficLight('darknet_ros_light', mission_start_stage, probability, standard_count), transitions= 
{'finish':'narrow_path'})
		smach.StateMachine.add('narrow_path',CommonMission('narrow_path', narrow_path_stage), transitions = 
{'finish':'lane_detector'})
		smach.StateMachine.add('lane_detector',CommonMission('lane_detector', lane_detector_stage), transitions = 
{'finish':'u_turn'})
		smach.StateMachine.add('u_turn',CommonMission('u_turn_and_crosswalk_stop', u_turn_stage), transitions =
{'finish':'wait_traffic_light'})
		smach.StateMachine.add('wait_traffic_light',TrafficLight('darknet_ros_light',  wait_traffic_light_stage, probability, standard_count), transitions = {'finish':'car_tracking'})
		smach.StateMachine.add('car_tracking',CommonMission('car_tracking', car_tracking_stage), transitions = {'finish':'sign_ab'})
		smach.StateMachine.add('sign_ab',SignAB('darknet_ros_ab', sign_ab_stage, probability, standard_count), transitions = 
{'finish':'parking'}, remapping = {'sign_ab_output':'sm_counter'})
		smach.StateMachine.add('parking',Parking('parking', parking_stage), transitions = 
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

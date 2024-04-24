import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Int16
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Pose, Point, Quaternion
from charmie_interfaces.msg import RobotSpeech
from charmie_interfaces.srv import ArmTrigger
from std_srvs.srv import SetBool
from functools import partial
import numpy as np 
import math

import time

import math

class ArmUfactory(Node):
	def __init__(self):
		super().__init__("arm_ufactory")
		self.get_logger().info("Initialised my test Node")	

		# ARM SERVICES

		self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
		self.set_joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
		self.motion_enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
		self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
		self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
		self.set_gripper_enable = self.create_client(SetInt16, '/xarm/set_gripper_enable')
		self.set_gripper_mode = self.create_client(SetInt16, '/xarm/set_gripper_mode')
		self.set_gripper = self.create_client(GripperMove, '/xarm/set_gripper_position')
		self.set_pause_time_client = self.create_client(SetFloat32, '/xarm/set_pause_time')
		self.get_gripper_position = self.create_client(GetFloat32,'/xarm/get_gripper_position')
		#self.plan_pose_client = self.create_client(PlanPose, '/xarm_pose_plan')
		#self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
		#self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')

		while not self.set_position_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Position...")

		while not self.set_joint_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Joint...")

		while not self.motion_enable_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Motion Enavle...")

		while not self.set_mode_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Mdde Client...")

		while not self.set_gripper_enable.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Gripper Enable...")

		while not self.set_gripper_mode.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Gripper Mode...")

		while not self.set_gripper.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Gripper Position...")

		while not self.set_pause_time_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Pause Time...")

		while not self.get_gripper_position.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Get Gripper Position...")

		self.create_service(ArmTrigger, 'arm_trigger', self.arm_trigger_callback)

		print("Bool TR Service is ready")
  


		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'flag_arm_finished_movement', 10)

		# ARM TOPICS

		self.barman_or_client_subscriber = self.create_subscription(Int16, "barman_or_client", self.go_barman_or_go_client_callback, 10)
		self.choose_action_subscriber = self.create_subscription(Int16, 'action', self.choose_action_callback, 10)
		self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)
	
		self.flag_arm_finished_movement_ = Bool()
		self.gripper_reached_target = Bool()
		self.set_gripper_req = GripperMove.Request()
		self.joint_values_req = MoveJoint.Request()
		self.get_gripper_req = GetFloat32.Request()
		self.set_pause_time = SetFloat32.Request()
		self.plan_pose_req = PlanPose.Request()
		self.plan_exec_req = PlanExec.Request()
		self.plan_pose_resp = PlanPose.Response()
		self.joint_plan_req = PlanJoint.Request()
		self.arm_finished_movement = Bool()

		self.next_arm_movement = -1
		self.gripper_tr = 0.0
		# self.gripper_tr_chegou = False

		self.gripper_opening = []
		self.a = 0
		self.choose_action = Int16()

		self.estado_tr = 0
		self.ctr_button = 0
		self.ctr_speaker = 0
		#self.next_arm_movement = 0
		self.processo = 0


	def arm_trigger_callback(self, request, response): # this only exists to have a service where we can: "while not self.arm_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # (nothing)
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages

		response.success = True
		response.message = "Arm Trigger"
		return response
	
			
	def go_barman_or_go_client_callback(self, place_to_go: Int16):
		self.get_logger().info(f"Received place_to_go: {place_to_go}")
		self.next_arm_movement = place_to_go.data
		self.demonstration()

	def setup(self):
		
		#define key positions and keypoints

		self.initial_position = [-215.0, 83.4, -65.0, -0.5, 74.9, -90.0] 
		self.restaurant_initial_position = [-224.8, 83.4, -65.0, -0.5, 74.9, -90.0] 
		self.pre_get_order_position = [-149.0, 2.0, -78.0, 245.0, 32.5, -331.0]
		self.get_order_position = [-158.0, 33.0, -118.0, 255.0, 22.0, -343.0]
		self.orient_to_table = [-195.4, 40.2, -52.7, 163.4, 77.8, -264.2]
		#self.pre_pick_coke_tray = [-30.0, 69.0, -116.0, -37.0, 91.0, -110.0]
		self.place_coke_tray = [-233.0, 45.0, -74.0, -35.0, 57.0, -116.0]
		self.pre_place_coke_tray = [-229.0, 39.0, -63.0, -35.0, 53.0, -110.0]
		#self.pick_coke_tray = [-40.0, 68.0, -98.0, -46.0, 74.0, -107.0]
		self.place_juice_tray = [-243.0, 22.0, -81.0, 184.0, -56.0, -335.0]
		#self.pick_juice_tray = [-48.0, 22.0, -76.0, -185.0, -49.0, 35.0]
		self.pre_place_juice_tray = [-237.0, 2.0, -92.0, 182.0, -87.0, -324.0]
		self.pre2_place_juice_tray = [-235.0, 9.0, -63.0, 184.0, -50.0, -325.0]
		self.place_milk_table = [-203.0, 26.0, -113.0, 159.0, 51.0, -239.0]
		self.pre_place_milk_table = [-196.0, 22.0, -108.0, 164.0, 50.0, -245.0]
		self.place_juice_table = [-207.0, -8.0, -77.0, 151.0, 51.0, -231.0]
		self.pre_place_juice_table = [-197.0, -14.0, -72.0, 159.0, 46.0, -242.0]
		self.place_coke_table = [-194.0, 54.0, -117.0, 170.0, 72.0, -255.0]
		self.pre_place_coke_table = [-185.0, 49.0, -110.0, 177.0, 73.0, -264.0]
		#self.check_object = [51.0, 17.0, -114.0, 5.0, 112.0, -44.0]
		
		#define key positions and keypoints
		#self.restaurant_initial_position = [6.0, 90.0, -90.0, 0.0, 90.0, 0.0] 
		print('Nada')

		########### EXPLANATION OF EACH MODE: ########### 
		#   0: position mode
		#   1: servo motion mode
		#   2: joint teaching mode
		#   4: joint velocity mode
		#   5: cartesian velocity mode
		#   6: joint online trajectory planning mode
		#   7: cartesian online trajectory planning mode

		set_mode_client_req = SetInt16.Request()
		set_mode_client_req.data = 0
		self.future = self.set_mode_client.call_async(set_mode_client_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('mode_client')

		########### EXPLANATION OF EACH STATE: ###########
		#   0: motion state
		#   3: pause state
		#   4: stop state

		set_state_client_req = SetInt16.Request()
		set_state_client_req.data = 0
		self.future = self.set_state_client.call_async(set_state_client_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('state_client')

		########### EXPLANATION OF MOTION ENABLE: ###########
		#   enable: 1 means enable, 0 means disable

		motion_enable_req = SetInt16ById.Request()
		motion_enable_req.id = 8
		motion_enable_req.data = 1
		self.future = self.motion_enable_client.call_async(motion_enable_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('motion_enable')

		set_gripper_enable_req = SetInt16.Request()
		set_gripper_enable_req.data = 1
		self.future = self.set_gripper_enable.call_async(set_gripper_enable_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('gripper_enable')

		set_gripper_mode_req = SetInt16.Request()
		set_gripper_mode_req.data = 0
		self.future = self.set_gripper_mode.call_async(set_gripper_mode_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('gripper_mode')


	def deg_to_rad(self, deg):
		rad = [deg[0] * math.pi / 180,
		 deg[1] * math.pi / 180,
		 deg[2] * math.pi / 180,
		 deg[3] * math.pi / 180,
		 deg[4] * math.pi / 180,
		 deg[5] * math.pi / 180,
		 ]
		return rad

	def callback_service_tr(self, future):
		try:
			print(future.result())
			# print(future.result().ret)
			
			""" if self.estado_tr == 0:
				self.estado_tr = 1
				self.demonstration()
			elif self.estado_tr == 1:
				self.estado_tr = 2
				self.demonstration()

			elif self.estado_tr == 3:
				self.estado_tr = 4
				self.demonstration()
			elif self.estado_tr == 4:
				self.estado_tr = 5
				self.demonstration()
			elif self.estado_tr == 5:
				self.estado_tr = 6
				self.demonstration()
			elif self.estado_tr == 6:
				self.estado_tr = 7
				self.demonstration()
			elif self.estado_tr == 7:
				self.estado_tr = 8
				self.demonstration() """
			
			
			
			self.estado_tr += 1
			print("ESTADO = ", self.estado_tr)
			#self.say_hello()
			#self.plan_and_execute()
			self.demonstration()
			""" if self.choose_action.data == 1:
				if self.ctr_button < 6:
					self.say_hello()
				else:
					self.ctr_button = 0
			if self.choose_action.data == 2:
				self.open_close_gripper()
			if self.choose_action.data == 3:
				self.go_rest_arm()
			if self.choose_action.data == 4:
				self.go_grab_first_object()
			if self.choose_action.data == 5:
				self.go_place_first_object_tray()
			if self.choose_action.data == 6:
				self.go_grab_second_object()
			if self.choose_action.data == 7:
				self.go_place_second_object_tray()
			if self.choose_action.data == 8:
				self.go_grab_third_object()
			if self.choose_action.data == 9:
				self.go_place_third_object()
			if self.choose_action.data == 10:
				self.place_3_object_table() """

		except Exception as e:
			self.get_logger().error("Service call failed: %r" % (e,))

	def choose_action_callback(self, act:Int16):
		self.choose_action.data = act.data
		#self.ctr_button += 1
		print("Received Button:", act.data)
		
		if self.choose_action.data == 0:
			print('hello')
			self.speak()
			if self.ctr_speaker == 3:
				self.ctr_speaker = 0
			else :
				self.ctr_speaker += 1
		if self.choose_action.data == 1:
			print('hello')
			self.say_hello()
		if self.choose_action.data == 2:
			print('open_close')
			self.open_close_gripper()
		if self.choose_action.data == 3:
			print('pass')
			self.go_rest_arm()
		if self.choose_action.data == 4:
			print('go grab 1st object')
			self.go_grab_first_object()
		if self.choose_action.data == 5:
			print('go place 1st object tray')
			self.go_place_first_object_tray()
		if self.choose_action.data == 6:
			print('go grab 2nd object')
			self.go_grab_second_object()
		if self.choose_action.data == 7:
			print('go place 2nd object tray')
			self.go_place_second_object_tray()
		if self.choose_action.data == 8:
			print('go grab 3rd object')
			self.go_grab_third_object()
		if self.choose_action.data == 9:
			print('go place 3rd object tray')
			self.go_place_third_object()
		if self.choose_action.data == 10:
			print('go place 2nd object tray')
			self.place_3_object_table()
			
	def callback_service_tr_gripper(self, future):
		try:
			print(future.result())
			# print(future.result().ret)
			self.gripper_tr = future.result().data
			# self.gripper_tr_chegou = True

			if self.check_gripper(self.gripper_tr, self.set_gripper_req.pos):
				print('chegou ao valor de gripper pretendido')
				self.estado_tr += 1
				self.gripper_reached_target.data = False
			
			print("ESTADO = ", self.estado_tr)

			self.demonstration()
			""" if self.choose_action.data == 1:
				if self.ctr_button < 6:
					self.say_hello()
				else:
					self.ctr_button = 0
			if self.choose_action.data == 2:
				self.open_close_gripper()
			if self.choose_action.data == 3:
				self.go_rest_arm()
			if self.choose_action.data == 4:
				self.go_grab_first_object()
			if self.choose_action.data == 5:
				self.go_place_first_object_tray()
			if self.choose_action.data == 6:
				self.go_grab_second_object()
			if self.choose_action.data == 7:
				self.go_place_second_object_tray()
			if self.choose_action.data == 8:
				self.go_grab_third_object()
			if self.choose_action.data == 9:
				self.go_place_third_object()
			if self.choose_action.data == 10:
				self.place_3_object_table() """

		except Exception as e:
			self.get_logger().error("Service call failed: %r" % (e,))

	def check_object_hand(self):
		if self.estado_tr == 0:

			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('aa')		


		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('cc')
	
		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('dd')

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.check_object)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('dd')

		elif self.estado_tr == 4:
			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('aa')

		elif self.estado_tr == 5:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO CHECK') 

	def go_grab_first_object(self):
		# self.flag_arm_finished_movement_.data = False
		# while self.flag_arm_finished_movement_.data == False:
		if self.estado_tr == 0:
			print('a')
			
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			#self.estado_tr = 1
			
		
		elif self.estado_tr == 1:

			print('b')
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			#self.estado_tr = 2

		elif self.estado_tr == 2:
			
			print('c')
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 1.0 #velocidade de 1.0 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			#self.estado_tr = 3

		elif self.estado_tr == 3:

			print('d')
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			position_reached = False
			#self.estado_tr = 4

		elif self.estado_tr == 4:
			print('e')
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			position_reached = False
			#self.estado_tr = 5

		elif self.estado_tr == 5:
			print('f')
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			position_reached = False
			#self.estado_tr = 6

		elif self.estado_tr == 6:
			print('g')
			#Position to pick object from barman
			self.joint_values_req.angles = self.deg_to_rad(self.get_order_position)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			position_reached = False
			#self.estado_tr = 7		

		elif self.estado_tr == 7:
			#Abrir garra
			print('h')
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			#self.estado_tr = 8

		# self.speech_str.command = "Please place the object on my hand. I will close my hand in 3, 2, 1."
		# self.speaker_publisher.publish(self.speech_str)
		# self.wait_for_end_of_speaking()
		# self.rgb_ctr = 12
		# self.rgb.data = self.rgb_ctr
		# self.rgb_mode_publisher.publish(self.rgb)

		elif self.estado_tr == 8:
			
			print('i')
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))

			#self.estado_tr = 9

		elif self.estado_tr == 9:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print("FEITO!!!")
			self.next_arm_movement = 999
			self.demonstration()


		# self.flag_arm_finished_movement_.data = True
		# print("DONE")

	def go_place_first_object_tray(self):

		#self.flag_arm_finished_movement_.data = False

		if self.estado_tr == 0:

			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('aa')
		
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('cc')
	
		elif self.estado_tr == 2: 
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('dd')

		elif self.estado_tr == 3: 
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('ee')

		elif self.estado_tr == 4: 
			self.joint_values_req.angles = self.deg_to_rad([-218.0, 41.0, -109.0, -23.0, 97.0, -124.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('ff')

		elif self.estado_tr == 5: 
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_coke_tray)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('gg')

		elif self.estado_tr == 6: 
			#place coke on tray
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('hh')
	

		elif self.estado_tr == 7: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 8: 
			#Fechar Garra
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 300.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 9:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('kk') 


		elif self.estado_tr == 10: 
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 700.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))"""
			print('ll') 
   
		elif self.estado_tr == 11:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 12:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr)) """

		elif self.estado_tr == 13: 
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('nn')

		elif self.estado_tr == 14:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 2.0') 
			self.next_arm_movement = 999
			self.demonstration()
		
			#self.flag_arm_finished_movement_.data = True

		# I will get ready to receive the second object
			

		# Depois disto o robot tem de dar um led verde no restaurante

	def go_grab_second_object(self):
		self.flag_arm_finished_movement_.data = False

		if self.estado_tr == 0:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad([-218.0, 31.0, -46.0, -31.0, 48.0, -100.0])
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('aaa')


		elif self.estado_tr == 1:
			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('bbb')


		elif self.estado_tr == 2:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ccc')

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('ddd')

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('eee')

		elif self.estado_tr == 5:
			#Position to pick object from barman
			self.joint_values_req.angles = self.deg_to_rad(self.get_order_position)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('fff')

		elif self.estado_tr == 6:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('ggg')

		# self.speech_str.command = "Please place the object on my hand. I will close my hand in 3, 2, 1."
		# self.speaker_publisher.publish(self.speech_str)
		# self.wait_for_end_of_speaking()
		# self.rgb_ctr = 12
		# self.rgb.data = self.rgb_ctr
		# self.rgb_mode_publisher.publish(self.rgb)

		elif self.estado_tr == 7:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('hhh')
			

		elif self.estado_tr == 8:
			"""
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			"""
			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('iii') 

		elif self.estado_tr == 9:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 3.0') 
			self.next_arm_movement = 999
			self.demonstration()
			# Depois disto o robot tem de falar no restaurant e dar um led verde

	def go_place_second_object_tray(self):
		#Fechar Garra
		if self.estado_tr == 0:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('a')			

		elif self.estado_tr == 1:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('b')
		
		elif self.estado_tr == 2:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('c')	
		
		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('d')

		elif self.estado_tr == 4:

			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('e')

		elif self.estado_tr == 5:

			self.joint_values_req.angles = self.deg_to_rad(self.pre2_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('f')


		elif self.estado_tr == 6:

			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('g')

		elif self.estado_tr == 7:

			#place coke on tray
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('h')

		elif self.estado_tr == 8: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 9: 
			#Fechar Garra
  			#self.set_gripper_req.pos = self.gripper_tr + 60.0
    		
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 10:
			self.set_gripper_req.pos = 700.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('kk') 


		elif self.estado_tr == 11: 
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('ll')

		elif self.estado_tr == 12:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr== 13:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('n') 

		elif self.estado_tr == 14:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('o')

		elif self.estado_tr == 15:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('p')

		elif self.estado_tr == 16:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 4.0') 
			self.next_arm_movement = 999
			self.demonstration()
			# Depois disto o robot tem de falar no restaurant e dar um led verde

		# I will get ready to receive the third object

	def go_grab_third_object(self):
		if self.estado_tr == 0:
			###POS JUICE TRAY
			self.joint_values_req.angles = self.deg_to_rad([-244.0, 16.0, -97.0, 183.0, -78.1, -334.0])
			self.joint_values_req.speed = 0.15 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad([-244.0, 16.0, -97.0, 183.0, -78.1, -334.0])
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('p')

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.5 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.get_order_position)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		# self.speech_str.command = "Please place the object on my hand. I will close my hand in 3, 2, 1."
		# self.speaker_publisher.publish(self.speech_str)
		# self.wait_for_end_of_speaking()
		# self.rgb_ctr = 12
		# self.rgb.data = self.rgb_ctr
		# self.rgb_mode_publisher.publish(self.rgb)

		elif self.estado_tr == 7:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('p')


		elif self.estado_tr == 8:
			"""self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))"""
			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr)) 
		
		elif self.estado_tr == 9:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 4.0') 
			self.next_arm_movement = 999
			self.demonstration()
			# Depois disto o robot tem de falar no restaurant e dar um led verde

	def go_place_third_object(self):
		if self.estado_tr == 0:
			#Fechar Garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('p')
		
		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_get_order_position)
			self.joint_values_req.speed = 0.5 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 5.0') 
			self.next_arm_movement = 999
			self.demonstration()
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			
		
		# self.speech_str.command = "I will now place the objects on the table on my right. Please stay clear from the space surrouding my arm."
		# self.speaker_publisher.publish(self.speech_str)
		# self.wait_for_end_of_speaking()
		# self.rgb_ctr = 12
		# self.rgb.data = self.rgb_ctr
		# self.rgb_mode_publisher.publish(self.rgb)
		
	def place_3_object_table(self):
		if self.estado_tr == 0:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -2.0, -40.0, 154.0, 98.0, -242.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_milk_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.place_milk_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 5: 
			#Fechar Garra
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
      		#self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 600.0
			self.set_gripper_req.wait = True
      		#self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('kk') 


		elif self.estado_tr == 7: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 8:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 700.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 9:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 11:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_milk_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 12:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -2.0, -40.0, 154.0, 98.0, -242.0])
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 13:
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 15:
			#self.joint_values_req.angles = self.deg_to_rad([-29.0, -15.0, -56.0, 15.0, 90.0, 0.0])
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

			#### TALVEZ ESTA TRANSIÇÃO POSSA SER CONSIDERADA ESTRANHA

		elif self.estado_tr == 16:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 17:
			###POS JUICE TRAY
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 18:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.15 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 19:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 20:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 21:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre2_place_juice_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 22:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 23:
			self.joint_values_req.angles = self.deg_to_rad([-243.0, -58.0, -5.0, 135.0, 88.0, -214.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 24:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
		elif self.estado_tr == 25:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 26: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 27: 
			#Fechar Garra
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 28:
			self.set_gripper_req.pos = 600.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('kk') 


		elif self.estado_tr == 29: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 30:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 700.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 31:
			self.set_gripper_req.pos = 800.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('n') 

		elif self.estado_tr == 32:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 33:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')


		elif self.estado_tr == 34:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -27.0, -45.0, 148.0, 68.0, -228.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 35:
			self.joint_values_req.angles = self.deg_to_rad([-218.0, 41.0, -109.0, -23.0, 97.0, -124.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 36:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_coke_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 37:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 38:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 39:

			# Fechar garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 40:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 41:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_coke_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 42:
			self.joint_values_req.angles = self.deg_to_rad([-202.0, 47.0, -63.0, 163.0, 119.0, -259.0])
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 43:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 44: 
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 45: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 46:
			self.set_gripper_req.pos = 600.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('kk') 


		elif self.estado_tr == 47: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 48:
			#self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.pos = 700.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 49:
			self.set_gripper_req.pos = 800.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			""" self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))"""
			print('n') 

		elif self.estado_tr == 50:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 51:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 52:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad([-212.0, 53.0, -66.0, 155.0, 123.0, -257.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 53:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 54:
			#Abrir garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 55:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO 6.0') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			
	def place_first_object_table(self):
		if self.estado_tr == 0:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -2.0, -40.0, 154.0, 98.0, -242.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_milk_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.place_milk_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 5: 
			#Fechar Garra
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 6:

			#Fechar Garra
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('kk')


		elif self.estado_tr == 7: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 8:
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 9:

			#Fechar Garra
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
			print('n')

		elif self.estado_tr == 10:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')


		elif self.estado_tr == 12:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO Entregue 1º objeto') 
			self.next_arm_movement = 999
			self.demonstration()

	def place_second_object_table(self):
		if self.estado_tr == 0:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_milk_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -2.0, -40.0, 154.0, 98.0, -242.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			#self.joint_values_req.angles = self.deg_to_rad([-29.0, -15.0, -56.0, 15.0, 90.0, 0.0])
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

			#### TALVEZ ESTA TRANSIÇÃO POSSA SER CONSIDERADA ESTRANHA

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			###POS JUICE TRAY
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_tray)
			self.joint_values_req.speed = 0.15 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 7:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 9:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre2_place_juice_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			self.joint_values_req.angles = self.deg_to_rad([-243.0, -58.0, -5.0, 135.0, 88.0, -214.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 12:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_juice_table)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
		elif self.estado_tr == 13:
			self.joint_values_req.angles = self.deg_to_rad(self.place_juice_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 15: 
			#Fechar Garra
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 16:

			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('kk')


		elif self.estado_tr == 17: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 18:
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 19:

			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('n')

		elif self.estado_tr == 20:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 21:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 22:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO Entrega do segundo objeto') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			self.next_arm_movement = 999
			self.demonstration()
	
	def place_third_object_table(self):
		if self.estado_tr == 0:
			#Waypoints
			self.joint_values_req.angles = self.deg_to_rad([-218.0, -27.0, -45.0, 148.0, 68.0, -228.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad([-218.0, 41.0, -109.0, -23.0, 97.0, -124.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_coke_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_tray)
			self.joint_values_req.speed = 0.2 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			# Fechar garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 8:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_coke_tray)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			self.joint_values_req.angles = self.deg_to_rad([-202.0, 47.0, -63.0, 163.0, 119.0, -259.0])
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 10:
			self.joint_values_req.angles = self.deg_to_rad(self.place_coke_table)
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ii')

		elif self.estado_tr == 12: 
			#Fechar Garra
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('jj')

		elif self.estado_tr == 13:

			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('kk')


		elif self.estado_tr == 14: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 15:
			self.set_gripper_req.pos = self.gripper_tr + 60.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('mm')

		elif self.estado_tr == 16:

			self.set_pause_time.data = 1.5
			self.future = self.set_pause_time_client.call_async(self.set_pause_time)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('n')

		elif self.estado_tr == 17:
			#Abrir garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 18:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 19:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO Terceiro objeto entregue') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			self.next_arm_movement = 999
			self.demonstration()

	def open_close_gripper(self):
		if self.estado_tr == 0:
		#Fechar garra
			print('a')
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
	
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad([-30.0, 90.0, -90.0, 0.0, 90.0, 10.0] )
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.4 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 4: 
			#Abrir garra
			self.set_gripper_req.pos = 20.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 6:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO Abrir fechar garra') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			
	def go_rest_arm(self):
		if self.estado_tr == 0:
			#waypoints
			self.joint_values_req.angles = self.deg_to_rad([-212.0, 53.0, -66.0, 155.0, 123.0, -257.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:

			#Abrir garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))

		elif self.estado_tr == 5:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO REST') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde
			self.next_arm_movement = 999
			self.demonstration()

	def check_gripper(self, current_gripper_pos, desired_gripper_pos):
		print('Abertura gripper em mm =', current_gripper_pos)
		self.gripper_opening.append(current_gripper_pos)
		if abs(current_gripper_pos - desired_gripper_pos) <= 5.0: #basicamente se a garra estiver no valor pretendido com uma diferença de 50mm aceito
			print('Valor de gripper alcançado. Vou para o próximo estado. \n')
			self.gripper_reached_target.data = True

		elif len(self.gripper_opening) > 100:
			i = 0
			reached = 0
			while i < len(self.gripper_opening) - 11: 
				i += 1
				#este ciclo tem o intuito verificar se o valor da garra nas últimas 3 iterações foi o mesmo. Apenas faço isto pois 
				#caso contrário ao fechar a garra ela nunca chegava ao valor que eu lhe passava e nunca avançava de estado
				if abs(self.gripper_opening[i] - self.gripper_opening[i+10]) <= 5.0:
					reached += 1

			if reached >= 5:
				self.gripper_reached_target.data = True

			if self.gripper_reached_target.data == True:
				print('Estou com o gripper nesta posição há algumas iterações. Vou para o próximo estado. \n')

		return self.gripper_reached_target.data

	def get_quaternion_from_euler(self, roll, pitch, yaw):
		"""
		Convert an Euler angle to a quaternion.
		
		Input
			:param roll: The roll (rotation around x-axis) angle in radians.
			:param pitch: The pitch (rotation around y-axis) angle in radians.
			:param yaw: The yaw (rotation around z-axis) angle in radians.
		
		Output
			:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
		"""
		qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
		qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
		qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		
		#print(qx,qy,qz,qw)
  
		return [qx, qy, qz, qw]

	def say_hello(self):
		if self.estado_tr == 0:
			#waypoints
			print('1')
			self.joint_values_req.angles = self.deg_to_rad([90.0, -60.0, -35.0, 0.0, -20.0, 90.0])
			self.joint_values_req.speed = 0.3 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 5.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('2')
			self.joint_values_req.angles = self.deg_to_rad([90.0, -60.0, -35.0, 0.0, 0.0, 90.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 5.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			print('3') 
			self.joint_values_req.angles = self.deg_to_rad([90.0, -60.0, -35.0, 0.0, 20.0, 90.0])
			self.joint_values_req.speed = 0.35 #velocidade de 1.5 é aceitável para maioria dos movimentos para waypoints
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 5.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			self.estado_tr = 0
			self.ctr_button += 1
	
	def plan_and_execute(self):
	
		if self.estado_tr == 0:
      
			""" self.joint_plan_req.target = self.deg_to_rad([40.0, 29.0, -113.0, -110.0, 31.0, 15.0])

			self.future = self.joint_plan_client.call_async(self.joint_plan_req)
			self.future.add_done_callback(partial(self.callback_service_tr)) """			
    
			target = Pose()
			#temp = self.deg_to_rad([792.0, -13.0, 40.0, 85.7, 6.5, 99.7])
			""" ang_0 = 85.6 * math.pi / 180.0
			ang_1 = 5.6 * math.pi / 180.0
			ang_2 = 99.8 * math.pi / 180.0
			
			target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = self.get_quaternion_from_euler(ang_0, ang_1, ang_2) 
			target.position.x = 656.0
			target.position.y = 456.2
			target.position.z = 530.9 """

			target.position = Point(x = 0.0, y = 0.0, z = 0.0)
			target.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
   
			self.plan_pose_req.target = target

			print("plan pose: ", self.plan_pose_req)

			self.future = self.plan_pose_client.call_async(self.plan_pose_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
			print('quero a resposta do serviço')
   

			""" elif self.estado_tr == 1:
				self.plan_exec_req.wait = True
				print(self.plan_exec_req)
				self.future = self.exec_plan_client.call_async(self.plan_exec_req)
				self.future.add_done_callback(partial(self.callback_service_tr)) """
 
	def speak(self):
		sp_ = RobotSpeech()
		if self.ctr_speaker == 0:
			sp_.language = 'pt'
			sp_.command = "Vou levar o brinquedo."
			self.speaker_publisher.publish(sp_)
			
		if self.ctr_speaker == 1:
			sp_.language = 'pt'
			sp_.command = "O meu nome é charmie."
			self.speaker_publisher.publish(sp_)
			
		if self.ctr_speaker == 2:
			sp_.language = 'pt'
			sp_.command = "Feliz natal."
			self.speaker_publisher.publish(sp_)
			
		if self.ctr_speaker == 3:
			sp_.language = 'pt'
			sp_.command = "ow ow ow"
			self.speaker_publisher.publish(sp_)
			
	def demonstration(self):

		print('valor vindo do pick and place: ', self.next_arm_movement)
		if self.next_arm_movement == 0:
			self.go_grab_first_object()
			#self.next_arm_movement = 1

		elif self.next_arm_movement == 1:
			self.go_place_first_object_tray()
			#self.next_arm_movement = 2
   
		elif self.next_arm_movement == 2:
			self.go_grab_second_object()
			#self.next_arm_movement = 3
   
		elif self.next_arm_movement == 3:
			self.go_place_second_object_tray()
			#self.next_arm_movement = 4
   
		elif self.next_arm_movement == 4:
			self.go_grab_third_object()
			#self.next_arm_movement = 5
   
		elif self.next_arm_movement == 5:
			self.go_place_third_object()
			#self.next_arm_movement = 6
   
		elif self.next_arm_movement == 6:
			self.place_first_object_table()
			#self.next_arm_movement = 7
   
		elif self.next_arm_movement == 7:
			self.place_second_object_table()
			#self.next_arm_movement = 8
   
		elif self.next_arm_movement == 8:
			self.place_third_object_table()
			#self.next_arm_movement = 9
   
		elif self.next_arm_movement == 9:
			self.go_rest_arm()
			#self.next_arm_movement = 0

		else:
			pass
   
		""" elif self.next_arm_movement == 10:
			self.check_object_hand()
			self.next_arm_movement = 11 """
   
		""" elif self.next_arm_movement == 11: 
			self.open_close_gripper()
			self.next_arm_movement = 12 """
		
	def demonstration_sell(self):
		if self.next_arm_movement == 0:
			self.go_grab_first_object()
		elif self.next_arm_movement == 1:
			self.go_place_first_object_tray()
		elif self.next_arm_movement == 2:
			self.go_grab_second_object()
		elif self.next_arm_movement == 3:
			self.go_place_second_object_tray()
		elif self.next_arm_movement == 4:
			self.go_grab_third_object()
		elif self.next_arm_movement == 5:
			self.go_place_third_object()
		elif self.next_arm_movement == 6:
			self.place_first_object_table()
		elif self.next_arm_movement == 7:
			self.place_second_object_table()
		elif self.next_arm_movement == 8:
			self.place_third_object_table()
		elif self.next_arm_movement == 9:
			self.go_rest_arm()
		elif self.next_arm_movement == 10:
			self.check_object_hand()
		elif self.next_arm_movement == 11:
			self.open_close_gripper()
	
	def move_to_goal(self):
		self.setup()
		#self.say_hello()
		#self.plan_and_execute()
		self.demonstration()
		
		print('---------')
		
		
		# self.demonstration()

def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	node.move_to_goal()
	#nodae.move_to_goal()
	rclpy.spin(node)
	rclpy.shutdown()
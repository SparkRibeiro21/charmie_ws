import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Int16, String
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Pose, Point, Quaternion
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

		# ARM TOPICS
		self.arm_command_subscriber = self.create_subscription(String, "arm_command", self.arm_command_callback, 10)
		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'arm_finished_movement', 10)

		# ARM SERVICES
		self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
		self.set_joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
		self.motion_enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
		self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
		self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
		self.set_gripper_enable = self.create_client(SetInt16, '/xarm/set_gripper_enable')
		self.set_gripper_mode = self.create_client(SetInt16, '/xarm/set_gripper_mode')
		self.set_gripper_speed = self.create_client(SetFloat32, '/xarm/set_gripper_speed')
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
			self.get_logger().warn("Waiting for Server Set Mode Client...")

		while not self.set_gripper_enable.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Gripper Enable...")

		while not self.set_gripper_speed.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Gripper Speed...")

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

		self.gripper_reached_target = Bool()
		self.set_gripper_req = GripperMove.Request()
		self.joint_values_req = MoveJoint.Request()
		self.position_values_req = MoveCartesian.Request()
		self.get_gripper_req = GetFloat32.Request()
		self.set_pause_time = SetFloat32.Request()
		self.plan_pose_req = PlanPose.Request()
		self.plan_exec_req = PlanExec.Request()
		self.plan_pose_resp = PlanPose.Response()
		self.joint_plan_req = PlanJoint.Request()

		self.wrong_movement_received = False
		self.end_of_movement = False
		self.gripper_tr = 0.0
		self.gripper_opening = []
		self.estado_tr = 0

		# initial debug movement 
		self.next_arm_movement = "debug_initial"

		self.setup()
		print('---------')
		self.movement_selection()


	def arm_trigger_callback(self, request, response): # this only exists to have a service where we can: "while not self.arm_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # (nothing)
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages

		response.success = True
		response.message = "Arm Trigger"
		return response


	def arm_command_callback(self, move: String):
		self.get_logger().info(f"Received movement selection: {move.data}")
		self.next_arm_movement = move.data
		self.movement_selection()
		# this is used when a wrong command is received
		if self.wrong_movement_received:
			self.wrong_movement_received = False
			temp = Bool()
			temp.data = False
			self.flag_arm_finish_publisher.publish(temp)


	def setup(self):
		
		#define key positions and keypoints
		self.initial_position_bruno = [-215.0, 83.4, -65.0, -0.5, 74.9, 270.0] 
		self.orient_to_table = [-195.4, 40.2, -52.7, 163.4, 77.8, 96.2]
		self.get_order_position = [-161.0, 20.0, -63.0, 256.0, 13.0, 24.0]


		# INITIAL DEBUG MOVEMENT
		self.secondary_initial_position_debug = [-214.8, 83.4, -65.0, -0.5, 74.9, 270.0] 

		# HELLO WAVING MOVEMENT
		self.initial_position =       [-224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.first_waving_position =  [ -90.0,  -53.0,  -35.0,  154.0,  -20.9,  110.0] 
		self.second_waving_position = [ -90.0,  -53.0,  -58.0,  154.0,    7.0,  110.0] 


		self.pre_place_bowl = [-649.9, 199.0, 786.7, 0.69, 0.007, -1.56]
		self.place_bowl = [-553.7, 199.3, 671.6, 1.520, -0.8796, -3.0927]
		self.pos_place_bowl = [-553.7, 132.3, 671.6, 1.520, -0.8796, -3.0927]
		self.orient_to_table_linear = [-357.3, 224.5, 433.0, 0.689, 0.0506, -1.614]
		self.pre_pre_grab_second_object_tray = [-434.3, 105.1, 51.0, 3.017, 0.0069, -1.56]
		self.pre_grab_second_object_tray = [-198.0, 350.0, 170.0, -1.57, 0.0, -1.57]
		self.grab_second_object_tray = [-198.0, 420.0, 170.0, -1.57, 0.0, -1.57]
		self.pos_grab_second_object_tray = [-198.0, 300.0, 170.0, -1.57, 0.0, -1.57]
		self.pre_place_cereal_table = [-649.9, 179.0, 786.7, 0.687, 0.0069, -1.56]
		self.place_cereal_table = [-650.2, 210.4, 786.9, 0.69115, 0.0069, -1.56]
		self.pre_grab_milk_tray = [-320.9, 430.0, 49.3, -2.2689, 0.0, -1.57]
		self.grab_milk_tray = [-230.0, 430.0, -27.0, -2.2689, 0.0, -1.57]
		self.pre_place_milk_tray = [-230.0, 380.0, -27.0, -2.2689, 0.0, -1.57]
		self.pos_grab_milk = [-230.0, 300.0, -27.0, -2.2689, 0.0, -1.57]
		self.place_milk_table = [-650.2, 210.4, 786.9, 0.69115, 0.0069, -1.56]
		self.grab_funilocopo = [-131.5, 430.0, -155.8, -2.2689, 0.0, -1.57]
		self.lift_funilocopo = [-135.0, 345.0, -160.0, -2.2689, 0.0, -1.57]
		self.place_cuttlery_table = [-489.1, 179.5, 593.5, 1.97, -0.808, 2.595]
		self.last_movement = [-572.3, -14.3, -45.4, 3.019, 0.0069, -1.56]
		self.pos_place_cereal_table = [-489.1, 150.5, 593.5, 1.97, -0.808, 2.595]
		self.place_cuttlery_table_up = [-489.1, 150.5, 593.5, 1.97, -0.808, 2.595]

		
		self.ready_to_pick_bag = [-182.5, -38.9, -42.8, 169.7, 8.9, 101.9]
		self.pre_pick_bag = [-271.8, -38.9, -42.8, 169.7, 8.9, 101.9]
		self.pos_pick_bag_rotate = [-265.2, 55.7, -145.6, 82.8, 90.0, 90.0]

		self.pre_pick_bag_linear = [24.3, 474.8, 477.4, math.radians(88.7), math.radians(-2.4), math.radians(179.7)]
		self.before_going_to_initial_position = [-454.3, 123.2, 622.4, math.radians(-156.2), math.radians(83.0), math.radians(29.0)]
		




		### SERVE THE BREAKFAST VARIABLES: ###
		
		# SET JOINTS VARIABLES
		self.get_order_position_joints = 				[ -161.0,   20.0,  -63.0,  256.0,   13.0,   24.0]
		self.get_lower_order_position_joints = 			[ -184.9,   17.5,  -62.0,  115.2,    4.9,  148.0]
		self.initial_position_joints = 					[ -224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 

		# SET POSITIONS VARIABLES
		self.get_order_position_linear = 				[ -581.4, -211.5,  121.8, math.radians( 132.1), math.radians(   1.9), math.radians( -87.1)]
		self.detect_objects_first_position_linear = 	[ -186.6,    1.9,  663.7, math.radians(  87.7), math.radians(   2.9), math.radians(-137.9)]

		self.above_cuttlery_cup = 						[ -135.0,  120.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_in_cuttlery_cup =					[ -135.0,  345.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_cuttlery_cup =				[ -198.0,  344.0, -106.2, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		
		self.above_milk_place_spot = 					[ -230.0,  380.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_milk_in_tray =						[ -230.0,  430.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_milk_in_tray =				[ -230.0,  300.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		
		self.above_cornflakes_place_spot = 				[ -198.0,  350.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_cornflakes_in_tray =					[ -198.0,  420.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_cornflakes_in_tray =		[ -198.0,  300.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
			

		self.pre_ready_to_pick_bag = 					[ -472.4, 28.7, 478.8,  math.radians(88.6),  math.radians(-2.2),  math.radians(-91.0)]
		self.pick_bag = 								[26.9, 867.6, 484.2, math.radians(88.9), math.radians(-2.4), math.radians(179.7)]
		# self.pick_bag = 								[28.7, 853.9, 528.3, math.radians(88.9), math.radians(-2.4), math.radians(179.7)]
		
		self.final_position_rotate = 					[-277.5, 149.8, 419.7, math.radians(-156.2), math.radians(83.0), math.radians(29.0)]
		
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

		# Velocidade do gripper varia entre 1.0 e 5000.0

		set_gripper_speed_req= SetFloat32.Request()
		set_gripper_speed_req.data = 1500.0
		self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		rclpy.spin_until_future_complete(self, self.future)

		print('gripper_speed')

	def deg_to_rad(self, deg):
		rad = [deg[0] * math.pi / 180,
		 deg[1] * math.pi / 180,
		 deg[2] * math.pi / 180,
		 deg[3] * math.pi / 180,
		 deg[4] * math.pi / 180,
		 deg[5] * math.pi / 180,
		 ]
		return rad
	
	def deg_to_rad_single(self, deg):
		rad = deg * math.pi / 180,
		return rad

	def callback_service_tr(self, future):
		try:
			print(future.result())			
			self.estado_tr += 1
			print("ESTADO = ", self.estado_tr)
			self.movement_selection()

		except Exception as e:
			self.get_logger().error("Service call failed: %r" % (e,))
			
	def callback_service_tr_gripper(self, future):
		try:
			print(future.result())

			self.gripper_tr = future.result().data
			
			if self.check_gripper(self.gripper_tr, self.set_gripper_req.pos):
				print('chegou ao valor de gripper pretendido')
				self.estado_tr += 1
				self.gripper_reached_target.data = False
			
			print("ESTADO = ", self.estado_tr)
			self.gripper_tr = future.result().data
			self.movement_selection()


		except Exception as e:
			self.get_logger().error("Service call failed: %r" % (e,))

	def open_close_gripper(self):
		if self.estado_tr == 0:
			print('a')
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
	
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.secondary_initial_position_debug)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4: 
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 5: 
			#Abrir garra
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 7: 
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1500.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			print('FEITO Abrir fechar garra')
			self.get_logger().info("FINISHED MOVEMENT")	

	def verify_if_object_is_grabbed(self):
		# aqui quero fechar, verificar se tenho algo e se tiver colocar uma flag a 1, se não tiver manter a 0. 
		# Essa flag é que me vai permitir avançar para o próximo estado ou ficar aqui e voltar ao princípio
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
			print('valor da garra =', self.future)
			
		elif self.estado_tr == 2:
			temp = Bool()

			if self.gripper_tr >= 5.0:
				# self.flag_object_grabbed.data = True
				# print('OBJECT GRABBED')
				temp.data = True
			else:
				# self.flag_object_grabbed.data = False
				# print('OBJECT NOT GRABBED')
				temp.data = False

			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def close_gripper(self):

		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def close_gripper_with_check_object(self):

		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('valor da garra =', self.future)
			
		elif self.estado_tr == 3:
			temp = Bool()

			if self.gripper_tr >= 5.0:
				# self.flag_object_grabbed.data = True
				# print('OBJECT GRABBED')
				temp.data = True
			else:
				# self.flag_object_grabbed.data = False
				# print('OBJECT NOT GRABBED')
				temp.data = False

			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def open_gripper(self):

		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def arm_go_rest(self):
		if self.estado_tr == 0:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def pick_bag_carry_my_luggage_pre_check_bag(self):
		print(self.estado_tr)
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))


		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			print('A')
			self.joint_values_req.angles = self.deg_to_rad(self.ready_to_pick_bag)
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			print('b')
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_bag )
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			print('d')
			self.position_values_req.pose = self.pick_bag
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			print('e')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 14.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr)) 

		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def carry_my_luggage_if_failed_pick(self):
		print(self.estado_tr)
		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.position_values_req.pose = self.pre_pick_bag_linear
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def carry_my_luggage_pick_bag_after_failing(self):
		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 14.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.position_values_req.pose = self.pick_bag
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 14.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def carry_my_luggage_bag_picked_correctly(self):
		print(self.estado_tr)
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pos_pick_bag_rotate)
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.final_position_rotate
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def carry_my_luggage_back_to_initial_position(self):
		print(self.estado_tr)
		if self.estado_tr == 0:
			self.position_values_req.pose = self.final_position_rotate
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 14.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.position_values_req.pose = self.before_going_to_initial_position
			self.position_values_req.speed = 50.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 14.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			print('A')
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = math.radians(25)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

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

	def movement_selection(self):
		# self.get_logger().info("INSIDE MOVEMENT_SELECTION")	
		print('valor vindo do pick and place: ', self.next_arm_movement)
		if self.next_arm_movement == "debug_initial":
			self.open_close_gripper()
		elif self.next_arm_movement == "carry_my_luggage_pre_check_bag":
			print('a')
			self.pick_bag_carry_my_luggage_pre_check_bag()
		elif self.next_arm_movement == "carry_my_luggage_if_failed_pick":
			print('a')
			self.carry_my_luggage_if_failed_pick()
		elif self.next_arm_movement == "carry_my_luggage_pick_bag_after_failing":
			print('a')
			self.carry_my_luggage_pick_bag_after_failing()
		elif self.next_arm_movement == "carry_my_luggage_bag_picked_correctly":
			print('a')
			self.carry_my_luggage_bag_picked_correctly()
		elif self.next_arm_movement == "carry_my_luggage_back_to_initial_position":
			print('a')
			self.carry_my_luggage_back_to_initial_position()

	

			""" # new serve breakfast functions
			elif self.next_arm_movement == "search_for_objects":
				self.search_for_objects()
			elif self.next_arm_movement == "search_for_objects_to_ask_for_objects":
				self.search_for_objects_to_ask_for_objects()
			elif self.next_arm_movement == "ask_for_objects_to_initial_position":
				self.ask_for_objects_to_initial_position()
			elif self.next_arm_movement == "verify_if_object_is_grabbed":
				self.verify_if_object_is_grabbed()
			elif self.next_arm_movement == "close_gripper":
				self.close_gripper()
			elif self.next_arm_movement == "close_gripper_with_check_object":
				self.close_gripper_with_check_object()
			elif self.next_arm_movement == "open_gripper":
				self.open_gripper()
			elif self.next_arm_movement == "collect_spoon_to_tray":
				self.collect_spoon_to_tray()
			elif self.next_arm_movement == "collect_milk_to_tray":
				self.collect_milk_to_tray()
			elif self.next_arm_movement == "collect_cornflakes_to_tray":
				self.collect_cornflakes_to_tray()
			elif self.next_arm_movement == "collect_bowl_to_initial_position":
				self.collect_bowl_to_initial_position() 
			"""
		
		# new storing groceries functions
		elif self.next_arm_movement == "arm_go_rest":
			self.arm_go_rest()
		elif self.next_arm_movement == "open_gripper":
			self.open_gripper()
		elif self.next_arm_movement == "close_gripper":
			self.close_gripper()
		elif self.next_arm_movement == "close_gripper_with_check_object":
			self.close_gripper_with_check_object()




		else:
			self.wrong_movement_received = True
			print('Wrong Movement Received - ', self.next_arm_movement)	
		
def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	rclpy.spin(node)
	rclpy.shutdown()
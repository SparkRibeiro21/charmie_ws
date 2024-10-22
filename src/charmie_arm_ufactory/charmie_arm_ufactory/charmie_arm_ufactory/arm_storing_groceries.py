import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Int16, String, Float32
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint, GetFloat32List
from geometry_msgs.msg import Pose, Point, Quaternion
from charmie_interfaces.msg import ArmController, ListOfFloats
from charmie_interfaces.srv import Trigger
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
		self.arm_command_subscriber = self.create_subscription(ArmController, "arm_command", self.arm_command_callback, 10)
		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'arm_finished_movement', 10)

		# ARM SERVICES
		self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
		self.set_joint_client = self.create_client(MoveJoint, '/xarm/set_servo_angle')
		self.motion_enable_client = self.create_client(SetInt16ById, '/xarm/motion_enable')
		self.get_position_client = self.create_client(GetFloat32List, '/xarm/get_position')
		self.set_mode_client = self.create_client(SetInt16, '/xarm/set_mode')
		self.set_state_client = self.create_client(SetInt16, '/xarm/set_state')
		self.set_gripper_enable = self.create_client(SetInt16, '/xarm/set_gripper_enable')
		self.set_gripper_mode = self.create_client(SetInt16, '/xarm/set_gripper_mode')
		self.set_gripper_speed = self.create_client(SetFloat32, '/xarm/set_gripper_speed')
		self.set_gripper = self.create_client(GripperMove, '/xarm/set_gripper_position')
		self.set_pause_time_client = self.create_client(SetFloat32, '/xarm/set_pause_time')
		self.get_gripper_position = self.create_client(GetFloat32,'/xarm/get_gripper_position')
		self.move_tool_line = self.create_client(MoveCartesian, '/xarm/set_tool_position')
		#self.plan_pose_client = self.create_client(PlanPose, '/xarm_pose_plan')
		#self.exec_plan_client = self.create_client(PlanExec, '/xarm_exec_plan')
		#self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')

		self.arm_pose_publisher = self.create_publisher(ArmController, 'arm_current_pose', 10)
  		
		while not self.move_tool_line.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server move_tool_line...")

	
		while not self.set_position_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Set Position...")
		
		while not self.get_position_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Get Arm Position...")

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
		

		self.create_service(Trigger, 'arm_trigger', self.arm_trigger_callback)
		
		print("Bool TR Service is ready")

		self.gripper_reached_target = Bool()
		self.set_gripper_req = GripperMove.Request()
		self.get_position_req = GetFloat32List.Request()
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
		self.adjust_position = 0.0

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
	
	def arm_command_callback(self, move: ArmController):
		self.get_logger().info(f"Received movement selection: {move}")
		self.next_arm_movement = move.command
		self.adjust_position = move.adjust_position

		self.movement_selection()
		# this is used when a wrong command is received
		if self.wrong_movement_received:
			self.wrong_movement_received = False
			temp = Bool()
			temp.data = False
			self.flag_arm_finish_publisher.publish(temp)


	def setup(self):
		
		#define key positions and keypoints
		self.initial_position_bruno = [-178.0, 83.4, -65.0, -0.5, 74.9, 270.0] 
		self.orient_to_table = [-195.4, 40.2, -52.7, 163.4, 77.8, 96.2]
		self.get_order_position = [-161.0, 20.0, -63.0, 256.0, 13.0, 24.0]


		# INITIAL DEBUG MOVEMENT
		self.secondary_initial_position_debug = [-214.8, 83.4, -65.0, -0.5, 74.9, 270.0] 

		# self.inside_wardrobe_left_door = [-206.7, 55.7, -108.5, 36.7, 13.7, 298.2]
		# self.inside_wardrobe_left_door_2 = [-209.4, 66.3, -124.0, -1.9, 35.1, 332.5]
		# self.inside_wardrobe_left_door_3 = [-215.5, 48.3, -79.9, 10.8, 9.3, 314.0]

		# self.inside_wardrobe_left_door = [-211.1, 55.3, -115.5, 18.0, 36.0, 316.8]
		# self.inside_wardrobe_left_door_2 = [-210.7, 64.5, -136.5, 4.7, 57.1, 328.5]
		# self.inside_wardrobe_left_door_2_new = [-210.5, 73.5, -154.6, 0.2, 72.7, 331.4]
		self.inside_wardrobe_left_door = [-197.4, 41.1, -88.0, 13.1, 19.9, 331.0]
		self.inside_wardrobe_left_door_2_new = [-195.7, 69.9, -150.7, -2.2, 72.3, 346.7]

		# self.inside_wardrobe_right_door = [-221.7, 78.3, -102.5, 135.4, 73.8, 106.3]
		# self.inside_wardrobe_right_door_2 = [-216.0, 98.2, -137.8, 154.7, 104.1, 111.5]
		self.inside_wardrobe_right_door = [-215.4, 73.7, -89.9, 143.0, 78.3, 99.3]
		self.inside_wardrobe_right_door_2 = [-206.8, 86.4, -111.4, 159.0, 115.6, 100.5]
		

		# self.wardrobe_left_door_outside = [-217.2, 88.2, -121.2, -1.0, 30.9, 317.8]
		# self.wardrobe_left_door_outside = [-213.5, 95.5, -147.2, -11.4, 60.9, 328.0]
		# self.wardrobe_left_door_outside_2 = [-219.5, 46.8, -87.5, -17.4, 52.2, 327.0]
		self.wardrobe_left_door_outside = [-199.9, 70.9, -95.8, -12.7, 36.3, 345.3]
		self.wardrobe_left_door_outside_2 = [-219.5, 46.8, -87.5, -17.4, 52.2, 327.0]

		self.wardrobe_right_door_outside = [-226.5, 49.5, -65.4, 141.6, 127.0, 105.8]
		self.wardrobe_right_door_outside_2 = [-224.4, 33.2, -59.0, 146.3, 118.5, 110.6]

		# HELLO WAVING MOVEMENT
		self.initial_position =       [-224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.first_waving_position =  [ -90.0,  -53.0,  -35.0,  154.0,  -20.9,  110.0] 
		self.second_waving_position = [ -90.0,  -53.0,  -58.0,  154.0,    7.0,  110.0] 
		self.front_robot = 			  [ -191.7,   17.1,  -50.6,   164.9,   58.2,  97.0]
		# self.arm_check_right_door =   [ -219.0, 23.6, -79.5, 124.3, 50.6, 131.8]
		self.arm_check_right_door =   [ -211.4, 19.6, -65.5, 138.3, 53.0, 116.6]
		# self.arm_check_left_door =    [ -221.7, 78.5, -102.3, 135.4, 73.8, 106.3]
		self.arm_check_left_door =    [ -215.4, 73.7, -89.8, 143.0, 78.3, 99.3]
		self.arm_check_right_door_inside_cabinet =   [ -195.9, 67.2, -140.7, 134.5, 23.2, 220.3]
		# self.arm_check_right_door_inside_cabinet_2 =   [ -207.6, 62.8, -140.4, 112.8, 30.4, 241.6]
		# self.arm_check_left_door_inside_cabinet =    [ -221.7, 78.5, -102.3, 135.4, 73.8, 16.3]
		self.arm_check_left_door_inside_cabinet = [ -215.4, 73.7, -89.8, 143.0, 78.3, 99.3]


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


		# self.get_order_position_linear =  [-581.4, -211.5, 121.8, 2.305, 0.033, -1.52]
		# self.above_tray = [-135.0, 120.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.above_cuttlery_cup = [-135.0, 280.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.middle_cuttlery_cup = [-135.0, 345.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.pos_cuttlery_cup = [-198.8, 344.9, -106.2, - 2.2689, 0.0, -1.57]
		# self.detect_objects_first_position = [-186.6, 1.9, 663.7, 1.531, 0.05, -2.407]




		# self.above_tray = 				  		[ -135.0,  120.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.above_cuttlery_cup = 				[ -135.0,  280.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.middle_cuttlery_cup = 				[ -135.0,  345.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.pos_cuttlery_cup = 				[ -198.8,  344.9, -106.2, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]


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

		self.ask_for_object = [-181.0, 83.0, -65.0, -1.0, -23.7, 270.0]
		self.arm_front_robot_mid = [-198.0, 37.6, -56.5, -197.5, 73.4, 96.0]

		self.arm_front_robot_mid_linear = [-646.8, 144.4, 107.5, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]

		# self.pre_place_cabinet_second_shelf_left_side = [-800.8, 452.7, -58.2, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_second_shelf_right_side = [-800.8, 452.7, 309.9, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_second_shelf_centre = [-800.8, 452.7, 160.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_third_shelf_left_side = [-800.8, 62.3, -58.2, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_third_shelf_right_side = [-800.8, 62.3, 309.9, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_third_shelf_centre = [-800.8, 62.3, 160.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_fourth_shelf_left_side = [-800.8, -400.0, -58.2, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_fourth_shelf_right_side = [-800.8, -400.0, 309.9, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		# self.pre_place_cabinet_fourth_shelf_centre = [-800.8, -400.0, 160.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]

		
		
		self.pre_place_cabinet_fourth_shelf_left_side = [-800.5, -39.1, -40.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		self.pre_place_cabinet_fourth_shelf_right_side = [-800.5, -39.1, 290.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		self.pre_place_cabinet_fourth_shelf_centre = [-800.5, -39.1, 100.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		self.pre_place_cabinet_fifth_shelf_left_side = [-646.8, -360.0, -40.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		self.pre_place_cabinet_fifth_shelf_right_side = [-646.8, -360.0, 290.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]
		self.pre_place_cabinet_fifth_shelf_centre = [-646.8, -360.0, 100.0, math.radians(87.5), math.radians(2.0), math.radians(-92.4)]

		# 646.8

		
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
			self.resultado = future.result()
			print(self.resultado)
			self.returning = future.result().ret
			print(self.returning)
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

	def search_for_objects(self):

		"""
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('a')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		"""
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.get_order_position_joints)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.detect_objects_first_position_linear
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
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

	def place_cabinet_second_shelf_left_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_second_shelf_left_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_second_shelf_centre(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_second_shelf_centre.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_second_shelf_right_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_second_shelf_right_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def place_cabinet_third_shelf_left_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_third_shelf_left_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_third_shelf_centre(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_third_shelf_centre.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	

	def place_cabinet_third_shelf_right_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_third_shelf_right_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fourth_shelf_left_side(self):
		if self.estado_tr == 0:

			temp_adjust_angle_bag = self.pre_place_cabinet_fourth_shelf_left_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fourth_shelf_right_side(self):
		if self.estado_tr == 0:

			temp_adjust_angle_bag = self.pre_place_cabinet_fourth_shelf_right_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fourth_shelf_centre(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_fourth_shelf_centre.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fifth_shelf_left_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_fifth_shelf_left_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fifth_shelf_right_side(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_fifth_shelf_right_side.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_cabinet_fifth_shelf_centre(self):
		if self.estado_tr == 0:
			temp_adjust_angle_bag = self.pre_place_cabinet_fifth_shelf_centre.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def arm_front_robot_linear(self):
		if self.estado_tr == 0:

			temp_adjust_angle_bag = self.arm_front_robot_mid_linear.copy()
			temp_adjust_angle_bag[1] = self.adjust_position

			self.position_values_req.pose = temp_adjust_angle_bag
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	

	def ask_for_object_routine(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.ask_for_object)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def arm_front_robot(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.arm_front_robot_mid)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			
		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def front_robot_oriented_front(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.front_robot)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def check_right_door_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.arm_check_right_door_inside_cabinet)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def check_right_door(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.arm_check_right_door)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	
	def check_left_door(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.arm_check_left_door)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def check_left_door_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.arm_check_left_door_inside_cabinet)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def get_arm_position(self):
		if self.estado_tr == 0: 
			self.future = self.get_position_client.call_async(self.get_position_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('a')

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
			print('--')
			arm_pose = self.resultado.datas
			print('arm pose:', arm_pose)
			print('--')
			arm_controller = ArmController()
			arm_pose_ = []
			for a in arm_pose:
				arm_pose_.append(a)
				print(a)	

			arm_controller.pose = arm_pose_
			print(arm_controller.pose)
			self.arm_pose_publisher.publish(arm_controller)
	
	def open_right_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_right_door)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_right_door_2)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def open_left_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_left_door)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_left_door_2_new)
			self.joint_values_req.speed = 0.6
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def finish_open_right_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.wardrobe_right_door_outside)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
		# 	print('a')
		# 	self.joint_values_req.angles = self.deg_to_rad(self.wardrobe_right_door_outside_2)
		# 	self.joint_values_req.speed = 0.5
		# 	self.joint_values_req.wait = True
		# 	self.joint_values_req.radius = 0.0
		# 	self.future = self.set_joint_client.call_async(self.joint_values_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
		# 	print('b')

		# elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def finish_open_left_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.wardrobe_left_door_outside)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def finish_open_left_door_from_inside_2(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.wardrobe_left_door_outside_2)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def go_initial_position(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.secondary_initial_position_debug)
			self.joint_values_req.speed = 0.8
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			print('FEITO Abrir fechar garra')
			self.get_logger().info("FINISHED MOVEMENT")	

	def movement_selection(self):
		# self.get_logger().info("INSIDE MOVEMENT_SELECTION")	
		print('valor vindo do pick and place: ', self.next_arm_movement)
		if self.next_arm_movement == "debug_initial":
			self.open_close_gripper()
		elif self.next_arm_movement == "ask_for_object_routine":
			self.ask_for_object_routine()
		elif self.next_arm_movement == "arm_front_robot":
			self.arm_front_robot()
		elif self.next_arm_movement == "arm_front_robot_linear":
			self.arm_front_robot_linear()
		elif self.next_arm_movement == "place_cabinet_second_shelf_left_side":
			self.place_cabinet_second_shelf_left_side()
		elif self.next_arm_movement == "place_cabinet_second_shelf_right_side":
			self.place_cabinet_second_shelf_right_side()
		elif self.next_arm_movement == "place_cabinet_second_shelf_centre":
			self.place_cabinet_second_shelf_centre()
		elif self.next_arm_movement == "place_cabinet_third_shelf_left_side":
			self.place_cabinet_third_shelf_left_side()
		elif self.next_arm_movement == "place_cabinet_third_shelf_right_side":
			self.place_cabinet_third_shelf_right_side()
		elif self.next_arm_movement == "place_cabinet_third_shelf_centre":
			self.place_cabinet_third_shelf_centre()
		elif self.next_arm_movement == "place_cabinet_fourth_shelf_left_side":
			self.place_cabinet_fourth_shelf_left_side()
		elif self.next_arm_movement == "place_cabinet_fourth_shelf_right_side":
			self.place_cabinet_fourth_shelf_right_side()
		elif self.next_arm_movement == "place_cabinet_fourth_shelf_centre":
			self.place_cabinet_fourth_shelf_centre()
		elif self.next_arm_movement == "place_cabinet_fifth_shelf_left_side":
			self.place_cabinet_fifth_shelf_left_side()
		elif self.next_arm_movement == "place_cabinet_fifth_shelf_right_side":
			self.place_cabinet_fifth_shelf_right_side()
		elif self.next_arm_movement == "place_cabinet_fifth_shelf_centre":
			self.place_cabinet_fifth_shelf_centre()

			


		
		elif self.next_arm_movement == "front_robot_oriented_front":
			self.front_robot_oriented_front()
		elif self.next_arm_movement == "get_arm_position":
			self.get_arm_position()
		elif self.next_arm_movement == "check_left_door":
			self.check_left_door()
		elif self.next_arm_movement == "check_left_door_inside":
			self.check_left_door_inside()
		elif self.next_arm_movement == "check_right_door":
			self.check_right_door()
		elif self.next_arm_movement == "check_right_door_inside":
			self.check_right_door_inside()
		elif self.next_arm_movement == "finish_open_left_door_from_inside":
			self.finish_open_left_door_from_inside()
		elif self.next_arm_movement == "finish_open_left_door_from_inside_2":
			self.finish_open_left_door_from_inside_2()
		elif self.next_arm_movement == "open_left_door_from_inside":
			self.open_left_door_from_inside()
		elif self.next_arm_movement == "finish_open_right_door_from_inside":
			self.finish_open_right_door_from_inside()
		elif self.next_arm_movement == "open_right_door_from_inside":
			self.open_right_door_from_inside()
		elif self.next_arm_movement == "go_initial_position":
			self.go_initial_position()

			

		


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
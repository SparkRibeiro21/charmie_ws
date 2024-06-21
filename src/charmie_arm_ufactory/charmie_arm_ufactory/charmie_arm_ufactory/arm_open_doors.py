import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Int16, String, Float32
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint, GetFloat32List
from geometry_msgs.msg import Pose, Point, Quaternion
from charmie_interfaces.msg import RobotSpeech, ListOfFloats
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
		self.arm_value_subscriber = self.create_subscription(Float32, "arm_value", self.arm_value_callback, 10)
		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'arm_finished_movement', 10)
		self.arm_pose_publisher = self.create_publisher(ListOfFloats, 'arm_current_pose', 10)
		self.get_joints_robot_subscriber = self.create_subscription(RobotMsg, '/xarm/robot_states', self.get_joints_robot_callback, 10)
		self.arm_set_pose_subscriber = self.create_subscription(ListOfFloats, 'arm_set_desired_pose', self.arm_desired_pose_callback, 10)
		self.arm_set_height_subscriber = self.create_subscription(Float32, 'arm_set_desired_height', self.arm_desired_height_callback, 10)

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
  		
		while not self.move_tool_line.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server move_tool_line...")

	
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

		while not self.get_position_client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server Get Arm Position...")
		

		self.create_service(ArmTrigger, 'arm_trigger', self.arm_trigger_callback)
		
		print("Bool TR Service is ready")

		self.gripper_reached_target = Bool()
		self.set_gripper_req = GripperMove.Request()
		self.joint_values_req = MoveJoint.Request()
		self.position_values_req = MoveCartesian.Request()
		self.move_line_tool_req = MoveCartesian.Request()
		self.get_gripper_req = GetFloat32.Request()
		self.get_position_req = GetFloat32List.Request()
		self.set_pause_time = SetFloat32.Request()
		self.plan_pose_req = PlanPose.Request()
		self.plan_exec_req = PlanExec.Request()
		self.plan_pose_resp = PlanPose.Response()
		self.joint_plan_req = PlanJoint.Request()

		self.resultado = []
		self.wrong_movement_received = False
		self.end_of_movement = False
		self.gripper_tr = 0.0
		self.gripper_opening = []
		self.estado_tr = 0
		self.value = -10.0
		self.arm_pose = []
		self.arm_height = 0.0
		self.robot_joints = []

		# initial debug movement 
  		# self.next_arm_movement = "debug_initial"
		self.next_arm_movement = "go_left"
		
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

	def get_joints_robot_callback(self, robot_joints: RobotMsg):
		self.robot_joints = robot_joints.angle
  		
	def arm_desired_pose_callback(self, arm_desired_pose: ListOfFloats):
		self.get_logger().info(f"Received pose selection: {arm_desired_pose.pose}")
		self.arm_pose = arm_desired_pose.pose

	def arm_desired_height_callback(self, arm_desired_height: Float32):
		self.get_logger().info(f"Received value selection: {arm_desired_height.data}")
		self.arm_height = arm_desired_height.data

	def arm_value_callback(self, value: Float32):
		self.get_logger().info(f"Received value selection: {value.data}")
		self.value = value.data

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
		self.initial_position_bruno = [-178.0, 83.4, -65.0, -0.5, 74.9, 270.0] 
		self.orient_to_table = [-195.4, 40.2, -52.7, 163.4, 77.8, 96.2]
		self.get_order_position = [-161.0, 20.0, -63.0, 256.0, 13.0, 24.0]


		# INITIAL DEBUG MOVEMENT
		self.secondary_initial_position_debug = [-214.8, 83.4, -65.0, -0.5, 74.9, 270.0] 

		# HELLO WAVING MOVEMENT
		self.initial_position =       [-224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.first_waving_position =  [ -90.0,  -53.0,  -35.0,  154.0,  -20.9,  110.0] 
		self.second_waving_position = [ -90.0,  -53.0,  -58.0,  154.0,    7.0,  110.0] 


		# SET JOINTS VARIABLES
		self.get_order_position_joints = 				[ -161.0,   20.0,  -63.0,  256.0,   13.0,   24.0]
		self.get_lower_order_position_joints = 			[ -184.9,   17.5,  -62.0,  115.2,    4.9,  148.0]
		self.initial_position_joints = 					[ -224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.front_robot = 								[ -191.7,   17.1,  -50.6,   164.9,   58.2,  97.0]
		# self.front_robot = 								[ -221.7,   -20.7, -48.3,   151.6,  77.8,  127.0] 
		# self.front_robot = 								[ -234.5,   -27.5, -27.3,   145.8,  95.7,  129.8] 
		self.side_robot = 								[ -213.5,   -5.8, -40.1,   177.7,  136.5,  122.2] 
		self.inside_wardrobe = [-218.4, 12.6, -112.5, 95.4, 35.3, 178.9]
		self.inside_wardrobe_1 = [-216.7, 24.2, -127.6, 55.8, 39.7, 212.8]
		self.inside_wardrobe_2 = [-216.8, 25.5, -130.4, 54.5, 40.6, 302.1]
		self.inside_wardrobe_mid_open = [-218.1, 21.7, -117.1, 32.0, 50.4, 321.8]
		self.inside_wardrobe_final = [-215.4, 41.1, -119.8, 3.7, 62.0, 326.8]
		self.inside_wardrobe_final_2 = [-214.6, 54.9, -124.3, 4.0, 52.7, 326.7]

		self.inside_wardrobe_side = [-303.8, -3.8, -114.7, 132.0, 98.9, 220.7]
		self.inside_wardrobe_side_1 = [-275.7, -8.1, -116.8, 116.7, 76.0, 211.8]
		self.pulling_door_side = [-283.6, -8.2, -115.2, 103.5, 93.3, 214.5]
		self.pulling_door_side_1 = [-249.3, -20.8, -81.8, 73.9, 68.5, 196.4]
		self.pulling_door_side_2 = [-234.1, -17.4, -65.9, 82.6, 49.8, 177.5]

		self.side_washing_machine = [-169.4, 31.6, -69.6, 274.9, 97.7, 55.8]
		self.side_washing_machine_2 = [-145.9, 2.6, -60.6, 285.5, 119.1, 39.3]
		self.final_open_washing_machine = [-233.6, 75.2, -133.8, 234.3, 46.4, 137.0]
		self.inside_wardrobe_to_open_right_door = [-216.9, 82.5, -106.8, 152.4, 133.6, 23.0]
		self.finish_inside_wardrobe_to_open_right_door = [-214.1, 74.3, -104.4, 161.0, 123.1, 27.3]

		self.prepare_drawer = [-154.1, 39.8, -85.0, 285.3, 109.0, 136.8]
		self.pre_close_drawer_above = [-148.7, 24.8, -62.3, 292.3, 109.9, 145.8]
		self.prepare_close_drawer = [-224.7, 21.2, -53.0, 125.7, 64.2, 208.4]

		self.inside_wardrobe_left_door = [-206.7, 55.7, -108.5, 36.7, 13.7, 298.2]
		self.inside_wardrobe_left_door_2 = [-209.4, 66.3, -124.0, -1.9, 35.1, 332.5]
		self.inside_wardrobe_left_door_3 = [-215.5, 48.3, -79.9, 10.8, 9.3, 314.0]

		self.wardrobe_left_door_outside = [-225.4, 76.0, -99.4, -31.3, 20.5, 346.1]

		self.close_right_door_outside = [-188.0, 83.4, -65.0, -0.5, 74.9, 270.0]
		self.close_right_door_outside_waypoint = [-161.0, 20.0, -63.0, 256.0, 13.0, 24.0]
		self.close_right_door_outside_2 = [-145.9, 2.5, -60.5, 285.5, 119.1, 130.3]
		self.close_right_door_outside_3 = [-197.0, 28.4, -84.7, 256.1, 76.0, 130.0]

		self.pre_initial_position_from_closing_cabinet = [-203.8, 4.8, -38.6, 165.8, 71.8, 185.8]


		self.close_left_door_outside = [-188.0, 83.4, -65.0, -0.5, 74.9, 270.0]
		self.close_left_door_outside_2 = [-164.9, 85.5, -110.0, 281.2, 97.8, 335.9]
		self.close_left_door_outside_3 = [-184.0, 87.6, -119.7, 263.6, 89.0, 328.3]

		self.pre_initial_position_from_closing_cabinet_left = [-184.3, 54.0, -72.0, 263.1, 90.4, 342.2]

		self.close_dishwasher = [-215.4, 80.8, -152.9, 96.3, 69.9, 70.6]



		# SET POSITIONS VARIABLES
		self.get_order_position_linear = 				[ -581.4, -211.5,  121.8, math.radians( 132.1), math.radians(   1.9), math.radians( -87.1)]
		self.oriented_floor = [math.radians(89.8), math.radians(0.1), math.radians(179.4)]
		self.oriented_floor = [math.radians(23.7), math.radians(-89.7), math.radians(-114.3)]

		self.orientation_open_drawer = [math.radians(34.6), math.radians(-86.4), math.radians(-123.5)]
		self.orientation_close_drawer = [math.radians(124.7), math.radians(-86.5), math.radians(-123.1)]
		
		self.pre_close_drawer = [-529.7, 419.3, 113.8, math.radians(124.7), math.radians(-86.5), math.radians(-123.1)]
		

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

	def go_initial_position(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.secondary_initial_position_debug)
			self.joint_values_req.speed = 0.4 
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

	def go_left(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [0.0, value, 0.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def go_right(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [0.0, value, 0.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def go_up(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [value, 0.0, 0.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def go_down(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [value, 0.0, 0.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def go_front(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [0.0, 0.0, value, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def go_back(self, value):
		if self.estado_tr == 0:
			self.move_line_tool_req.pose = [0.0, 0.0, value, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.value = 0.0
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
			arm_pose_ = ListOfFloats()
			for a in arm_pose:
				arm_pose_.pose.append(a)
				print(a)	

			print(arm_pose_.pose)
			self.arm_pose_publisher.publish(arm_pose_)

	def debug(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = set_desired_pose_arm
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

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
			self.joint_values_req.speed = 0.4
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

	def front_robot_oriented_side(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.side_robot)
			self.joint_values_req.speed = 0.2
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

	def finish_close_right_door_from_outside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pre_initial_position_from_closing_cabinet)
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

	def finish_close_left_door_from_outside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pre_initial_position_from_closing_cabinet_left)
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
	

	def close_right_door_from_outside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.close_right_door_outside)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.close_right_door_outside_waypoint)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.close_right_door_outside_2)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.close_right_door_outside_3)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 4:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def close_left_door_from_outside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.close_left_door_outside)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.close_left_door_outside_2)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.close_left_door_outside_3)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def open_left_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_left_door)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_left_door_2)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_left_door_3)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def finish_open_left_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.wardrobe_left_door_outside)
			self.joint_values_req.speed = 0.2
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




	""" def open_left_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			self.move_line_tool_req.pose = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_1)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_2)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 4:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_mid_open)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 5:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_final)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 6:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_final_2)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 7:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
 	"""

	def finish_open_right_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.finish_inside_wardrobe_to_open_right_door)
			self.joint_values_req.speed = 0.2
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

	def open_right_door_from_inside(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_to_open_right_door)
			self.joint_values_req.speed = 0.2
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

	def open_left_door(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		if self.estado_tr == 1:
			self.move_line_tool_req.pose = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
			self.move_line_tool_req.speed = 40.0
			self.move_line_tool_req.acc = 500.0
			self.move_line_tool_req.wait = True
			self.move_line_tool_req.timeout = 4.0
			self.future = self.move_tool_line.call_async(self.move_line_tool_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_1)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_2)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 4:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_mid_open)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 5:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_final)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 6:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_final_2)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 7:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def open_left_door_from_side(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_side)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		if self.estado_tr == 1:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.inside_wardrobe_side_1)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pulling_door_side)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 3:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pulling_door_side_1)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 4:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pulling_door_side_2)
			self.joint_values_req.speed = 0.2
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 5:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_front_robot(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ -643.5, height[1],  222.0, math.radians( 88.1), math.radians(2.9), math.radians(-90.1)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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

	# def change_height_front_robot(self, height):
	# 	if self.estado_tr == 0:
	# 		print('a')
	# 		self.position_values_req.pose = [ -352.0, height[1],  558.0, math.radians( 41.6), math.radians(3.2), math.radians(-93.0)]
	# 		self.position_values_req.speed = 40.0
	# 		self.position_values_req.acc = 400.0
	# 		self.position_values_req.wait = True
	# 		self.position_values_req.timeout = 30.0
	# 		self.future = self.set_position_client.call_async(self.position_values_req)
	# 		self.future.add_done_callback(partial(self.callback_service_tr))
	# 		print('b')

	# 	elif self.estado_tr == 1:
	# 		print('.')
	# 		if self.returning != 0:
	# 			print('no')
	# 			self.estado_tr = 0
	# 			self.movement_selection()
				
	# 		else:
	# 			print('yes')
	# 			self.estado_tr = 2
	# 			self.arm_pose = []
	# 			self.movement_selection()

	# 	elif self.estado_tr == 2:
	# 		temp = Bool()
	# 		temp.data = True
	# 		self.flag_arm_finish_publisher.publish(temp)
	# 		self.estado_tr = 0
	# 		self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_side_robot(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ 100.0, height[1],  665.0, math.radians( -2.1), math.radians(3.2), math.radians(-93.0)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_front_robot_value(self, height):
		if self.estado_tr == 0:
			print('a', height)
			self.position_values_req.pose = [ -643.5, height,  222.0, math.radians( 88.1), math.radians(2.9), math.radians(-90.1)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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

	# def change_height_front_robot_value(self, height):
	# 	if self.estado_tr == 0:
	# 		print('a')
	# 		self.position_values_req.pose = [ -440.0, height,  664.0, math.radians( 41.6), math.radians(3.2), math.radians(-93.0)]
	# 		self.position_values_req.speed = 40.0
	# 		self.position_values_req.acc = 400.0
	# 		self.position_values_req.wait = True
	# 		self.position_values_req.timeout = 30.0
	# 		self.future = self.set_position_client.call_async(self.position_values_req)
	# 		self.future.add_done_callback(partial(self.callback_service_tr))
	# 		print('b')

	# 	elif self.estado_tr == 1:
	# 		print('.')
	# 		if self.returning != 0:
	# 			print('no')
	# 			self.estado_tr = 0
	# 			self.movement_selection()
	# 		else:
	# 			print('yes')
	# 			self.estado_tr = 2
	# 			self.movement_selection()

	# 	elif self.estado_tr == 2:
	# 		temp = Bool()
	# 		temp.data = True
	# 		self.flag_arm_finish_publisher.publish(temp)
	# 		self.estado_tr = 0
	# 		self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_side_robot_value(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ 100.0, height,  665.0, math.radians( -2.1), math.radians(3.2), math.radians(-93.0)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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

	def change_height_front_left_robot(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ -643.5, height[1], -113.0, math.radians( 88.1), math.radians(2.9), math.radians(-90.1)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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

	# def change_height_front_left_robot(self, height):
	# 	if self.estado_tr == 0:
	# 		print('a')
	# 		self.position_values_req.pose = [ -580.1, height[1], 354.7, math.radians( 41.6), math.radians(3.2), math.radians(-93.0)]
	# 		self.position_values_req.speed = 40.0
	# 		self.position_values_req.acc = 400.0
	# 		self.position_values_req.wait = True
	# 		self.position_values_req.timeout = 30.0
	# 		self.future = self.set_position_client.call_async(self.position_values_req)
	# 		self.future.add_done_callback(partial(self.callback_service_tr))
	# 		print('b')

	# 	elif self.estado_tr == 1:
	# 		print('.')
	# 		if self.returning != 0:
	# 			print('no')
	# 			self.estado_tr = 0
	# 			self.movement_selection()
	# 		else:
	# 			print('yes')
	# 			self.arm_pose = []
	# 			self.estado_tr = 2
	# 			self.movement_selection()

	# 	elif self.estado_tr == 2:
	# 		temp = Bool()
	# 		temp.data = True
	# 		self.flag_arm_finish_publisher.publish(temp)
	# 		self.estado_tr = 0
	# 		self.get_logger().info("FINISHED MOVEMENT")	


	def change_height_side_left_robot(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ -330.0, height[1],  665.0, math.radians( -2.1), math.radians(3.2), math.radians(-93.0)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_front_left_robot_value(self, height):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = [ -580.1, height, 354.7, math.radians( 41.6), math.radians(3.2), math.radians(-93.0)]
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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

	def move_linear(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			self.position_values_req.pose = set_desired_pose_arm
			self.position_values_req.speed = 40.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
   
	def lower_arm_close_washing_machine(self, joint_1):
		if self.estado_tr == 0:
			
			print('a')
			print(self.robot_joints)
			
			joint_1 = math.radians(joint_1)
			self.robot_joints[1] = float(self.robot_joints[1])
			self.robot_joints[2] = float(self.robot_joints[2])
			self.robot_joints[3] = float(self.robot_joints[3])
			self.robot_joints[4] = float(self.robot_joints[4])
			self.robot_joints[5] = float(self.robot_joints[5])
			# self.position_values_req.pose = [joint_1, self.robot_joints[1], self.robot_joints[2], self.robot_joints[3], self.robot_joints[4], self.robot_joints[5]]
			print([joint_1, self.robot_joints[1], self.robot_joints[2], self.robot_joints[3], self.robot_joints[4], self.robot_joints[5]])
			a = [float(val) for val in [joint_1, self.robot_joints[1], self.robot_joints[2], self.robot_joints[3], self.robot_joints[4], self.robot_joints[5]]]

			self.joint_values_req.angles = a
			self.joint_values_req.speed = 0.1
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
				self.value = 0.0
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def prepare_to_close_drawer(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.pre_close_drawer_above)
			self.joint_values_req.speed = 0.5
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')
			
		elif self.estado_tr == 1:
			print('a')
			self.position_values_req.pose = self.pre_close_drawer
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 1
				self.movement_selection()
			else:
				print('yes')
				self.arm_pose = []
				self.estado_tr = 3
				self.movement_selection()

		elif self.estado_tr == 3:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def change_height_close_drawer_javardo(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.prepare_close_drawer)
			self.joint_values_req.speed = 0.5
			self.joint_values_req.wait = False
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_open_drawer(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			print(set_desired_pose_arm)
			self.position_values_req.pose = [set_desired_pose_arm[0], set_desired_pose_arm[1], set_desired_pose_arm[2], self.orientation_open_drawer[0],self.orientation_open_drawer[1], self.orientation_open_drawer[2]]
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	

	def change_height_close_drawer(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			print(set_desired_pose_arm)
			self.position_values_req.pose = [set_desired_pose_arm[0], set_desired_pose_arm[1], set_desired_pose_arm[2], self.orientation_close_drawer[0],self.orientation_close_drawer[1], self.orientation_close_drawer[2]]
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def change_height_open_drawer(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			print(set_desired_pose_arm)
			self.position_values_req.pose = [set_desired_pose_arm[0], set_desired_pose_arm[1], set_desired_pose_arm[2], self.orientation_open_drawer[0],self.orientation_open_drawer[1], self.orientation_open_drawer[2]]
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def change_depth_to_open_washing_machine(self, set_desired_pose_arm):
		if self.estado_tr == 0:
			print('a')
			print(set_desired_pose_arm)
			self.position_values_req.pose = [set_desired_pose_arm[0], set_desired_pose_arm[1], set_desired_pose_arm[2], self.oriented_floor[0],self.oriented_floor[1], self.oriented_floor[2]]
			self.position_values_req.speed = 80.0
			self.position_values_req.acc = 400.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 30.0
			self.future = self.set_position_client.call_async(self.position_values_req)
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def arm_prepare_open_drawer(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.prepare_drawer)
			self.joint_values_req.speed = 0.5
			self.joint_values_req.wait = False
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def prepare_to_close_dishwasher(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.side_washing_machine)
			self.joint_values_req.speed = 0.5
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 1:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.close_dishwasher)
			self.joint_values_req.speed = 0.4
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('b')

		elif self.estado_tr == 2:
			print('.')
			if self.returning != 0:
				print('no')
				self.estado_tr = 0
				self.movement_selection()
			else:
				print('yes')
				self.arm_pose = []
				self.estado_tr = 3
				self.movement_selection()

		elif self.estado_tr == 3:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def arm_side_of_washing_machine(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.side_washing_machine)
			self.joint_values_req.speed = 0.5
			self.joint_values_req.wait = False
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
				self.arm_pose = []
				self.estado_tr = 2
				self.movement_selection()

		elif self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def end_opening_washing_machine(self):
		if self.estado_tr == 0:
			print('a')
			self.joint_values_req.angles = self.deg_to_rad(self.final_open_washing_machine)
			self.joint_values_req.speed = 0.3
			self.joint_values_req.wait = False
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

	def movement_selection(self):
		# self.get_logger().info("INSIDE MOVEMENT_SELECTION")	
		print('valor vindo do pick and place: ', self.next_arm_movement)
		if self.next_arm_movement == "debug_initial":
			self.open_close_gripper()
		elif self.next_arm_movement == "go_initial_position":
			self.go_initial_position()
		elif self.next_arm_movement == "open_gripper":
			self.open_gripper()
		elif self.next_arm_movement == "close_gripper":
			self.close_gripper()
		elif self.next_arm_movement == "close_gripper_with_check_object":
			self.close_gripper_with_check_object()
		elif self.next_arm_movement == "go_left":
			self.go_left(self.value)
		elif self.next_arm_movement == "go_right":
			self.go_right(self.value)
		elif self.next_arm_movement == "go_up":
			self.go_up(self.value)
		elif self.next_arm_movement == "go_down":
			self.go_down(self.value)
		elif self.next_arm_movement == "go_front":
			self.go_front(self.value)
		elif self.next_arm_movement == "go_back":
			self.go_back(self.value)
		elif self.next_arm_movement == "get_arm_position":
			self.get_arm_position()
		elif self.next_arm_movement == "move_linear":
			self.move_linear(self.arm_pose)
		elif self.next_arm_movement == "debug":
			self.debug(self.arm_pose)
		elif self.next_arm_movement == "front_robot_oriented_front":
			self.front_robot_oriented_front()
		elif self.next_arm_movement == "front_robot_oriented_side":
			self.front_robot_oriented_side()
		elif self.next_arm_movement == "prepare_to_close_dishwasher":
			self.prepare_to_close_dishwasher()
		elif self.next_arm_movement == "change_height_front_robot":
			print(self.arm_pose)
			self.change_height_front_robot(self.arm_pose)
		elif self.next_arm_movement == "change_height_side_robot":
			self.change_height_side_robot(self.arm_pose)
		elif self.next_arm_movement == "change_height_front_left_robot":
			self.change_height_front_left_robot(self.arm_pose)
		elif self.next_arm_movement == "change_height_side_left_robot":
			self.change_height_side_left_robot(self.arm_pose)
		elif self.next_arm_movement == "change_height_front_robot_value":
			print(self.arm_height)
			self.change_height_front_robot_value(self.arm_height)
		elif self.next_arm_movement == "change_height_side_robot_value":
			print(self.arm_height)
			self.change_height_side_robot_value(self.arm_height)
		elif self.next_arm_movement == "open_left_door":
			self.open_left_door()
		elif self.next_arm_movement == "open_left_door_from_inside":
			self.open_left_door_from_inside()
		elif self.next_arm_movement == "close_right_door_from_outside":
			self.close_right_door_from_outside()
		elif self.next_arm_movement == "finish_close_right_door_from_outside":
			self.finish_close_right_door_from_outside()
		elif self.next_arm_movement == "close_left_door_from_outside":
			self.close_left_door_from_outside()
		elif self.next_arm_movement == "finish_close_left_door_from_outside":
			self.finish_close_left_door_from_outside()
		elif self.next_arm_movement == "finish_open_left_door_from_inside":
			self.finish_open_left_door_from_inside()
		elif self.next_arm_movement == "open_right_door_from_inside":
			self.open_right_door_from_inside()
		elif self.next_arm_movement == "finish_open_right_door_from_inside":
			self.finish_open_right_door_from_inside()
		elif self.next_arm_movement == "open_left_door_from_side":
			self.open_left_door_from_side()
		elif self.next_arm_movement == "change_height_close_drawer_javardo":
			self.change_height_close_drawer_javardo()
		elif self.next_arm_movement == "change_height_close_drawer":
			print('---', self.arm_pose)
			self.change_height_close_drawer(self.arm_pose)
		elif self.next_arm_movement == "change_height_open_drawer":
			print('---', self.arm_pose)
			self.change_height_open_drawer(self.arm_pose)
		elif self.next_arm_movement == "prepare_to_close_drawer":
			self.prepare_to_close_drawer()		
		elif self.next_arm_movement == "change_depth_to_open_washing_machine":
			print('---', self.arm_pose)
			self.change_depth_to_open_washing_machine(self.arm_pose)
		elif self.next_arm_movement == "arm_side_of_washing_machine":
			self.arm_side_of_washing_machine()
		elif self.next_arm_movement == "arm_prepare_open_drawer":
			self.arm_prepare_open_drawer()
		elif self.next_arm_movement == "lower_arm_close_washing_machine":
			print('Joint1:', self.value)
			self.lower_arm_close_washing_machine(self.value)
		elif self.next_arm_movement == "end_opening_washing_machine":
			self.end_opening_washing_machine()
			
		
		

		else:
			self.wrong_movement_received = True
			print('Wrong Movement Received - ', self.next_arm_movement)	
		
		self.value = 0.0


def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	rclpy.spin(node)
	rclpy.shutdown()
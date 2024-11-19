import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Int16, String
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Pose, Point, Quaternion
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

		self.create_service(Trigger, 'arm_trigger', self.arm_trigger_callback)
		
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
		self.restaurant_initial_position = [-224.8, 83.4, -65.0, -0.5, 74.9, 270.0] 
		self.orient_to_table = [-195.4, 40.2, -52.7, 163.4, 77.8, 96.2]
		self.get_order_position = [-161.0, 20.0, -63.0, 256.0, 13.0, 24.0]


		# INITIAL DEBUG MOVEMENT
		self.secondary_initial_position_debug = [-214.8, 83.4, -65.0, -0.5, 74.9, 270.0] 

		# HELLO WAVING MOVEMENT
		self.initial_position =       [-224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.first_waving_position =  [ -90.0,  -53.0,  -35.0,  154.0,  -20.9, 110.0] 
		self.second_waving_position = [ -90.0,  -53.0,  -58.0,  154.0,    7.0, 110.0] 


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




		self.get_order_position_linear = [-581.4, -211.5, 121.8, 2.305, 0.033, -1.52]
		self.above_tray = [-135.0, 120.0, -160.0, - 2.2689, 0.0, -1.57]
		self.above_cuttlery_cup = [-135.0, 280.0, -160.0, - 2.2689, 0.0, -1.57]
		self.middle_cuttlery_cup = [-135.0, 345.0, -160.0, - 2.2689, 0.0, -1.57]
		self.pos_cuttlery_cup = [-198.8, 344.9, -106.2, - 2.2689, 0.0, -1.57]









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

	def hello(self):
		
		# Removed safety waypoints
		"""
		if self.estado_tr == 0:
			print('a')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
	
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('ll')

		elif self.estado_tr == 2: # safety
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		"""

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.first_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			print('a')
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.second_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			print('a')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.first_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			print('a')
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.joint_values_req.angles = self.deg_to_rad(self.second_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			print('a')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.joint_values_req.angles = self.deg_to_rad(self.first_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			print('a')
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.joint_values_req.angles = self.deg_to_rad(self.second_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			print('a')
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 12:
			self.joint_values_req.angles = self.deg_to_rad(self.first_waving_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 13:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			print('FEITO Abrir fechar garra')
			self.get_logger().info("FINISHED MOVEMENT")	

	def place_objects_table(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
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

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.orient_to_table)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.position_values_req.pose = self.pre_place_bowl
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.place_bowl
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.position_values_req.pose = self.pos_place_bowl
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.position_values_req.pose = self.orient_to_table_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			self.position_values_req.pose = self.pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.position_values_req.pose = self.grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			self.set_gripper_req.pos = 200.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 12:
			self.position_values_req.pose = self.pos_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 13:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14:
			self.position_values_req.pose = self.pre_place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 15:
			self.position_values_req.pose = self.place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 16:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))


		elif self.estado_tr == 17:
			self.position_values_req.pose = self.pre_place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 18:
			self.position_values_req.pose = self.orient_to_table_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 19:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 20:
			self.position_values_req.pose = self.pre_grab_milk_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 21:
			self.position_values_req.pose = self.grab_milk_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 22:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 23:
			self.position_values_req.pose = self.pos_grab_milk
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 24:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 25:
			self.position_values_req.pose = self.pre_place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 26:
			self.position_values_req.pose = self.place_milk_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 27:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 28:
			self.position_values_req.pose = self.pre_place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 29:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 30:
			self.position_values_req.pose = self.grab_funilocopo
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))


		elif self.estado_tr == 31:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 32:
			self.position_values_req.pose = self.lift_funilocopo
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 33:
			self.position_values_req.pose = self.pre_pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 34:
			self.position_values_req.pose = self.pre_place_cereal_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 35:
			self.position_values_req.pose = self.place_cuttlery_table
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 36:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 37:
			self.position_values_req.pose = self.place_cuttlery_table_up 
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 38:
			self.position_values_req.pose = self.pos_place_cereal_table 
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 39:
			self.position_values_req.pose = self.last_movement
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 40:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 41:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			print('FEITO Abrir fechar garra')
			self.get_logger().info("FINISHED MOVEMENT")	

	def pick_objects_barman(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
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

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.get_order_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			time.sleep(1)
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.position_values_req.pose = self.above_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.position_values_req.pose = self.above_cuttlery_cup
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.set_gripper_req.pos = 650.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.middle_cuttlery_cup
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.position_values_req.pose = self.pos_cuttlery_cup
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 12:
			self.position_values_req.pose = self.get_order_position_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 13:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14:
			time.sleep(1)
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 15:
			self.position_values_req.pose = self.pre_place_milk_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 16:
			self.position_values_req.pose = self.grab_milk_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 17:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 18:
			self.position_values_req.pose = self.pos_grab_milk
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 19:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 20:
			self.position_values_req.pose = self.get_order_position_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 21:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 22:
			time.sleep(1)
			self.set_gripper_req.pos = 250.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 23:
			self.position_values_req.pose = self.pre_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 24:
			self.position_values_req.pose = self.grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 25:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 26:
			self.position_values_req.pose = self.pos_grab_second_object_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 27:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 28:
			self.position_values_req.pose = self.get_order_position_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 29:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 30:
			time.sleep(1)
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 31:
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position)
			self.joint_values_req.speed = 0.4 
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 32:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			print('FEITO Abrir fechar garra')
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
		elif self.next_arm_movement == "hello":
			self.hello()
		elif self.next_arm_movement == "place_objects":
			self.place_objects_table()
		elif self.next_arm_movement == "pick_objects":
			self.pick_objects_barman()

		else:
			self.wrong_movement_received = True
			print('Wrong Movement Received - ', self.next_arm_movement)	
		
def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	rclpy.spin(node)
	rclpy.shutdown()
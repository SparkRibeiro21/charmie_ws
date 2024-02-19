import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from geometry_msgs.msg import Pose, Point, Quaternion
from charmie_interfaces.msg import RobotSpeech
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

		self.create_service(SetBool, 'bool_service', self.bool_service_callback)
		print("Bool TR Service is ready")
  


		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'flag_arm_finished_movement', 10)

		# ARM TOPICS

		self.barman_or_client_subscriber = self.create_subscription(Int16, "barman_or_client", self.go_barman_or_go_client_callback, 10)
		self.speaker_publisher = self.create_publisher(RobotSpeech, "speech_command", 10)

		self.object_grabbed_publisher = self.create_publisher(Bool, "object_grabbed", 10)
	
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

		self.flag_object_grabbed = Bool()
		self.flag_object_grabbed.data = False

		self.estado_tr = 0
		self.ctr_button = 0
		self.ctr_speaker = 0
		self.processo = 0

	def bool_service_callback(self, request, response):

		if request.data:
			print("Received True. Performing an action.")
			response.success = True
			response.message = "Action Performed Sucessfully"
		else:
			print("Received False. Performing another action.")
			response.success = True
			response.message = "Action Failed"


		self.demonstration()

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

		# Velocidade do gripper varia entre 1.0 e 5000.0

		set_gripper_speed_req= SetFloat32.Request()
		set_gripper_speed_req.data = 5000.0
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

	def callback_service_tr(self, future):
		try:
			print(future.result())			
			self.estado_tr += 1
			print("ESTADO = ", self.estado_tr)
			self.demonstration()

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
			self.demonstration()


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
			print('aa')
		
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('valor da garra =', self.future)
			print('cc')

		elif self.estado_tr == 2:
			if self.gripper_tr >= 5.0:
				self.flag_object_grabbed.data = True
				print('OBJECT GRABBED')
			else:
				self.flag_object_grabbed.data = False
				print('OBJECT NOT GRABBED')

			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0

			self.object_grabbed_publisher.publish(self.flag_object_grabbed)
			self.flag_object_grabbed.data = False

			print("FEITO!!!")
			self.next_arm_movement = 999
			self.demonstration()

	def open_gripper(self):
		# aqui quero fechar, verificar se tenho algo e se tiver colocar uma flag a 1, se não tiver manter a 0. 
		# Essa flag é que me vai permitir avançar para o próximo estado ou ficar aqui e voltar ao princípio
		if self.estado_tr == 0:
			#Fechar Garra
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
			print('aa')
		
		elif self.estado_tr == 1: 
			self.future = self.get_gripper_position.call_async(self.get_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr_gripper))
			print('valor da garra =', self.future)
			print('cc')

		elif self.estado_tr == 2:
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print("FEITO!!!")
			self.next_arm_movement = 999
			self.demonstration()
		
	def open_close_gripper(self):
		if self.estado_tr == 0:
		#Fechar garra
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
			self.joint_values_req.angles = self.deg_to_rad(self.restaurant_initial_position )
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
			self.arm_finished_movement.data = True
			self.flag_arm_finish_publisher.publish(self.arm_finished_movement)
			self.arm_finished_movement.data = False
			self.estado_tr = 0
			print('FEITO Abrir fechar garra') 
			# Depois disto o robot tem de falar no restaurant e dar um led verde

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

	def demonstration(self):
		self.get_logger().info("INSIDE DEMONSTRATION")	
		print('valor vindo do pick and place: ', self.next_arm_movement)
		print('wqijhuebds')
		print('estado tr: ',self.estado_tr)
		if self.next_arm_movement == 19:
			self.open_gripper()

		elif self.next_arm_movement == 18:
				#print('estado tr: ',self.estado_tr)
			self.open_close_gripper()

		elif self.next_arm_movement == 20:
				#print('estado tr: ',self.estado_tr)
			self.verify_if_object_is_grabbed()
				#if self.flag_object_grabbed.data == True:

		else:
			print('pass - ', self.next_arm_movement)
			pass
	
		""" elif self.next_arm_movement == 10:
			self.check_object_hand()
			self.next_arm_movement = 11 """
   
		""" elif self.next_arm_movement == 11: 
			self.open_close_gripper()
			self.next_arm_movement = 12 """
		
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
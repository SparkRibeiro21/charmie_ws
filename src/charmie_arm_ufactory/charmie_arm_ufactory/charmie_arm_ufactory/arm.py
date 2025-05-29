import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from charmie_interfaces.msg import ArmController
from charmie_interfaces.srv import Trigger
from functools import partial
import math
import time


class ArmUfactory(Node):
	def __init__(self):
		super().__init__("arm_ufactory")
		self.get_logger().info("Initialised my test Node")	
		
		# THIS VALUE HAS TO BE IN "cm" 1cm = 0.01m
		# self.HEIGHT_TABLE_PLACE_OBJECTS = 91.0
		# self.HEIGHT_TABLE_PLACE_OBJECTS = 58.0
		# self.HEIGHT_TABLE_PLACE_OBJECTS = 78.0
		self.HEIGHT_TABLE_PLACE_OBJECTS = 74.0

		# ARM TOPICS
		self.arm_command_subscriber = self.create_subscription(ArmController, "arm_command", self.arm_command_callback, 10)
		self.flag_arm_finish_publisher = self.create_publisher(Bool, 'arm_finished_movement', 10)

		# ARM SERVICES
		self.set_position_client = self.create_client(MoveCartesian, '/xarm/set_position')
		self.set_tool_position_client = self.create_client(MoveCartesian, '/xarm/set_tool_position')
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

		while not self.set_tool_position_client.wait_for_service(1.0):
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
		self.next_arm_movement = "start_debug"
		self.joint_motion_values = []
		self.move_tool_line_pose = []
		self.linear_motion_pose  = []

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
		self.joint_motion_values = move.joint_motion_values
		self.move_tool_line_pose = move.move_tool_line_pose
		self.linear_motion_pose  = move.linear_motion_pose

		self.movement_selection()
		# this is used when a wrong command is received
		if self.wrong_movement_received:
			self.get_logger().error(f"NO AERM MOVEMENT NAMED: {move}")
			self.wrong_movement_received = False
			temp = Bool()
			temp.data = False
			self.flag_arm_finish_publisher.publish(temp)


	def setup(self):
		# new_put_tray
		
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


		""" self.pre_place_bowl = [-649.9, 199.0, 786.7, 0.69, 0.007, -1.56]
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
		self.place_cuttlery_table_up = [-489.1, 150.5, 593.5, 1.97, -0.808, 2.595] """


		# self.get_order_position_linear =  [-581.4, -211.5, 121.8, 2.305, 0.033, -1.52]
		# self.above_tray = [-135.0, 120.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.above_cuttlery_cup = [-135.0, 280.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.middle_cuttlery_cup = [-135.0, 345.0, -160.0, - 2.2689, 0.0, -1.57]
		# self.pos_cuttlery_cup = [-198.8, 344.9, -106.2, - 2.2689, 0.0, -1.57]
		# self.detect_objects_first_position = [-186.6, 1.9, 663.7, 1.531, 0.05, -2.407]




		# self.above_tray = 				  		[ -135.0,  120.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.above_cuttlery_cup = 				[ -135.0,  280.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.middle_cuttlery_cup = 				[ -135.0,  345.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.pos_cuttlery_cup = 					[ -198.8,  344.9, -106.2, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]


		### SEARCH FOR OBJECT ON TABLE FRONTAL JOINT VARIABLES###
		self.initial_position_joints_Pedro =			[-225.0, 83.0, -65.0, -1.0, 75.0, 270.0]
		self.search_table_front_joints =				[-215.0, -70.0, -16.0, 80.0, 30.0, 182.0]
		### SEARCH FOR OBJECT ON TABLE TOP JOINT VARIABLES###
		self.search_table_top_joints =					[-164.2, 41.7, -123.3, -94, 105.7, 280.4]

		### SERVE THE BREAKFAST VARIABLES: ###
		height_adjust = float(-(self.HEIGHT_TABLE_PLACE_OBJECTS-75.0)*10) #76.0
		print("height_adjust:", height_adjust)
		
		# SET JOINTS VARIABLES
		# self.get_order_position_joints = 				[ -161.0,   20.0,  -63.0,  256.0,   13.0,   24.0]
		self.get_lower_order_position_joints = 			[ -184.8,   17.5,  -62.0,  115.2,    4.9,  148.0]
		self.initial_position_joints = 					[ -224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		self.initial_position_joints_alternative_robocup_cornflakes = 	[ -206.0,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
		# self.pre_place_bowl_joint = 					[ -171.3,	33.6,  -98.3,   245.1,  92.4, - 15.9]
		self.pre_place_bowl_joint = 					[ -169.8,	28.0,  -81.8,   246.9,  97.9, - 6.3]
		self.pre_pick_cereals_tray_joints = 			[-193.2, 37.6, -74.9, -16.7, 126.5, 262.8]
		self.pre_pick_milk_tray_joints = 				[-203.4, 35.4, -61.2, -19.4, 74.5, 261.5]
		self.pre_pick_spoon_tray_joints = 				[-207.7, 48.4, -86.1, -21.8, 84.9, 255.2]
	
		# SET POSITIONS VARIABLES
		self.get_order_position_linear = 				[ -581.4, -211.5,  121.8, math.radians( 132.1), math.radians(   1.9), math.radians( -87.1)]
		self.get_lower_order_position_linear = 			[ -581.4,  100.0,  121.8, math.radians( 132.1), math.radians(   1.9), math.radians( -87.1)]
		# self.detect_objects_first_position_linear = 	[ -186.6,    1.9,  663.7, math.radians(  87.7), math.radians(   2.9), math.radians(-137.9)]
		self.detect_objects_first_position_linear = 	[ -178.7, -122.8,  642.0, math.radians(  90.0), math.radians(   0.0), math.radians(-140.0)]

		self.above_cuttlery_cup = 						[ -135.0,  260.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_in_cuttlery_cup =					[ -135.0,  345.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_cuttlery_cup =				[ -135.0,  260.0, -160.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		
		self.above_milk_place_spot = 					[ -230.0,  380.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		# self.place_milk_in_tray =						[ -230.0,  430.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_milk_in_tray =						[ -230.0,  445.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_milk_in_tray =				[ -230.0,  300.0,  -27.0, math.radians(-130.0), math.radians(   0.0), math.radians( -90.0)]
		
		self.above_cornflakes_place_spot = 				[ -198.0,  350.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
		self.place_cornflakes_in_tray =					[ -198.0,  420.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
		self.step_away_from_cornflakes_in_tray =		[ -198.0,  300.0,  170.0, math.radians( -90.0), math.radians(   0.0), math.radians( -90.0)]
			
		
		### ALTERNATIVE ###
		self.above_cornflakes_place_spot_alternative = 	[-232.7, -13.0, -37.8, -2.9, 51.8, 219.7]
		self.place_cornflakes_in_tray_alternative = 	[-221.6, 377.9, 107.6, math.radians(-179.0), math.radians(  -0.1), math.radians( -89.4)]
		self.step_away_from_cornflakes_in_tray_alternative = [-224.5, 378.1, 273.4, math.radians(-179.0), math.radians(  -0.1), math.radians( -89.4)] 
		
		self.pre_pick_cereals_tray_joints_cornflakes_alternative = 	[-238.9, -5.6, -69.0, -2.4, 75.5, 213.1]
		self.pick_cereals_tray_cornflakes_alternative = 			[-222.5, 377.9, 159.4, math.radians(-179.0), math.radians(-0.1), math.radians(-89.4)]							
		self.post_pick_cereals_tray_cornflakes_alternative = 		[-221.5, 277.3, 159.6, math.radians(-179.0), math.radians(-0.1), math.radians(-89.4)]							
		self.placing_cereal_at_table_cornflakes_alternative = 		[-579.5, 150.0+height_adjust, 812.9, math.radians(40.6), math.radians(4.5), math.radians(-90.1)]
		self.pos_placing_cereal_at_table_cornflakes_alternative = 	[-579.5,   0.0+height_adjust, 812.9, math.radians(40.6), math.radians(4.5), math.radians(-90.1)]

		### Test SET_TOOL_POSITION positions
		self.set_tool_position_1 = [  100.0 , 0.0,  0.0, math.radians( 0.0), math.radians( 0.0), math.radians( 0.0)]
		self.set_tool_position_2 = [ -100.0,  0.0,  0.0, math.radians( 0.0), math.radians( 0.0), math.radians( 0.0)]

		# pour positions
		self.new_cornflakes_pre_pick_tray_position = [-204.9, -51.0, -31.9, -30.2, 70.9, 253.9]
		self.new_cornflakes_pick_tray_position = [-230.0, 309.6, 146.5, math.radians(178.7), math.radians(  30.2), math.radians( -90.1)]
		# close gripper
		self.new_cornflakes_post_pick_tray_position = [-219.8, 109.7, 146.3, math.radians(178.7), math.radians(  30.2), math.radians( -90.1)]
		self.new_cornflakes_pre_pour = [-161.5, -12.8, -78.8,  84.6, -19.2, 182.0]
		self.new_cornflakes_pour1 = [-602.3, 130.7+height_adjust, 628.1, math.radians(-37.2), math.radians(  85.9), math.radians( 142.4)]
		self.new_cornflakes_pour2 = [-602.4, 131.0+height_adjust, 628.0, math.radians(-37.2), math.radians(  85.9), math.radians( 142.4)]
		self.new_cornflakes_pour3 = [-602.4, 131.0+height_adjust, 628.0, math.radians(-37.2), math.radians(  85.9), math.radians( 142.4)]
		self.new_cornflakes_pour4 = [-605.1, 148.1+height_adjust, 576.4, math.radians(-86.7), math.radians(   5.2), math.radians(  89.9)]
		
		# pour 3
		# pour 2 
		# pour 1
		# self.new_cornflakes_pre_pour


		# place positions
		self.new_cornflakes_pre_place = [-153.6,  10.2, -124.9,  95.3, -46.5, 148.7]
		self.new_cornflakes_place = [-555.4,  92.6+height_adjust, 784.8, math.radians( 77.5), math.radians(  8.4), math.radians(-112.7)]
		self.new_cornflakes_post_place = [-392.7,  30.7+height_adjust, 745.5, math.radians( 77.5), math.radians(  8.4), math.radians(-112.7)]
		
		# get lower order posiiton joints

		


		# -184.8 joints (low order)
		# -238.9 joints (pre pick cron)
		# -222.5 linear (aprox pick corn)
		# close 
		# -221.5 linear (pos pick corn)
		# 3 -184.8 joints (low order)




		# self.pre_place_bowl_linear = 					[-667.9, 149.0+height_adjust, 481.1, math.radians(59.4), math.radians(39.8), math.radians(175.8)]
		# self.place_bowl_table_final = 					[-719.1, 320.5+height_adjust, 557.6, math.radians(59.4), math.radians(39.8), math.radians(175.8)]				
		# self.little_adjustment_after_placing_bowl = 	[-709.5, 289.1+height_adjust, 543.2, math.radians(59.4), math.radians(39.8), math.radians(175.8)]
		self.pre_place_bowl_linear = 					[-619.4, 149.0+height_adjust, 426.1, math.radians(59.4), math.radians(39.8), math.radians(175.8)]
		self.place_bowl_table_final = 					[-670.6, 320.5+height_adjust, 502.6, math.radians(59.4), math.radians(39.8), math.radians(175.8)]				
		self.little_adjustment_after_placing_bowl = 	[-661.0, 290.1+height_adjust, 488.2, math.radians(59.4), math.radians(39.8), math.radians(175.8)]
  
		self.pick_cereals_tray = 						[-228.0, 420.0, 170.0, math.radians(-90.0), math.radians(0.0), math.radians(-90.0)]							
		self.pos_pick_cereals_tray = 					[-394.4, 120.0,  74.4, math.radians(173.3), math.radians(0.0), math.radians(-90.0)]
		self.strategic_avoid_possible_chair_cereals =  	[-648.7,  20.0, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		
		self.reach_position_to_pour_cereals_bowl = 		[-592.8, 220.0+height_adjust, 612.0, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		# self.pour_cereals_bowl_linear = 				[-726.2, 229.5, 611.1, math.radians(-34.7), math.radians(22.4), math.radians(119.0)]
		self.pour_cereals_bowl_linear = 				[-634.7, 226.4+height_adjust, 580.6, math.radians(-24.4), math.radians(33.4), math.radians(140.6)]
		
  
		self.after_pouring_cereals_at_bowl = 			[-606.9, 223+height_adjust, 600.9, math.radians(31.0), math.radians(27.5), math.radians(-127.5)]
		
		# self.placing_cereal_at_table = 					[-630.3, 240.0+height_adjust, 872.3, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		# self.pos_placing_cereal_at_table = 				[-630.3,  40.0+height_adjust, 872.3, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		self.placing_cereal_at_table = 					[-579.5, 230.0+height_adjust, 812.9, math.radians(40.6), math.radians(4.5), math.radians(-90.1)]
		self.pos_placing_cereal_at_table = 				[-579.5,  40.0+height_adjust, 812.9, math.radians(40.6), math.radians(4.5), math.radians(-90.1)]
  
  
		self.pos_picking_milk_tray = 					[-394.4, 120.0, 74.4, math.radians(173.3), math.radians(0.0), math.radians(-90.0)]
		self.reach_position_to_pour_milk_bowl = 		[-600.2, 220.0+height_adjust, 620.5, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		# self.pour_milk_bowl_linear = 					[-692.0, 239.0, 655.1, math.radians(-33.3), math.radians(24.6), math.radians(122.3)]
		self.pour_milk_bowl_linear = 					[-620.4, 238.7+height_adjust, 600.2, math.radians(-10.2), math.radians(39.6), math.radians(164.2)]

		self.after_pouring_milk_at_bowl = 				[-575.1, 237.7+height_adjust, 647.2, math.radians(22.5), math.radians(34.6), math.radians(-143.9)]
		# self.placing_milk_at_table = 					[-560.5, 242.9+height_adjust, 767.5, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		# self.pos_placing_milk_at_table = 				[-560.5,  40.0+height_adjust, 767.5, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		self.placing_milk_at_table = 					[-516.7, 242.9+height_adjust, 716.3, math.radians(40.5), math.radians(3.2), math.radians(-92.0)]
		self.pos_placing_milk_at_table = 				[-516.7,  40.0+height_adjust, 716.3, math.radians(40.5), math.radians(3.2), math.radians(-92.0)]

		self.pre_pick_funilocopo_tray = 				[-202.7, 430.0, -97.8, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.pick_funilocopo_tray = 					[-131.2, 430.0, -157.8, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.lift_funilocopo_a_bit_from_tray = 			[-131.2, 380.0, -157.8, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.lift_funilocopo_more =						[-394.4, 120.0, 74.4, math.radians(173.3), math.radians(0.0), math.radians( -90.0)]


		self.reach_position_to_place_spoon_table = 		[-648.7, 220.0+height_adjust, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		self.place_spoon_table_joints =         		[-455.4, 239.3+height_adjust, 459.7, math.radians(114.0), math.radians(-44.7), math.radians(147.6)]
		self.pos_place_spoon_table = 					[-648.7, 120.0+height_adjust, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
  

  		# place spoon serve breakfast with v2 of funilocopo
		self.pre_pick_funilocopo_v2_tray = 				[-208.7, 430.0, -105.0, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.pick_funilocopo_v2_tray = 					[-162.5, 430.0, -143.8, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.lift_funilocopo_v2_from_tray = 			[-162.5, 347.3, -143.8, math.radians(-130.0), math.radians(0.0), math.radians( -90.0)]
		self.joints_pre_place_table_funilocopo_v2 =		[ -181.9,    5.1,  -77.5,  177.2,   66.0,   94.2] 
		self.place_spoon_in_table_funilocopo_v2 =     	[ -610.6, 308.7+height_adjust, 641.6, math.radians(74.4), math.radians(-47.7), math.radians(-159.4)]
		self.step_away_from_table_funilocopo_v2 =     	[ -648.7, 20.0, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		
		# spoon facing the other way, as if the person is already sitting on the other side of the table
		self.joints_pre_place_table_funilocopo_v2_facing_other_side =		[ -181.9,    5.1,  -77.5,  177.2,   66.0,   269.2] 
		self.place_spoon_in_table_funilocopo_v2_facing_other_side =     	[ -591.5, 308.7+height_adjust, 648.8, math.radians(-82.2), math.radians(49.1), math.radians(3.8)]
		self.step_away_from_table_funilocopo_v2_facing_other_side =     	[ -648.7, 20.0, 677.4, math.radians(-40.5), math.radians(0.0), math.radians(90.0)]
		
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
	

	### ARM STD FUNCTIONS ###

	def set_gripper_position_(self, pos=0.0, wait=True, timeout=4.0):
		set_gripper_pos = GripperMove.Request()
		set_gripper_pos.pos = float(pos)
		set_gripper_pos.wait = wait
		set_gripper_pos.timeout = float(timeout)
		self.future = self.set_gripper.call_async(set_gripper_pos)
		self.future.add_done_callback(partial(self.callback_service_tr))

	def set_gripper_speed_(self, speed=1000.0):
		set_gripper_speed_req = SetFloat32.Request()
		set_gripper_speed_req.data = float(speed)
		self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		self.future.add_done_callback(partial(self.callback_service_tr))

	def get_gripper_position_(self):
		self.future = self.get_gripper_position.call_async(self.get_gripper_req)
		self.future.add_done_callback(partial(self.callback_service_tr_gripper))

	def set_position_values_(self, pose=None, speed=200.0, acc=1000.0, wait=True, timeout=4.0):
		
		if pose is None:
			pose = self.get_lower_order_position_linear

		position_values = MoveCartesian.Request()
		position_values.pose = pose
		position_values.speed = float(speed)
		position_values.acc = float(acc)
		position_values.wait = wait
		position_values.timeout = float(timeout)
		self.future = self.set_position_client.call_async(position_values)
		self.future.add_done_callback(partial(self.callback_service_tr))  

	def set_tool_position_values_(self, pose=None, speed=200.0, acc=1000.0, wait=True, timeout=4.0):
		
		if pose is None:
			pose = self.get_lower_order_position_linear

		position_values = MoveCartesian.Request()
		position_values.pose = pose
		position_values.speed = float(speed)
		position_values.acc = float(acc)
		position_values.wait = wait
		position_values.timeout = float(timeout)
		self.future = self.set_tool_position_client.call_async(position_values)
		self.future.add_done_callback(partial(self.callback_service_tr))  

	def set_joint_values_(self, angles=None, speed=60.0, wait=True, radius=0.0):

		if angles is None:
			angles = self.initial_position_joints

		joint_values = MoveJoint.Request()
		joint_values.angles = self.deg_to_rad(angles)
		joint_values.speed = math.radians(speed)
		joint_values.wait = wait
		joint_values.radius = float(radius)
		self.future = self.set_joint_client.call_async(joint_values)
		self.future.add_done_callback(partial(self.callback_service_tr))

	def return_if_object_is_grabbed_finish_arm_movement_(self, min_object_grabbed_position=5.0):

			temp = Bool()

			if self.gripper_tr >= min_object_grabbed_position:
				print('OBJECT GRABBED')
				temp.data = True
			else:
				print('OBJECT NOT GRABBED')
				temp.data = False

			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
		
	def finish_arm_movement_(self):
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


	### GENERIC ARM MOVEMENTS ###

	def hello(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.first_waving_position, speed=25, wait=False)
			case 1:
				self.set_gripper_position_(pos=900.0, wait=False)
			case 2:
				self.set_joint_values_(angles=self.second_waving_position, speed=25, wait=False)
			case 3:
				self.set_gripper_position_(pos=0.0, wait=False)
			case 4:
				self.set_joint_values_(angles=self.first_waving_position, speed=25, wait=False)
			case 5:
				self.set_gripper_position_(pos=900.0, wait=False)
			case 6:
				self.set_joint_values_(angles=self.second_waving_position, speed=25, wait=False)
			case 7:
				self.set_gripper_position_(pos=0.0, wait=False)
			case 8:
				self.set_joint_values_(angles=self.first_waving_position, speed=25, wait=False)
			case 9:
				self.set_gripper_position_(pos=900.0, wait=False)
			case 10:
				self.set_joint_values_(angles=self.second_waving_position, speed=25, wait=False)
			case 11:
				self.set_gripper_position_(pos=0.0, wait=False)
			case 12:
				self.set_joint_values_(angles=self.first_waving_position, speed=25, wait=False)
			case 13:
				self.set_joint_values_(angles=self.initial_position, speed=25, wait=True)
			case 14:
				self.finish_arm_movement_()

	def start_debug(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=2000)
			case 1:
				self.set_gripper_position_(pos=900.0, wait=True)
			case 2:
				self.set_joint_values_(angles=self.secondary_initial_position_debug, speed=20, wait=True)
			case 3:
				self.set_joint_values_(angles=self.initial_position, speed=20, wait=True)
			case 4:
				self.set_gripper_position_(pos=0.0, wait=False)
			case 5:
				self.get_gripper_position_()
			case 6:
				self.finish_arm_movement_()

	def start_debug_move_tool_line_test(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=2000)
			case 1:
				self.set_gripper_position_(pos=900.0, wait=True)
			case 2:
				self.set_tool_position_values_(pose=self.set_tool_position_1, speed=100, wait=True)
				# self.set_joint_values_(angles=self.secondary_initial_position_debug, speed=20, wait=True)
			case 3:
				self.set_tool_position_values_(pose=self.set_tool_position_2, speed=100, wait=True)
				# self.set_joint_values_(angles=self.initial_position, speed=20, wait=True)
			case 4:
				self.set_gripper_position_(pos=0.0, wait=False)
			case 5:
				self.get_gripper_position_()
			case 6:
				self.finish_arm_movement_()

	def initial_pose_to_ask_for_objects(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.get_lower_order_position_joints, speed=50, wait=True)
			case 1:
				self.finish_arm_movement_()

	def search_for_objects_to_ask_for_objects(self):
		match self.estado_tr:
			case 0:
				self.set_position_values_(pose=self.get_lower_order_position_linear, speed=200, wait=True)
			case 1:
				self.finish_arm_movement_()

	def initial_pose_to_search_for_objects(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.get_lower_order_position_joints, speed=50, wait=True)
			case 1:
				self.set_position_values_(pose=self.detect_objects_first_position_linear, speed=150, wait=True)
			case 2:
				self.finish_arm_movement_()

	def ask_for_objects_to_initial_position(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.initial_position_joints, speed=60, wait=True)
			case 1:
				self.finish_arm_movement_()

	def ask_for_objects_to_initial_position_alternative_robocup_cornflakes(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.initial_position_joints_alternative_robocup_cornflakes, speed=60, wait=True)
			case 1:
				self.finish_arm_movement_()

	def verify_if_object_is_grabbed(self):
		# aqui quero fechar, verificar se tenho algo e se tiver colocar uma flag a 1, se não tiver manter a 0. 
		# Essa flag é que me vai permitir avançar para o próximo estado ou ficar aqui e voltar ao princípio
		match self.estado_tr:
			case 0:
				self.set_gripper_position_(pos=0.0, wait=True)
			case 1:
				self.get_gripper_position_()
			case 2:
				self.return_if_object_is_grabbed_finish_arm_movement_(min_object_grabbed_position=5.0)

	def close_gripper(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=1000)
			case 1:
				self.set_gripper_position_(pos=0.0, wait=True)
			case 2:
				self.finish_arm_movement_()

	def close_gripper_with_check_object(self, min_value):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=1000)
			case 1:
				self.set_gripper_position_(pos=min_value, wait=True)
			case 2:
				self.get_gripper_position_()
			case 3:
				self.return_if_object_is_grabbed_finish_arm_movement_(min_object_grabbed_position=min_value+5.0)

	def open_gripper(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=5000)
			case 1:
				self.set_gripper_position_(pos=900, wait=True)
			case 2:
				self.finish_arm_movement_()

	def adjust_linear_motion(self):
		match self.estado_tr:
			case 0:
				self.set_position_values_(pose=self.linear_motion_pose, speed=50, wait=True)
			case 1:
				self.finish_arm_movement_()

	def adjust_move_tool_line(self):
		match self.estado_tr:
			case 0:
				self.set_tool_position_values_(pose=self.move_tool_line_pose, speed=50, wait=True)
			case 1:
				self.finish_arm_movement_()

	def adjust_joint_motion(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.joint_motion_values, speed=25, wait=True)
			case 1:
				self.finish_arm_movement_()

	### SEARCH FOR OBJECT ON TABLE FRONTAL###
	def initial_pose_to_search_table_front(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=5000)
			case 1:
				self.set_gripper_position_(pos=0, wait=True)
			case 2:
				self.set_joint_values_(angles=self.initial_position_joints_Pedro, speed=30, wait=True)
			case 3:
				self.set_joint_values_(angles=self.search_table_front_joints, speed=30, wait=True)
			case 4:
				self.finish_arm_movement_()

	def search_table_to_initial_pose(self):
		match self.estado_tr:
			case 0:
				self.set_joint_values_(angles=self.initial_position_joints_Pedro, speed=30, wait=True)
			case 1:
				self.finish_arm_movement_()

	### SEARCH FOR OBJECT ON TABLE TOP###
	def initial_pose_to_search_table_top(self):
		match self.estado_tr:
			case 0:
				self.set_gripper_speed_(speed=5000)
				self.set_gripper_position_(pos=0, wait=True)
			case 1:
				self.set_joint_values_(angles=self.initial_position_joints_Pedro, speed=50, wait=True)
			case 2:
				self.set_joint_values_(angles=self.search_table_top_joints, speed=10, wait=True)
			case 3:
				self.finish_arm_movement_()



	### SERVE THE BREAKFAST ARM MOVEMENTS ###

	def collect_spoon_to_tray(self):

		if self.estado_tr == 0:
			self.position_values_req.pose = self.above_cuttlery_cup
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_in_cuttlery_cup
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.step_away_from_cuttlery_cup
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 5:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.position_values_req.pose = self.get_lower_order_position_linear
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def collect_milk_to_tray(self):

		if self.estado_tr == 0:
			self.position_values_req.pose = self.above_milk_place_spot
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_milk_in_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.step_away_from_milk_in_tray
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 5:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.position_values_req.pose = self.get_lower_order_position_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def collect_cornflakes_to_tray(self):

		if self.estado_tr == 0:
			self.position_values_req.pose = self.above_cornflakes_place_spot
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_cornflakes_in_tray
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.step_away_from_cornflakes_in_tray
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 5:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.position_values_req.pose = self.get_lower_order_position_linear
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")

	def collect_cornflakes_to_tray_alternative_robocup_cornflakes(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.above_cornflakes_place_spot_alternative)
			self.joint_values_req.speed = math.radians(60)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_cornflakes_in_tray_alternative # -221.6
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.step_away_from_cornflakes_in_tray_alternative # -224.5
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 5:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
					
		elif self.estado_tr == 7:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(60)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")


	def collect_bowl_to_initial_position(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position_joints)
			self.joint_values_req.speed = math.radians(60)
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


	### PLACE OBJECTS ON THE TABLE

	def place_bowl_table(self):
		# if self.estado_tr == 0:
		# 	self.joint_values_req.angles = self.deg_to_rad(self.initial_position_joints)
		# 	self.joint_values_req.speed = math.radians(60)
		# 	self.joint_values_req.wait = True
		# 	self.joint_values_req.radius = 0.0
		# 	self.future = self.set_joint_client.call_async(self.joint_values_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
   
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_bowl_joint)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_bowl_table_final
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 2000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 500.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.position_values_req.pose = self.little_adjustment_after_placing_bowl
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 5:
			self.set_gripper_req.pos = 80.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.set_gripper_req.pos = 850.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 7:
			self.position_values_req.pose = self.pre_place_bowl_linear
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 8:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def pour_cereals_bowl(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_cereals_tray_joints)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.get_logger().warning("FINISH MOVE TO CEREAL")
			self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 1:
		# 	set_gripper_speed_req= SetFloat32.Request()
		# 	set_gripper_speed_req.data = 5000.0
		# 	self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 850.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  1:
			self.position_values_req.pose = self.pick_cereals_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.get_logger().warning("START DESCEND TO CEREAL")
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 200.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  4:
			self.position_values_req.pose = self.pos_pick_cereals_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  5:
			self.position_values_req.pose = self.strategic_avoid_possible_chair_cereals
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  6:
			self.position_values_req.pose = self.reach_position_to_pour_cereals_bowl
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  7:
			self.position_values_req.pose = self.pour_cereals_bowl_linear
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.after_pouring_cereals_at_bowl
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 9:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def pour_cereals_bowl_alternative_robocup_cornflakes(self):


		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 1:
			self.joint_values_req.angles = self.deg_to_rad(self.new_cornflakes_pre_pick_tray_position)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  2:
			self.position_values_req.pose = self.new_cornflakes_pick_tray_position
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
  
		elif self.estado_tr ==  5:
			self.position_values_req.pose = self.new_cornflakes_post_pick_tray_position
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.joint_values_req.angles = self.deg_to_rad(self.new_cornflakes_pre_pour)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr ==  7:
			self.position_values_req.pose = self.new_cornflakes_pour1
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  8:
			self.position_values_req.pose = self.new_cornflakes_pour2
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  9:
			self.position_values_req.pose = self.new_cornflakes_pour3
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  10:
			self.position_values_req.pose = self.new_cornflakes_pour4
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  11:
			self.position_values_req.pose = self.new_cornflakes_pour3
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  12:
			self.position_values_req.pose = self.new_cornflakes_pour2
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  13:
			self.position_values_req.pose = self.new_cornflakes_pour1
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 14:
			self.joint_values_req.angles = self.deg_to_rad(self.new_cornflakes_pre_pour)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 15:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def place_cereal_table(self):   

		if self.estado_tr ==  0:
			self.position_values_req.pose = self.placing_cereal_at_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 1:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 400.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			self.set_gripper_req.pos = 850.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 3:
			self.position_values_req.pose = self.pos_placing_cereal_at_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 4:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	

	def place_cereal_table_alternative_robocup_cornflakes(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.new_cornflakes_pre_place)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  1:
			self.position_values_req.pose = self.new_cornflakes_place
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 850.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  4:
			self.position_values_req.pose = self.new_cornflakes_post_place
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(50)
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
	

	def pour_milk_bowl(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_milk_tray_joints)
			self.joint_values_req.speed = math.radians(60)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		# elif self.estado_tr ==  1:
		# 	set_gripper_speed_req= SetFloat32.Request()
		# 	set_gripper_speed_req.data = 5000.0
		# 	self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 850.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr ==  1:
			self.position_values_req.pose = self.place_milk_in_tray
			self.position_values_req.speed = 150.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  4:
			self.position_values_req.pose = self.pos_picking_milk_tray
			self.position_values_req.speed = 99.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  5:
			self.position_values_req.pose = self.strategic_avoid_possible_chair_cereals
			self.position_values_req.speed = 99.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  6:
			self.position_values_req.pose = self.reach_position_to_pour_milk_bowl
			self.position_values_req.speed = 99.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr ==  7:
			self.position_values_req.pose = self.pour_milk_bowl_linear
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.after_pouring_milk_at_bowl
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 9:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def place_milk_table(self):   
		if self.estado_tr == 0:
			self.position_values_req.pose = self.placing_milk_at_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 1:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 600.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			self.set_gripper_req.pos = 850.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 3:
			self.position_values_req.pose = self.pos_placing_milk_at_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 4:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
	
	def place_spoon_table(self):
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_spoon_tray_joints)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		# elif self.estado_tr == 1:
		# 	set_gripper_speed_req= SetFloat32.Request()
		# 	set_gripper_speed_req.data = 5000.0
		# 	self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 850.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.pre_pick_funilocopo_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			self.position_values_req.pose = self.pick_funilocopo_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 3:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 5:
			self.position_values_req.pose = self.lift_funilocopo_a_bit_from_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.position_values_req.pose = self.lift_funilocopo_more
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 7:
			self.position_values_req.pose = self.strategic_avoid_possible_chair_cereals
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.reach_position_to_place_spoon_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 9:
			self.position_values_req.pose = self.place_spoon_table_joints
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		# elif self.estado_tr == 10:
		# 	self.position_values_req.pose = self.small_up_after_drop_spoon_table
		# 	self.position_values_req.speed = 150.0
		# 	self.position_values_req.acc = 1000.0
		# 	self.position_values_req.wait = True
		# 	self.position_values_req.timeout = 14.0
		# 	self.future = self.set_position_client.call_async(self.position_values_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.position_values_req.pose = self.pos_place_spoon_table
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 11:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def place_spoon_table_funilocopo_v2(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_spoon_tray_joints)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 1:
		# 	set_gripper_speed_req= SetFloat32.Request()
		# 	set_gripper_speed_req.data = 5000.0
		# 	self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 850.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 1:
		# 	self.position_values_req.pose = self.pre_pick_funilocopo_v2_tray
		# 	self.position_values_req.speed = 120.0
		# 	self.position_values_req.acc = 1000.0
		# 	self.position_values_req.wait = True
		#	self.position_values_req.timeout = 4.0
		#	self.future = self.set_position_client.call_async(self.position_values_req)
		#	self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 1:
			self.position_values_req.pose = self.pick_funilocopo_v2_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 4:
			self.position_values_req.pose = self.lift_funilocopo_v2_from_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.joints_pre_place_table_funilocopo_v2)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.position_values_req.pose = self.place_spoon_in_table_funilocopo_v2
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.set_gripper_req.pos = 300.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.step_away_from_table_funilocopo_v2
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def place_spoon_table_funilocopo_v2_facing_other_side(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_pick_spoon_tray_joints)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 1:
		# 	set_gripper_speed_req= SetFloat32.Request()
		# 	set_gripper_speed_req.data = 5000.0
		# 	self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 2:
		# 	self.set_gripper_req.pos = 850.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

		# elif self.estado_tr == 1:
		# 	self.position_values_req.pose = self.pre_pick_funilocopo_v2_tray
		# 	self.position_values_req.speed = 120.0
		# 	self.position_values_req.acc = 1000.0
		# 	self.position_values_req.wait = True
		# 	self.position_values_req.timeout = 4.0
		# 	self.future = self.set_position_client.call_async(self.position_values_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 1:
			self.position_values_req.pose = self.pick_funilocopo_v2_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
   
		elif self.estado_tr == 4:
			self.position_values_req.pose = self.lift_funilocopo_v2_from_tray
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.joints_pre_place_table_funilocopo_v2_facing_other_side)
			self.joint_values_req.speed = math.radians(70)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 6:
			self.position_values_req.pose = self.place_spoon_in_table_funilocopo_v2_facing_other_side
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.set_gripper_req.pos = 300.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 8:
			self.position_values_req.pose = self.step_away_from_table_funilocopo_v2_facing_other_side
			self.position_values_req.speed = 120.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 9:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 10:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = False
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 11:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def arm_go_rest(self):
		# if self.estado_tr == 0:
		# 	self.set_gripper_req.pos = 0.0
		# 	self.set_gripper_req.wait = False
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))
			
		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = math.radians(60)
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def movement_selection(self):
		
		# self.get_logger().info("INSIDE MOVEMENT_SELECTION")	
		print('valor vindo do pick and place: ', self.next_arm_movement)
		
		match self.next_arm_movement:
			
			# GENERIC
			case "start_debug":
				self.start_debug()
				# self.start_debug_move_tool_line_test()

			case  "hello":
				self.hello()

			case "initial_pose_to_ask_for_objects":
				self.initial_pose_to_ask_for_objects()
			case "ask_for_objects_to_initial_position":
				self.ask_for_objects_to_initial_position()
			case "initial_pose_to_search_for_objects":
				self.initial_pose_to_search_for_objects()
			case "search_for_objects_to_ask_for_objects":
				self.search_for_objects_to_ask_for_objects()

			case "arm_go_rest":
				self.arm_go_rest()
			case "verify_if_object_is_grabbed":
				self.verify_if_object_is_grabbed()
			case "close_gripper":
				self.close_gripper()
			case "close_gripper_with_check_object":
				self.close_gripper_with_check_object(0)
			case "open_gripper":
				self.open_gripper()

			# ADJUSTS MOVEMENTS FROM ARMCONTROLLER
			case "adjust_linear_motion":
				self.adjust_linear_motion()
			case "adjust_move_tool_line":
				self.adjust_move_tool_line()
			case "adjust_joint_motion":
				self.adjust_joint_motion()

			# SERVE BREAKFAST	
			case "place_bowl_table":
				self.place_bowl_table()
			case "pour_cereals_bowl":
				self.pour_cereals_bowl()
			case "place_cereal_table":
				self.place_cereal_table()
			case "pour_cereals_bowl_alternative_robocup_cornflakes":
				self.pour_cereals_bowl_alternative_robocup_cornflakes()
			case "place_cereal_table_alternative_robocup_cornflakes":
				self.place_cereal_table_alternative_robocup_cornflakes()
			case "pour_milk_bowl":
				self.pour_milk_bowl()
			case "place_milk_table":
				self.place_milk_table()
			case "place_spoon_table":
				self.place_spoon_table()
			case "place_spoon_table_funilocopo_v2":
				self.place_spoon_table_funilocopo_v2()
			case "place_spoon_table_funilocopo_v2_facing_other_side":
				self.place_spoon_table_funilocopo_v2_facing_other_side()
			case "collect_spoon_to_tray":
				self.collect_spoon_to_tray()
			case "collect_milk_to_tray":
				self.collect_milk_to_tray()
			case "collect_cornflakes_to_tray":
				self.collect_cornflakes_to_tray()
			case "collect_cornflakes_to_tray_alternative_robocup_cornflakes":
				self.collect_cornflakes_to_tray_alternative_robocup_cornflakes()
			case "collect_bowl_to_initial_position":
				self.collect_bowl_to_initial_position()
			case "close_gripper_with_check_object_cornflakes":
				self.close_gripper_with_check_object(200)
			case "ask_for_objects_to_initial_position_alternative_robocup_cornflakes":
				self.ask_for_objects_to_initial_position_alternative_robocup_cornflakes()

			# SEARCH FOR OBJECT ON TABLE FRONTAL
			case "initial_pose_to_search_table_front":
				self.initial_pose_to_search_table_front()
			case "search_table_to_initial_pose":
				self.search_table_to_initial_pose()

			# SEARCH FOR OBJECT ON TABLE TOP
			case "initial_pose_to_search_table_top":
				self.initial_pose_to_search_table_top()
			
			# if there is an error regarding a movement
			case _:
				self.wrong_movement_received = True
				print('Wrong Movement Received - ', self.next_arm_movement)	
			
def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	rclpy.spin(node)
	rclpy.shutdown()
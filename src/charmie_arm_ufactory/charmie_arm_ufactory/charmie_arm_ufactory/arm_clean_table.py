import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, String
from xarm_msgs.srv import MoveCartesian, MoveJoint, SetInt16ById, SetInt16, GripperMove, GetFloat32, SetTcpLoad, SetFloat32, PlanPose, PlanExec, PlanJoint
from charmie_interfaces.msg import ArmController
from charmie_interfaces.srv import ArmTrigger
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
		self.HEIGHT_TABLE_PLACE_OBJECTS = 76.0


		self.HEIGHT_TOP_DISHWASHER_RACK = 60.0
		self.HEIGHT_BOTTOM_DISHWASHER_RACK = 25.0


		# ARM TOPICS
		self.arm_command_subscriber = self.create_subscription(ArmController, "arm_command", self.arm_command_callback, 10)
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


		### SERVE THE BREAKFAST VARIABLES: ###
		height_adjust = float(-(self.HEIGHT_TABLE_PLACE_OBJECTS-76.0)*10)
		print("height_adjust:", height_adjust)
		
		# SET JOINTS VARIABLES
		# self.get_order_position_joints = 				[ -161.0,   20.0,  -63.0,  256.0,   13.0,   24.0]
		self.get_lower_order_position_joints = 			[ -184.9,   17.5,  -62.0,  115.2,    4.9,  148.0]
		self.initial_position_joints = 					[ -224.8,   83.4,  -65.0,   -0.5,   74.9,  270.0] 
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

		# 4
		self.reach_position_to_place_spoon_table = 		[-648.7, 220.0+height_adjust, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
		
		
		
		
		# 5
		self.place_spoon_table_joints = 				[-455.4, 239.3+height_adjust, 459.7, math.radians(104.1), math.radians(-48.0), math.radians(161.3)]
		# 6
		# self.small_up_after_drop_spoon_table = 			[-501.6, 209.3+height_adjust, 513.9, math.radians(122.1), math.radians(-40.0), math.radians(135.7)]
		# 7
		self.pos_place_spoon_table = 					[-648.7, 120.0+height_adjust, 677.4, math.radians(40.5), math.radians(0.0), math.radians(-90.0)]
  

		### CLEAN THE TABLE VARIABLES!!! ###
		height_top_rack    = float(-(self.HEIGHT_TOP_DISHWASHER_RACK-60.0)*10)
		height_bottom_rack_torso_up = float(-(self.HEIGHT_BOTTOM_DISHWASHER_RACK+14.0-25.0)*10)
		height_bottom_rack_torso_down = float(-(self.HEIGHT_BOTTOM_DISHWASHER_RACK-25.0)*10)
		
		
		
		print("height_adjust:", height_adjust)
		self.pre_dishwasher =  			[ -273.9,  -94.8,  -20.4,    0.3,   22.1,  270.0]
		
		self.pre_place_cup =  			[ -298.4,  -44.1, -100.2,   30.0,   55.2,  254.5]
		self.place_cup = 				[   25.5,  393.3+height_top_rack,  924.8, math.radians(  86.6), math.radians(  -0.7), math.radians( 177.7)]
	

		self.close_rack_place_plate_rack_height = 		[   21.0,  393.3+height_top_rack,  605.3, math.radians(  86.6), math.radians(  -0.7), math.radians( 177.7)]
		self.close_rack_first_rack_push = 				[   24.0,  365.1+height_top_rack,  914.5, math.radians(  86.6), math.radians(  -0.7), math.radians( 177.7)]
		self.pre_dishwasher_perpendicular =  			[ -273.9,  -94.8,  -20.4,  180.7,   64.5,  180.0]
		self.close_rack_second_push_pre_adjust_height = [   47.7,  430.8+height_top_rack,  729.5, math.radians(  -2.6), math.radians(   0.4), math.radians( -94.4)]
		self.close_rack_second_push = 					[   55.3,  392.0+height_top_rack,  966.5, math.radians(  -2.6), math.radians(   0.4), math.radians( -94.4)]
		self.close_rack_second_push_back = 				[   52.4,  392.0+height_top_rack,  901.3, math.radians(  -2.6), math.radians(   0.4), math.radians( -94.4)]
		






  
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
			self.joint_values_req.speed = math.radians(30) 
			self.joint_values_req.wait = False
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.initial_position)
			self.joint_values_req.speed = math.radians(30) 
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

	### ACTUAL SERVE THE BREAKFAST ARM MOVEMENTS ###

	def search_for_objects(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(50)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.position_values_req.pose = self.detect_objects_first_position_linear
			self.position_values_req.speed = 150.0
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


	### ACTUAL SERVE THE BREAKFAST ARM MOVEMENTS ###

	def initial_pose_to_ask_for_objects(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(40)
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

	def search_for_objects_to_ask_for_objects(self):

		if self.estado_tr == 0:
			self.position_values_req.pose = self.get_lower_order_position_linear
			self.position_values_req.speed = 200.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 4.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

	def ask_for_objects_to_initial_position(self):

		# if self.estado_tr == 0:
		# 	self.set_gripper_req.pos = 0.0
		# 	self.set_gripper_req.wait = True
		# 	self.set_gripper_req.timeout = 4.0
		# 	self.future = self.set_gripper.call_async(self.set_gripper_req)
		# 	self.future.add_done_callback(partial(self.callback_service_tr))

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

	def close_gripper_with_check_object(self, min_value):

		if self.estado_tr == 0:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 1000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 1:
			self.set_gripper_req.pos = float(min_value)
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

			if self.gripper_tr >= min_value+5:
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


	def pre_dishwasher_to_ask_for_objects(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.get_lower_order_position_joints)
			self.joint_values_req.speed = math.radians(40)
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


	def ask_for_objects_to_pre_dishwasher(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_dishwasher)
			self.joint_values_req.speed = math.radians(40)
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


	def place_cup_in_dishwasher(self):

		if self.estado_tr == 0:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_cup)
			self.joint_values_req.speed = math.radians(30)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 1:
			self.position_values_req.pose = self.place_cup
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 2:
			set_gripper_speed_req= SetFloat32.Request()
			set_gripper_speed_req.data = 5000.0
			self.future = self.set_gripper_speed.call_async(set_gripper_speed_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 4:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_place_cup)
			self.joint_values_req.speed = math.radians(30)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 5:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_dishwasher)
			self.joint_values_req.speed = math.radians(30)
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


	def place_plate_in_dishwasher(self):

		if self.estado_tr == 0:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		if self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def place_bowl_in_dishwasher(self):

		if self.estado_tr == 0:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		if self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	

		

	def place_cutlery_in_dishwasher(self):

		if self.estado_tr == 0:
			self.set_gripper_req.pos = 900.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		elif self.estado_tr == 1:
			self.set_gripper_req.pos = 0.0
			self.set_gripper_req.wait = True
			self.set_gripper_req.timeout = 4.0
			self.future = self.set_gripper.call_async(self.set_gripper_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		if self.estado_tr == 2:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	
		

	def open_dishwasher_rack(self):

		if self.estado_tr == 0:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def close_dishwasher_rack(self):

		if self.estado_tr == 0:
			self.position_values_req.pose = self.close_rack_place_plate_rack_height
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))
		
		if self.estado_tr == 1:
			self.position_values_req.pose = self.close_rack_first_rack_push
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 2:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_dishwasher)
			self.joint_values_req.speed = math.radians(30)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 3:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_dishwasher_perpendicular)
			self.joint_values_req.speed = math.radians(30)
			self.joint_values_req.wait = True
			self.joint_values_req.radius = 0.0
			self.future = self.set_joint_client.call_async(self.joint_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 4:
			self.position_values_req.pose = self.close_rack_second_push_pre_adjust_height
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 5:
			self.position_values_req.pose = self.close_rack_second_push
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		if self.estado_tr == 6:
			self.position_values_req.pose = self.close_rack_second_push_back
			self.position_values_req.speed = 100.0
			self.position_values_req.acc = 1000.0
			self.position_values_req.wait = True
			self.position_values_req.timeout = 14.0
			self.future = self.set_position_client.call_async(self.position_values_req)
			self.future.add_done_callback(partial(self.callback_service_tr))

		elif self.estado_tr == 7:
			self.joint_values_req.angles = self.deg_to_rad(self.pre_dishwasher)
			self.joint_values_req.speed = math.radians(30)
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


	def open_dishwasher_door(self):

		if self.estado_tr == 0:
			temp = Bool()
			temp.data = True
			self.flag_arm_finish_publisher.publish(temp)
			self.estado_tr = 0
			self.get_logger().info("FINISHED MOVEMENT")	


	def close_dishwasher_door(self):

		if self.estado_tr == 0:
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
			""" elif self.next_arm_movement == "hello":
			self.hello()
		elif self.next_arm_movement == "place_objects":
			self.place_objects_table()
		elif self.next_arm_movement == "pick_objects":
			self.pick_objects_barman() """
   
		elif self.next_arm_movement == "place_bowl_table":
			self.place_bowl_table()
		elif self.next_arm_movement == "pour_cereals_bowl":
			self.pour_cereals_bowl()
		elif self.next_arm_movement == "place_cereal_table":
			self.place_cereal_table()
		elif self.next_arm_movement == "pour_milk_bowl":
			self.pour_milk_bowl()
		elif self.next_arm_movement == "place_milk_table":
			self.place_milk_table()
		elif self.next_arm_movement == "place_spoon_table":
			self.place_spoon_table()
		elif self.next_arm_movement == "arm_go_rest":
			self.arm_go_rest()

		# new serve breakfast functions
		elif self.next_arm_movement == "search_for_objects":
			self.search_for_objects()
		elif self.next_arm_movement == "search_for_objects_to_ask_for_objects":
			self.search_for_objects_to_ask_for_objects()
		
		# new
		elif self.next_arm_movement == "initial_pose_to_ask_for_objects":
			self.initial_pose_to_ask_for_objects()
		# elif self.next_arm_movement == "collect_milk_to_tray2":
		# 	self.collect_milk_to_tray2()

		elif self.next_arm_movement == "ask_for_objects_to_initial_position":
			self.ask_for_objects_to_initial_position()
		elif self.next_arm_movement == "verify_if_object_is_grabbed":
			self.verify_if_object_is_grabbed()
		elif self.next_arm_movement == "close_gripper":
			self.close_gripper()
		elif self.next_arm_movement == "close_gripper_with_check_object":
			self.close_gripper_with_check_object(0)
		elif self.next_arm_movement == "close_gripper_with_check_object_cornflakes":
			self.close_gripper_with_check_object(200)
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
		

		# CLEAN THE TABLE POSITIONS
		elif self.next_arm_movement == "ask_for_objects_to_pre_dishwasher":
			self.ask_for_objects_to_pre_dishwasher()
		elif self.next_arm_movement == "pre_dishwasher_to_ask_for_objects":
			self.pre_dishwasher_to_ask_for_objects()
		elif self.next_arm_movement == "place_cup_in_dishwasher":
			self.place_cup_in_dishwasher()
		elif self.next_arm_movement == "place_plate_in_dishwasher":
			self.place_plate_in_dishwasher()
		elif self.next_arm_movement == "place_bowl_in_dishwasher":
			self.place_bowl_in_dishwasher()
		elif self.next_arm_movement == "place_cutlery_in_dishwasher":
			self.place_cutlery_in_dishwasher()
		elif self.next_arm_movement == "open_dishwasher_rack":
			self.open_dishwasher_rack()
		elif self.next_arm_movement == "close_dishwasher_rack":
			self.close_dishwasher_rack()
		elif self.next_arm_movement == "open_dishwasher_door":
			self.open_dishwasher_door()
		elif self.next_arm_movement == "close_dishwasher_door":
			self.close_dishwasher_door()


		else:
			self.wrong_movement_received = True
			print('Wrong Movement Received - ', self.next_arm_movement)	
		
def main(args=None):
	rclpy.init(args=args)
	node = ArmUfactory()
	rclpy.spin(node)
	rclpy.shutdown()
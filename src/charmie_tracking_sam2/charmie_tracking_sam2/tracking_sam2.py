#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD
from charmie_interfaces.msg import TrackingMask, ListOfPoints, BoundingBox, MaskDetection, ListOfMaskDetections
from charmie_interfaces.srv import ActivateTracking

import threading
import cv2
import torch
from sam2.build_sam import build_sam2_camera_predictor
from pathlib import Path
from cv_bridge import CvBridge
import numpy as np
import time
import math

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from charmie_point_cloud.point_cloud_class import PointCloud

data_lock = threading.Lock()

class TrackingNode(Node):

    def __init__(self):
        super().__init__('Tracking')
        self.get_logger().info("Initialised CHARMIE Tracking Node")

        # Tracking Variables Initialisation
        self.tracking_flag = False
        self.calibration_mode = False # Calibrations starts as true so it is executed right at the beggining

        self.tracking_received_points = []
        self.tracking_received_labels = []
        self.tracking_received_bbox = []

        self.br = CvBridge()
        self.head_rgb = Image()
        self.head_depth = Image()
        self.new_head_rgb = False
        self.new_head_depth = False
        
        self.CAM_IMAGE_WIDTH = 848
        self.CAM_IMAGE_HEIGHT = 480

        self.head_rgb_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH, 3), np.uint8)
        self.head_depth_cv2_frame = np.zeros((self.CAM_IMAGE_HEIGHT, self.CAM_IMAGE_WIDTH), np.uint8)

        ### Topics ###
        # Intel Realsense
        self.rgbd_head_subscriber = self.create_subscription(RGBD, "/CHARMIE/D455_head/rgbd", self.get_rgbd_head_callback, 10)
        # Publica a máscara do objecto a ser seguido
        self.tracking_mask_publisher = self.create_publisher(TrackingMask, 'tracking_mask', 10)
        
        # SERVICES:
        # Acitvate and Deactivate Tracking Service
        self.activate_yolo_objects_service = self.create_service(ActivateTracking, "activate_tracking", self.callback_activate_tracking)

        ### TF buffer and listener ###
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ### Class ###
        self.point_cloud = PointCloud()


    def callback_activate_tracking(self, request, response):
        
        # Type of service received: 
        # bool activate            # activate or deactivate tracking system
        # ListOfPoints points      # list of points to track
        # BoundingBox bounding_box # bounding box to track
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        
        self.get_logger().info("Activate Tracking Service: %s" %("("+str(request.activate)+", "
                                                                    +str(request.points)+", "
                                                                    +str(request.bounding_box)+")"))
        
        self.tracking_received_points.clear()
        self.tracking_received_labels.clear()
        self.tracking_received_bbox.clear()

        for point in request.points.coords:
            self.tracking_received_points.append((int(point.x), int(point.y)))
            self.tracking_received_labels.append(int(point.z))


        if request.bounding_box.box_width > 0.0 and request.bounding_box.box_height > 0.0:
            self.tracking_received_bbox = [request.bounding_box.box_top_left_x, request.bounding_box.box_top_left_y, \
                                           request.bounding_box.box_top_left_x + request.bounding_box.box_width, \
                                           request.bounding_box.box_top_left_y + request.bounding_box.box_height]
                                
        # print("===", self.tracking_received_points, self.tracking_received_labels)

        self.calibration_mode = True
        self.tracking_flag = request.activate

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response
    
    def get_rgbd_head_callback(self, rgbd: RGBD):
        with data_lock: 
            self.head_rgb = rgbd.rgb
            self.head_rgb_cv2_frame = self.br.imgmsg_to_cv2(rgbd.rgb, "bgr8")
            self.head_depth = rgbd.depth
            self.head_depth_cv2_frame = self.br.imgmsg_to_cv2(rgbd.depth, "passthrough")
        self.new_head_rgb = True
        self.new_head_depth = True
        # print("Head (h,w):", rgbd.rgb_camera_info.height, rgbd.rgb_camera_info.width, rgbd.depth_camera_info.height, rgbd.depth_camera_info.width)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    th_main = threading.Thread(target=ThreadMainTracking, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def ThreadMainTracking(node: TrackingNode):
    main = TrackingMain(node)
    main.main()

class TrackingMain():

    def __init__(self, node: TrackingNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node

        home = str(Path.home())

        # Define model and configurations
        sam2_checkpoint = home+"/sam2/checkpoints/sam2.1_hiera_small.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"
        self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)

        self.initial_obj_id = 1  # Object ID for tracking
        
        self.DEBUG_DRAW = False

        self.prev_frame_time = time.time() # used to record the time when we processed last frame
        self.new_frame_time = time.time() # used to record the time at which we processed current frame
        
    def get_transform(self, camera=""):

        match camera:
            case "head":
                child_link = 'D455_head_color_frame'
                parent_link = 'base_footprint'
            case "hand":
                child_link = 'D405_hand_color_frame'
                parent_link = 'base_footprint'
            case "base":
                child_link = 'camera_color_frame'
                parent_link = 'base_footprint'
            case "":
                child_link = 'base_footprint'
                parent_link = 'map'

        # proceed to lookup_transform
        if self.node.tf_buffer.can_transform(parent_link, child_link, rclpy.time.Time()):
            
            # print(parent_link, child_link, "GOOD")
            try:
                transform = self.node.tf_buffer.lookup_transform(
                    parent_link,        # target frame
                    child_link,         # source frame
                    rclpy.time.Time()   # latest available
                    # timeout=rclpy.duration.Duration(seconds=0.1) quero por isto???
                )
            except Exception as e:
                self.node.get_logger().warn(f"TF lookup failed: {e}")
                transform = None
                return  # or handle the error appropriately
        else:
            # print(parent_link, child_link, "BAD")
            transform = None
        
        return transform, child_link

    def combined_polygon_centroid(self, polygons, binary_mask):
        
        MIN_ACCEPTABLE_AREA = 100
        total_area = 0
        centroid_x = 0
        centroid_y = 0
        area_of_each_polygon = []
        centroid_of_each_polygon = []
        updated_filtered_polygons = []

        # print(binary_mask)

        for polygon_points in polygons:
            points = np.array(polygon_points, dtype=np.float32)
            
            # Skip invalid polygons
            if len(points) < 3:
                continue
            if not np.array_equal(points[0], points[-1]):
                points = np.vstack([points, points[0]])
            
            x = points[:, 0]
            y = points[:, 1]
            A = 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]) # using the abolute value, because depending on the orientation (CW or CCW) the area may be returned as negative
            
            # Skip polygons with zero area
            if abs(A) < MIN_ACCEPTABLE_AREA:
                continue

            Cx = np.sum((x[:-1] + x[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)
            Cy = np.sum((y[:-1] + y[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)

            # Convert centroid coordinates to binary mask space
            Cx_ = max(0, min(binary_mask.shape[1] - 1, int(Cx + 0.5)))
            Cy_ = max(0, min(binary_mask.shape[0] - 1, int(Cy + 0.5)))
            
            # Access binary mask value
            mask_value = binary_mask[Cy_, Cx_]  # Note: mask is accessed as [row, col]

            # print(f"Centroid: ({Cx_}, {Cy_}), Mask Value: {mask_value}, Area: {abs(A)}")

            # Remove all polygons that are not part of the object being tracked (check if centroid is part of image mask)
            if mask_value == 0:
                continue
            
            updated_filtered_polygons.append(polygon_points)
            area_of_each_polygon.append(abs(A))
            centroid_of_each_polygon.append((Cx, Cy))
            
            total_area += A
            centroid_x += Cx * A
            centroid_y += Cy * A

        if total_area == 0:
            return None, None, None, None  # No valid polygons
        
        # Final combined centroid
        combined_Cx = centroid_x / total_area
        combined_Cy = centroid_y / total_area
        combined_C = (combined_Cx, combined_Cy)
        return combined_C, updated_filtered_polygons, area_of_each_polygon, centroid_of_each_polygon
    
    def filter_and_publish_tracking_data(self, polygons, binary_mask, depth_frame):

        if polygons:
            centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon = self.combined_polygon_centroid(polygons, binary_mask)
            # print(area_each_polygon, updated_filtered_polygons)
            if centroid is not None:
                
                msg = TrackingMask()
                msg.centroid.x = float(centroid[0])
                msg.centroid.y = float(centroid[1])
                
                list_masks = ListOfMaskDetections()
                list_masks_for_pc = []
                
                for p in updated_filtered_polygons: # only goes through filtres polygons, rather than all polygons

                    new_mask = MaskDetection()
                    new_mask_for_pc = []
                    for c in p:
                            
                        points_mask = Point()
                        points_mask.x = float(c[0])
                        points_mask.y = float(c[1])
                        points_mask.z = 0.0
                        new_mask.point.append(points_mask)

                        points_mask_for_pc = np.array([float(c[0]), float(c[1])], dtype=np.float32)
                        new_mask_for_pc.append(points_mask_for_pc)

                    list_masks.masks.append(new_mask)
                    
                    new_mask_for_pc = np.array(new_mask_for_pc)
                    list_masks_for_pc.append(new_mask_for_pc)

                list_masks_for_pc = np.array(list_masks_for_pc, dtype=object)  # dtype=object if polygons have different number of points
                
                msg.mask = list_masks

                ### PREVIOUSLY THE POINT CLOUD WAS CALCULATED BYA WEIGHTED AVERAGE OF ALL MASKS
                ### HOWEVER, WE CAME TO THE CONCLUSION THAT THIS ADDED SOME ERRORS TO THE DISTANCE READING
                ### THE MOST STABLE VERSION USES THE POINT CLOUD FROM THE MASK WITH BIGGEST AREA

                highest_area_polygon = []
                max_area = 0
                # print("AREAS PRE:", len(area_each_polygon), area_each_polygon)
                # Selects the mask with higher area
                for p, a in zip(list_masks_for_pc, area_each_polygon):
                    # print(a, p)
                    if a > max_area:
                        # Update the maximum area and the corresponding coordinates
                        max_area = a
                        highest_area_polygon = p

                # print("FINAL:")
                # print(max_area)
                # print(max_area, highest_area_polygon)

                obj_3d_cam_coords = Point()
                
                # if we have a correct mask 
                if len(highest_area_polygon) >= 3 and max_area > 0:
                    
                    obj_3d_cam_coords = self.node.point_cloud.convert_mask_to_3dpoint(depth_img=depth_frame, camera="head", mask=highest_area_polygon)
                    print("Max Area:", round(obj_3d_cam_coords.x, 2), round(obj_3d_cam_coords.y, 2), round(obj_3d_cam_coords.z, 2))


                ### HOWEVER IF IT IS NECESSARY TO GO BACK TO WIGHTED AVERAGE FOR ALL FILTERED MASKS:
                total_area = 0
                obj_3d_cam_coords = Point()
                for p, a in zip(list_masks_for_pc, area_each_polygon):
                
                    # if we have a correct mask 
                    if len(p) >= 3 and a > 0:
                        
                        temp_obj_3d_cam_coords = self.node.point_cloud.convert_mask_to_3dpoint(depth_img=depth_frame, camera="head", mask=p)
                        # print(round(obj_3d_cam_coords.x, 2), round(obj_3d_cam_coords.y, 2), round(obj_3d_cam_coords.z, 2))
                        total_area += a
                        obj_3d_cam_coords.x += temp_obj_3d_cam_coords.x * a
                        obj_3d_cam_coords.y += temp_obj_3d_cam_coords.y * a
                        obj_3d_cam_coords.z += temp_obj_3d_cam_coords.z * a
                        

                if total_area > 0:
                    # Compute weighted averages
                    obj_3d_cam_coords.x = obj_3d_cam_coords.x / total_area
                    obj_3d_cam_coords.y = obj_3d_cam_coords.y / total_area
                    obj_3d_cam_coords.z = obj_3d_cam_coords.z / total_area
                
                print("Weighted Avg Area:", round(obj_3d_cam_coords.x, 2), round(obj_3d_cam_coords.y, 2), round(obj_3d_cam_coords.z, 2))

                
                # if there is no correct depth point available, it returns (0, 0, 0) 
                # or no correct mask
                if obj_3d_cam_coords.x == 0 and obj_3d_cam_coords.y == 0 and obj_3d_cam_coords.z == 0: 

                    # creates transforms to base_footprint and map if available
                    map_transform, _ = self.get_transform() # base_footprint -> map
                    transform, camera_link = self.get_transform("head")

                    point_cam = PointStamped()
                    point_cam.header.stamp = self.node.get_clock().now().to_msg()
                    point_cam.header.frame_id = camera_link
                    point_cam.point = obj_3d_cam_coords
                    msg.position_cam = point_cam.point

                    transformed_point = PointStamped()
                    transformed_point_map = PointStamped()
                    if transform is not None:
                        transformed_point = do_transform_point(point_cam, transform)
                        msg.position_relative = transformed_point.point
                        self.node.get_logger().info(f"Object in base_footprint frame: {transformed_point.point}")

                        if map_transform is not None:
                            transformed_point_map = do_transform_point(transformed_point, map_transform)
                            msg.position_absolute = transformed_point_map.point
                            self.node.get_logger().info(f"Object in map frame: {transformed_point_map.point}")



                ### TR TR TR TO DO: WEIGHTED AVERAGE ALSO FOR #D COORDS ARE NOT ONLY FOR CENTROID
                            
                # POINT CLOUD HERE
                #                             
                # self.node.waiting_for_pcloud = True
                # self.node.call_point_cloud_mask_server(requested_objects, "head")
                # 
                # while self.node.waiting_for_pcloud:
                #     pass






                ### CONFIRMAR SE OS DOIS MASS COMMENTS ABAIXO, SAO OS DOIS USADOS...







                ### creio que este ja estava em comentario
                ### CALCULATES FINAL 3D TRACKING COORDINATES USING A WEIGHTED AVERAGE
                """
                weighted_sum_x = 0
                weighted_sum_y = 0
                weighted_sum_z = 0
                max_area = 0
                print("NEW PC:", len(self.node.point_cloud_mask_response.coords))
                for p, a in zip(self.node.point_cloud_mask_response.coords, area_each_polygon):
                    print(f"(x,y,z)): ({p.center_coords.x}, {p.center_coords.y}, {p.center_coords.z}), Area: {a}")

                    # Remove the points whose mask does not have a valid depth point inside (returned by PC as: (x=0.0, y=0.0, z=0.0))
                    if p.center_coords.x != 0 and p.center_coords.y != 0 and p.center_coords.z != 0 and a > MIN_AREA_FOR_PC_CALCULATION:
                        # Use the area of each mask to weight the position of the object
                        weighted_sum_x += p.center_coords.x * a
                        weighted_sum_y += p.center_coords.y * a
                        weighted_sum_z += p.center_coords.z * a
                        max_area += a

                if max_area > 0:
                    # Compute weighted averages
                    x_f = weighted_sum_x / max_area
                    y_f = weighted_sum_y / max_area
                    z_f = weighted_sum_z / max_area
                """
                
                # I think this just uses the bigger area... version after trying to get the average of all polygons
                ### CALCULATES FINAL 3D TRACKING COORDINATES USING A WEIGHTED AVERAGE
                """
                max_area = 0
                x_f, y_f, z_f = 0.0, 0.0, 0.0
                print("NEW PC:", len(self.node.point_cloud_mask_response.coords))
                for p, a in zip(self.node.point_cloud_mask_response.coords, area_each_polygon):
                    print(f"(x,y,z)): ({p.center_coords.x}, {p.center_coords.y}, {p.center_coords.z}), Area: {a}")

                    # Remove the points whose mask does not have a valid depth point inside (returned by PC as: (x=0.0, y=0.0, z=0.0))
                    if p.center_coords.x != 0 and p.center_coords.y != 0 and p.center_coords.z != 0 and a > max_area:
                        # Update the maximum area and the corresponding coordinates
                        max_area = a
                        x_f = p.center_coords.x
                        y_f = p.center_coords.y
                        z_f = p.center_coords.z

                if max_area > 0:

                    # Output result
                    print(f"Weighted Average (x, y, z): ({x_f}, {y_f}, {z_f})")

                    # changes the axis of point cloud coordinates to fit with robot axis
                    object_rel_pos = Point()
                    object_rel_pos.x =  x_f/1000
                    object_rel_pos.y =  y_f/1000
                    object_rel_pos.z =  z_f/1000
                    msg.position_relative = object_rel_pos
                    
                    # calculate the absolute position according to the robot localisation
                    angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                    dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                    theta_aux = math.pi/2 - (angle_obj - self.node.robot_pose.theta)

                    target_x = dist_obj * math.cos(theta_aux) + self.node.robot_pose.x
                    target_y = dist_obj * math.sin(theta_aux) + self.node.robot_pose.y

                    object_abs_pos = Point()
                    object_abs_pos.x = target_x
                    object_abs_pos.y = target_y
                    object_abs_pos.z = z_f/1000
                    msg.position_absolute = object_abs_pos
                """                    
                self.node.tracking_mask_publisher.publish(msg)

            return centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon
        
        return None, None, None, None

    def main(self):

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):

            if self.DEBUG_DRAW:
                cv2.namedWindow("Segmented Objects TR")
    
            while True:

                if self.node.new_head_rgb:

                    tot = time.time()
                    
                    with data_lock:
                        head_image_frame = self.node.head_rgb_cv2_frame.copy()
                        head_depth_frame = self.node.head_depth_cv2_frame.copy()
                    
                    self.node.new_head_rgb = False
                    # frame = self.node.br.imgmsg_to_cv2(self.node.head_rgb, "bgr8")
                    height, width = head_image_frame.shape[:2]
                    main_with_mask = head_image_frame.copy()
    
                    if self.node.tracking_flag:

                        if self.node.calibration_mode:

                            self.predictor.load_first_frame(head_image_frame)

                            if self.node.tracking_received_points:
                                # Use points and object ID as prompts to multiple points:
                                _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                                    frame_idx=0,  # First frame
                                    obj_id=self.initial_obj_id,
                                    points=self.node.tracking_received_points,
                                    labels=self.node.tracking_received_labels
                                )
                                self.node.calibration_mode = False

                            elif self.node.tracking_received_bbox:
                                # Use bounding box and object ID as prompts
                                _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                                    frame_idx=0,  # First frame
                                    obj_id=self.initial_obj_id,
                                    bbox=self.node.tracking_received_bbox
                                )
                                self.node.calibration_mode = False

                            else:
                                self.node.tracking_flag = False
                            
                  
                        else: # Standard work mode (tracking)  
                            
                            aaa = time.time()
                            # Track object in subsequent frames
                            out_obj_ids, out_mask_logits = self.predictor.track(head_image_frame)

                            print("PREDICT TIME:", time.time() - aaa)

                            # Convert logits to binary mask
                            mask = (out_mask_logits[0] > 0).cpu().numpy().astype("uint8") * 255  # Binary mask, 2D
                        
                            # Ensure the mask is 2D before applying colormap
                            if mask.ndim == 3:
                                mask = mask.squeeze()  # Remove extra dimensions if present

                            # Create a white mask where the object is segmented
                            white_mask = (mask > 0).astype("uint8") * 255  # Binary mask, 2D with white pixels

                            if self.DEBUG_DRAW:
                                # Apply the white mask to the frame
                                frame_with_mask = cv2.bitwise_and(head_image_frame, head_image_frame, mask=white_mask)
                                # Display the result
                                cv2.imshow("Frame with Mask", frame_with_mask)

                                ### ORIGINAL MASK BY SAM2 ###
                                # # Apply colormap for visualization
                                # mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
                                # # Blend the mask with the frame
                                # overlay = cv2.addWeighted(frame, 0.7, mask_colored, 0.3, 0)
                                # # Display the result
                                # v2.imshow("Segmented Object", overlay)

                                green_overlay = np.zeros_like(head_image_frame)
                                green_overlay[:, :] = [0, 255, 0]  # BGR for green
                                green_part = cv2.bitwise_and(green_overlay, green_overlay, mask=mask)
                                overlay_green = cv2.addWeighted(head_image_frame, 1.0, green_part, 0.3, 0)
                                
                                # cv2.imshow("Frame with Green Mask", overlay_green)
                                main_with_mask = overlay_green
                                
                            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                            polygons = []
                            for obj in contours:
                                coords = []
                                    
                                # for point in obj:
                                #     coords.append(int(point[0][0]))
                                #     coords.append(int(point[0][1]))
                                for point in obj:
                                    coords.append([int(point[0][0]), int(point[0][1])])  # Store as [x, y]
                                polygons.append(coords)
                                # polygons.append(coords)

                            centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon = self.filter_and_publish_tracking_data(polygons, white_mask, head_depth_frame)

                            if self.DEBUG_DRAW:
                            
                                teste = head_image_frame.copy()

                                if updated_filtered_polygons:
                                    for p in updated_filtered_polygons:
                                        cv2.polylines(teste, [np.array(p, dtype=np.int32)], True, (0, 255, 255), 5)
                                        cv2.fillPoly(teste, [np.array(p, dtype=np.int32)], (0, 100, 100))

                                    # print(len(updated_filtered_polygons))
                                    # print("out")
                                    # print(centroid)

                                    if centroid is not None:
                                        # print("in centroid")
                                        cv2.circle(teste, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                                        cv2.circle(main_with_mask, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                                        # cv2.circle(white_mask, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)

                                        for p, a in zip(centroid_each_polygon, area_each_polygon):
                                            cv2.circle(teste, (int(p[0]), int(p[1])), 5, (255, 0, 0), -1)
                                            cv2.circle(main_with_mask, (int(p[0]), int(p[1])), 5, (255, 0, 0), -1)
                                            cv2.putText(teste, 'A:' + str(int(a)), (int(p[0]), int(p[1])), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                                            cv2.putText(main_with_mask, 'A:' + str(int(a)), (int(p[0]), int(p[1])), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        
                                        
                                cv2.imshow("Frame with Mask BW", white_mask)
                                cv2.imshow("Test Polygon", teste)

                    # Display the result
                    # cv2.imshow("Segmented Object", overlay)
                    self.new_frame_time = time.time()
                    self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
                    self.prev_frame_time = self.new_frame_time
                    self.fps = str(self.fps)
                    print("fps:", self.fps)
                                
                    if self.DEBUG_DRAW:

                        if self.node.tracking_received_points:
                            for i, point in enumerate(self.node.tracking_received_points):
                                color = (0, 255, 0)  if self.node.tracking_received_labels[i] == 1 else (0, 0, 255)
                                # print(point, color)
                                cv2.circle(main_with_mask, point, 5, color, -1)
                        if self.node.tracking_received_bbox:
                            cv2.rectangle(main_with_mask, (self.node.tracking_received_bbox[0], self.node.tracking_received_bbox[1]), (self.node.tracking_received_bbox[2], self.node.tracking_received_bbox[3]), (255, 0, 0), 2)

                
                        # print(self.fps)
                        cv2.putText(main_with_mask, 'fps:' + self.fps, (0, height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.imshow("Segmented Objects TR", main_with_mask)
                        # cv2.imshow("Frame with Mask", cv2.bitwise_and(frame, frame, ))
                        cv2.waitKey(1)


                    print("TOTAL TIME:", time.time() - tot)

        if self.DEBUG_DRAW:
            cv2.destroyAllWindows()

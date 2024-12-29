#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from sensor_msgs.msg import Image
from charmie_interfaces.msg import TrackingMask, ListOfPoints, BoundingBox, MaskDetection, ListOfMaskDetections
from charmie_interfaces.srv import ActivateTracking, GetPointCloudMask

import threading
import cv2
import torch
from sam2.build_sam import build_sam2_camera_predictor
from pathlib import Path
from cv_bridge import CvBridge
import numpy as np
import time
import math


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
        self.new_head_rgb = False
        self.waiting_for_pcloud = False

        # robot localization
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0

        ### Topics ###
        # Intel Realsense
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        # Publica a máscara do objecto a ser seguido
        self.tracking_mask_publisher = self.create_publisher(TrackingMask, 'tracking_mask', 10)
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)

        # SERVICES:
         # Point Cloud Mask Service
        self.point_cloud_mask_client = self.create_client(GetPointCloudMask, "get_point_cloud_mask")

        while not self.point_cloud_mask_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Point Cloud Mask...")

        # Acitvate and Deactivate Tracking Service
        self.activate_yolo_objects_service = self.create_service(ActivateTracking, "activate_tracking", self.callback_activate_tracking)
       

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
    
    # request point cloud information from point cloud node
    def call_point_cloud_mask_server(self, req, camera):
        request = GetPointCloudMask.Request()
        request.data = req
        request.camera = camera
    
        future = self.point_cloud_mask_client.call_async(request)
        future.add_done_callback(self.callback_call_point_cloud_mask)

    def callback_call_point_cloud_mask(self, future):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            self.point_cloud_mask_response = future.result()
            self.waiting_for_pcloud = False
            # print("Received Back")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def get_color_image_head_callback(self, img: Image):
        self.head_rgb = img
        self.new_head_rgb = True
 
    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_x = pose.x
        self.robot_y = pose.y
        self.robot_t = pose.theta


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
        
        self.DEBUG_DRAW = True

        self.prev_frame_time = time.time() # used to record the time when we processed last frame
        self.new_frame_time = time.time() # used to record the time at which we processed current frame
        
    def polygon_centroid(self, polygon_points):
        # Ensure the points are in NumPy array format
        points = np.array(polygon_points, dtype=np.float32)

        # Close the polygon if it's not already closed
        if not np.array_equal(points[0], points[-1]):
            points = np.vstack([points, points[0]])

        # Calculate the signed area (A)
        x = points[:, 0]
        y = points[:, 1]
        A = 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])

        # Skip if the area is 0
        if A == 0:
            return None

        # Calculate the centroid (Cx, Cy)
        Cx = np.sum((x[:-1] + x[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)
        Cy = np.sum((y[:-1] + y[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)

        return Cx, Cy

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
    
    def filter_and_publish_tracking_data(self, polygons, binary_mask):

        MIN_AREA_FOR_PC_CALCULATION = 4000

        if polygons:
            centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon = self.combined_polygon_centroid(polygons, binary_mask)
            if centroid is not None:
                
                msg = TrackingMask()
                msg.centroid.x = float(centroid[0])
                msg.centroid.y = float(centroid[1])
                
                list_masks = ListOfMaskDetections()
                requested_objects = []
                
                for p in updated_filtered_polygons: # only goes through filtres polygons, rather than all polygons

                    new_mask = MaskDetection()
                    for c in p:
                            
                        points_mask = Point()
                        points_mask.x = float(c[0])
                        points_mask.y = float(c[1])
                        points_mask.z = 0.0
                        new_mask.point.append(points_mask)

                    list_masks.masks.append(new_mask)
                    requested_objects.append(new_mask)
                
                # msg.binary_mask = self.node.br.cv2_to_imgmsg(white_mask, encoding='mono8')
                msg.mask = list_masks
                                            
                self.node.waiting_for_pcloud = True
                self.node.call_point_cloud_mask_server(requested_objects, "head")

                while self.node.waiting_for_pcloud:
                    pass

                weighted_sum_x = 0
                weighted_sum_y = 0
                weighted_sum_z = 0
                total_weight = 0
                print("NEW PC:", len(self.node.point_cloud_mask_response.coords))
                for p, a in zip(self.node.point_cloud_mask_response.coords, area_each_polygon):
                    print(f"(x,y,z)): ({p.center_coords.x}, {p.center_coords.y}, {p.center_coords.z}), Area: {a}")

                    # Remove the points whose mask does not have a valid depth point inside (returned by PC as: (x=0.0, y=0.0, z=0.0))
                    if p.center_coords.x != 0 and p.center_coords.y != 0 and p.center_coords.z != 0 and a > MIN_AREA_FOR_PC_CALCULATION:
                        # Use the area of each mask to weight the position of the object
                        weighted_sum_x += p.center_coords.x * a
                        weighted_sum_y += p.center_coords.y * a
                        weighted_sum_z += p.center_coords.z * a
                        total_weight += a
    
                if total_weight > 0:
                    # Compute weighted averages
                    x_avg = weighted_sum_x / total_weight
                    y_avg = weighted_sum_y / total_weight
                    z_avg = weighted_sum_z / total_weight

                    # Output result
                    print(f"Weighted Average (x, y, z): ({x_avg}, {y_avg}, {z_avg})")

                    # changes the axis of point cloud coordinates to fit with robot axis
                    object_rel_pos = Point()
                    object_rel_pos.x =  -y_avg/1000
                    object_rel_pos.y =  x_avg/1000
                    object_rel_pos.z =  z_avg/1000
                    msg.position_relative = object_rel_pos
                    
                    # calculate the absolute position according to the robot localisation
                    angle_obj = math.atan2(object_rel_pos.x, object_rel_pos.y)
                    dist_obj = math.sqrt(object_rel_pos.x**2 + object_rel_pos.y**2)

                    theta_aux = math.pi/2 - (angle_obj - self.node.robot_t)

                    target_x = dist_obj * math.cos(theta_aux) + self.node.robot_x
                    target_y = dist_obj * math.sin(theta_aux) + self.node.robot_y

                    object_abs_pos = Point()
                    object_abs_pos.x = target_x
                    object_abs_pos.y = target_y
                    object_abs_pos.z = z_avg/1000
                    msg.position_absolute = object_abs_pos
                    
                    self.node.tracking_mask_publisher.publish(msg)

            return centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon
        
        return None, None, None, None

    def main(self):

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):

            if self.DEBUG_DRAW:
                cv2.namedWindow("Segmented Objects TR")
    
            while True:

                if self.node.new_head_rgb:
                    self.node.new_head_rgb = False
                    frame = self.node.br.imgmsg_to_cv2(self.node.head_rgb, "bgr8")
                    height, width = frame.shape[:2]
                    main_with_mask = frame.copy()
    
                    if self.node.tracking_flag:

                        if self.node.calibration_mode:

                            self.predictor.load_first_frame(frame)

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
                            
                            # Track object in subsequent frames
                            out_obj_ids, out_mask_logits = self.predictor.track(frame)

                            # Convert logits to binary mask
                            mask = (out_mask_logits[0] > 0).cpu().numpy().astype("uint8") * 255  # Binary mask, 2D
                        
                            # Ensure the mask is 2D before applying colormap
                            if mask.ndim == 3:
                                mask = mask.squeeze()  # Remove extra dimensions if present

                            # Create a white mask where the object is segmented
                            white_mask = (mask > 0).astype("uint8") * 255  # Binary mask, 2D with white pixels

                            if self.DEBUG_DRAW:
                                # Apply the white mask to the frame
                                frame_with_mask = cv2.bitwise_and(frame, frame, mask=white_mask)
                                # Display the result
                                cv2.imshow("Frame with Mask", frame_with_mask)

                                ### ORIGINAL MASK BY SAM2 ###
                                # # Apply colormap for visualization
                                # mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
                                # # Blend the mask with the frame
                                # overlay = cv2.addWeighted(frame, 0.7, mask_colored, 0.3, 0)
                                # # Display the result
                                # v2.imshow("Segmented Object", overlay)

                                green_overlay = np.zeros_like(frame)
                                green_overlay[:, :] = [0, 255, 0]  # BGR for green
                                green_part = cv2.bitwise_and(green_overlay, green_overlay, mask=mask)
                                overlay_green = cv2.addWeighted(frame, 1.0, green_part, 0.3, 0)
                                
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

                            centroid, updated_filtered_polygons, area_each_polygon, centroid_each_polygon = self.filter_and_publish_tracking_data(polygons, white_mask)

                            if self.DEBUG_DRAW:
                            
                                teste = frame.copy()

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

        if self.DEBUG_DRAW:
            cv2.destroyAllWindows()

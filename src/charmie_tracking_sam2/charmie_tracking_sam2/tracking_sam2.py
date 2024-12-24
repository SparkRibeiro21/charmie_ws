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
                                
        print("===", self.tracking_received_points, self.tracking_received_labels)

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
        
        self.DEBUG_DRAW = False

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

    def combined_polygon_centroid(self, polygons):
        total_area = 0
        centroid_x = 0
        centroid_y = 0

        for polygon_points in polygons:
            points = np.array(polygon_points, dtype=np.float32)
            
            # Skip invalid polygons
            if len(points) < 3:
                continue
            if not np.array_equal(points[0], points[-1]):
                points = np.vstack([points, points[0]])
            
            x = points[:, 0]
            y = points[:, 1]
            A = 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])
            
            # Skip polygons with zero area
            if A == 0:
                continue
            
            Cx = np.sum((x[:-1] + x[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)
            Cy = np.sum((y[:-1] + y[1:]) * (x[:-1] * y[1:] - x[1:] * y[:-1])) / (6 * A)
            
            total_area += A
            centroid_x += Cx * A
            centroid_y += Cy * A

        if total_area == 0:
            return None  # No valid polygons
        
        # Final combined centroid
        combined_Cx = centroid_x / total_area
        combined_Cy = centroid_y / total_area
        return combined_Cx, combined_Cy
    
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
                            if polygons:
                                centroid = self.combined_polygon_centroid(polygons)
                                if centroid is not None:
                                    
                                    msg = TrackingMask()
                                    msg.centroid.x = float(centroid[0])
                                    msg.centroid.y = float(centroid[1])
                                    
                                    list_masks = ListOfMaskDetections()
                                    
                                    for p in polygons:

                                        new_mask = MaskDetection()
                                        
                                        for c in p:
                                                
                                            points_mask = Point()
                                            points_mask.x = float(c[0])
                                            points_mask.y = float(c[1])
                                            points_mask.z = 0.0
                                            new_mask.point.append(points_mask)

                                        list_masks.masks.append(new_mask)
                                    
                                    # msg.binary_mask = self.node.br.cv2_to_imgmsg(white_mask, encoding='mono8')
                                    msg.mask = list_masks
                                                                
                                    object_abs_pos = Point()
                                    object_abs_pos.x = 3.0
                                    object_abs_pos.y = 2.0
                                    object_abs_pos.z = 1.0
                                    msg.position_absolute = object_abs_pos
                                    
                                    # when i add the method to get the 3D coordinates, i have to calculate the final 3D coordiantes
                                    # as a weighted average using the area of each mask as the weight

                                    # resolver a cena de o tracking ficar sempre a mostrar no mapa
                                    """
                                    requested_objects = []
                                    for mask in masks:
                                        m = MaskDetection()
                                        for p in mask.xy[0]:

                                            points_mask = Point()
                                            points_mask.x = float(p[0])
                                            points_mask.y = float(p[1])
                                            points_mask.z = 0.0
                                            m.point.append(points_mask)

                                        requested_objects.append(m)

                                    # print(requested_objects)

                                    self.node.waiting_for_pcloud = True
                                    self.node.call_point_cloud_mask_server(requested_objects, "head")

                                    while self.node.waiting_for_pcloud:
                                        pass

                                    new_pcloud = self.node.point_cloud_mask_response.coords
                                    """
                                
                                    self.node.tracking_mask_publisher.publish(msg)

                            if self.DEBUG_DRAW:
                            
                                teste = frame.copy()

                                if polygons:
                                    for p in polygons:
                                        cv2.polylines(teste, [np.array(p, dtype=np.int32)], True, (0, 255, 0), 2)
                                        cv2.fillPoly(teste, [np.array(p, dtype=np.int32)], (0, 100, 0))
                                                                
                                    # print(polygons[0])
                                    # centroid = self.polygon_centroid(polygons[0])
                                    # centroid = self.combined_polygon_centroid(polygons)
                                    # print(centroid)

                                    if centroid is not None:
                                        cv2.circle(teste, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                                        # cv2.circle(white_mask, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                                cv2.imshow("Frame with Mask BW", white_mask)
                                cv2.imshow("Test Polygon", teste)

                    # Display the result
                    # cv2.imshow("Segmented Object", overlay)
                    self.new_frame_time = time.time()
                    self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
                    self.prev_frame_time = self.new_frame_time
                    self.fps = str(self.fps)
                    print(self.fps)
                                
                    if self.DEBUG_DRAW:

                        if self.node.tracking_received_points:
                            for i, point in enumerate(self.node.tracking_received_points):
                                color = (0, 255, 0)  if self.node.tracking_received_labels[i] == 1 else (0, 0, 255)
                                print(point, color)
                                cv2.circle(main_with_mask, point, 5, color, -1)
                        if self.node.tracking_received_bbox:
                            cv2.rectangle(main_with_mask, (self.node.tracking_received_bbox[0], self.node.tracking_received_bbox[1]), (self.node.tracking_received_bbox[2], self.node.tracking_received_bbox[3]), (255, 0, 0), 2)

                
                        print(self.fps)
                        cv2.putText(main_with_mask, 'fps:' + self.fps, (0, height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                        cv2.imshow("Segmented Objects TR", main_with_mask)
                        # cv2.imshow("Frame with Mask", cv2.bitwise_and(frame, frame, ))
                        cv2.waitKey(1)

        if self.DEBUG_DRAW:
            cv2.destroyAllWindows()

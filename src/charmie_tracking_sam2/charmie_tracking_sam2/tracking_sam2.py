#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from charmie_interfaces.msg import TrackingMask, ListOfPoints, BoundingBox
from charmie_interfaces.srv import ActivateTracking

import threading
import cv2
import torch
from sam2.build_sam import build_sam2_camera_predictor
from pathlib import Path
import numpy as np
import time


class TrackingNode(Node):

    def __init__(self):
        super().__init__('Tracking')
        self.get_logger().info("Initialised CHARMIE Tracking Node")

        # Tracking Variables Initialisation
        self.tracking_flag = False
        self.tracking_received_points = ListOfPoints()
        self.tracking_received_bbox = BoundingBox()

        # TOPICS:
        # Publica a máscara do objecto a ser seguido
        self.tracking_mask_publisher = self.create_publisher(TrackingMask, 'tracking_mask', 10)

        # SERVICES:
        # Ativa e desativa o tracking
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
        
        self.tracking_flag = request.activate
        self.tracking_received_points = request.points
        self.tracking_received_bbox = request.bounding_box


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

        # Load video or camera
        self.cap = cv2.VideoCapture(0)  # Replace 0 with video file path if needed

        self.initial_obj_id = 1  # Object ID for tracking

        # self.tracking_flag = True  # Flag to enable tracking
        self.calibration_mode = False # Calibrations starts as true so it is executed right at the beggining
        self.first_calibration_done = False

        self.selected_points = []
        self.point_labels = []
        self.done_selecting = False

        self.bounding_box_events = False
        self.points_events = False

        self.drawing = False
        self.ix, self.iy = -1, -1  # Initial coordinates
        self.x, self.y = -1, -1  # Mouse coordinates
        self.selected_bbox = []                  
        
        self.prev_frame_time = time.time() # used to record the time when we processed last frame
        self.new_frame_time = time.time() # used to record the time at which we processed current frame
        

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for selecting points and labels."""
        
        if self.points_events:
            if event == cv2.EVENT_MBUTTONDOWN:
                # Append selected point and label
                self.selected_points.append((x, y))
                print(f"Selected point: {(x, y)}")
                label = "1"                

                self.point_labels.append(int(label))
                # Print the current state of points and labels
                print(f"Selected points ({len(self.selected_points)}): {self.selected_points}")
                print(f"Point Labels ({len(self.point_labels)}): {self.point_labels}")

            if event == cv2.EVENT_RBUTTONDOWN: #  or event == cv2.EVENT_MBUTTONDOWN:
                # Append selected point and label
                self.selected_points.append((x, y))
                print(f"Selected point: {(x, y)}")
                label = "0"
                
                self.point_labels.append(int(label))
                # Print the current state of points and labels
                print(f"Selected points ({len(self.selected_points)}): {self.selected_points}")
                print(f"Point Labels ({len(self.point_labels)}): {self.point_labels}")

        if self.bounding_box_events:
            
            if event == cv2.EVENT_LBUTTONDOWN:  # Mouse button pressed
                self.drawing = True
                self.ix, self.iy = x, y
                self.x = x
                self.y = y

            elif event == cv2.EVENT_MOUSEMOVE:  # Mouse movement
                if self.drawing:
                    self.x = x
                    self.y = y

            elif event == cv2.EVENT_LBUTTONUP:  # Mouse button released
                self.drawing = False
                self.selected_bbox = [(self.ix, self.iy), (self.x, self.y)]  # Store rectangle coordinates

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

            cv2.namedWindow("Segmented Objects TR")
            cv2.setMouseCallback("Segmented Objects TR", self.mouse_callback)

            while True:

                ret, frame = self.cap.read()
                height, width = frame.shape[:2]  # Get frame dimensions
                main_with_mask = frame.copy()
                
                if not ret:
                    break

                if self.node.tracking_flag:

                    if self.calibration_mode:
                        
                        if self.points_events:
                            for i, point in enumerate(self.selected_points):
                                color = (0, 255, 0)  if self.point_labels[i] == 1 else (0, 0, 255)
                                cv2.circle(main_with_mask, point, 5, color, -1)

                        if self.bounding_box_events:
                            if self.selected_bbox:
                                cv2.rectangle(main_with_mask, self.selected_bbox[0], self.selected_bbox[1], (0, 255, 0), 2)
                            if self.drawing:
                                cv2.rectangle(main_with_mask, (self.ix, self.iy), (self.x, self.y), (255, 0, 0), 2)

                        if self.done_selecting:
                            
                            self.predictor.load_first_frame(frame)

                            multiple_points = []
                            bbox = []

                            # if self.points_events and not self.bounding_box_events:
                            if self.selected_points:
                                
                                print("just points")
                                multiple_points = [(x , y) for x, y in self.selected_points]
                                # Use points and object ID as prompts to multiple points:
                                _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                                    frame_idx=0,  # First frame
                                    obj_id=self.initial_obj_id,
                                    points=multiple_points,
                                    labels=self.point_labels
                                )
                                
                            # if self.bounding_box_events and not self.points_events:
                            elif self.selected_bbox:

                                print("just bbox")
                                # multiple_bboxes = [[r1[0], r1[1], r2[0], r2[1]] for r1, r2 in self.selected_bboxes]
                                bbox = [self.selected_bbox[0][0], self.selected_bbox[0][1], self.selected_bbox[1][0], self.selected_bbox[1][1]]
                                # Use bounding box and object ID as prompts
                                _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                                    frame_idx=0,  # First frame
                                    obj_id=self.initial_obj_id,
                                    bbox=bbox
                                )

                            self.selected_bbox.clear()                              
                            self.selected_points.clear()
                            self.point_labels.clear()

                            self.calibration_mode = False
                            self.points_events = False
                            self.bounding_box_events = False
                            self.done_selecting = False
                            self.first_calibration_done = True
                            print("Calibration done")


                    else: # Standard work mode (tracking)  
                        
                        if self.first_calibration_done:
                            
                            # Track object in subsequent frames
                            out_obj_ids, out_mask_logits = self.predictor.track(frame)

                            # Convert logits to binary mask
                            mask = (out_mask_logits[0] > 0).cpu().numpy().astype("uint8") * 255  # Binary mask, 2D
                        
                            # Ensure the mask is 2D before applying colormap
                            if mask.ndim == 3:
                                mask = mask.squeeze()  # Remove extra dimensions if present

                            # Create a white mask where the object is segmented
                            white_mask = (mask > 0).astype("uint8") * 255  # Binary mask, 2D with white pixels
                        
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

                            teste = frame.copy()

                            if polygons:
                                for p in polygons:
                                    # cv2.polylines(teste, [np.array(p).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
                                    # cv2.fillPoly(teste, [np.array(p).reshape((-1, 1, 2))], (0, 100, 0))
                                    cv2.polylines(teste, [np.array(p, dtype=np.int32)], True, (0, 255, 0), 2)
                                    cv2.fillPoly(teste, [np.array(p, dtype=np.int32)], (0, 100, 0))
                                                            
                                # print(polygons[0])
                                # centroid = self.polygon_centroid(polygons[0])
                                centroid = self.combined_polygon_centroid(polygons)

                                print(centroid)

                                if centroid is not None:
                                    cv2.circle(teste, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                                    cv2.circle(white_mask, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
                            
                            cv2.imshow("Frame with Mask BW", white_mask)
                            cv2.imshow("Test Polygon", teste)
                            
                # Display the result
                # cv2.imshow("Segmented Object", overlay)
                self.new_frame_time = time.time()
                self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
                self.prev_frame_time = self.new_frame_time
                self.fps = str(self.fps)

                #print(self.fps)
                cv2.putText(main_with_mask, 'fps:' + self.fps, (0, height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("Segmented Objects TR", main_with_mask)
                # cv2.imshow("Frame with Mask", cv2.bitwise_and(frame, frame, ))
        
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    self.node.tracking_flag = not self.node.tracking_flag

                elif key == ord('s'): # Finalize the selection
                    self.done_selecting = True

                elif key == ord('c'):
                    if self.first_calibration_done:
                        self.predictor.reset_state()
                    self.calibration_mode = True
                    self.bounding_box_events = True
                    self.points_events = True

        self.cap.release()
        cv2.destroyAllWindows()

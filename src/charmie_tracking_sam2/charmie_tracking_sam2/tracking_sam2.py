#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point

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


        # TOPICS:
        # Publica a máscara do objecto a ser seguido


        # SERVICES:
        # Ativa e desativa o tracking


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

        # Define model and configurations (ORIGINAL)
        # sam2_checkpoint = "../checkpoints/sam2.1_hiera_small.pt"
        # model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"
        # predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)

        print("HEY")

        # Load video or camera
        self.cap = cv2.VideoCapture(0)  # Replace 0 with video file path if needed

        self.if_init = False
        self.initial_obj_id = 1  # Object ID for tracking
        self.tracking_flag = True  # Flag to enable tracking

        """
        # Define points for initialization (example: 3 points)
        self.initial_points = [
            (320, 240),  # Center point
            (300, 220),  # Slightly left and up
            (340, 260)   # Slightly right and down
            # (32, 180),  # Slightly left and up
            # (528, 197)   # Slightly right and down
        ]
        self.point_labels = [1, 1, 1]  # Positive labels for all points
        """

        self.selected_points = []
        self.point_labels = []
        self.done_selecting = False

        self.bounding_box_events = False
        self.points_events = True

        self.drawing = False
        self.ix, self.iy = -1, -1  # Initial coordinates
        self.x, self.y = -1, -1  # Mouse coordinates
        self.rect = None      # To store the final rectangle coordinates
        self.finished_drawing = False
        
        self.prev_frame_time = time.time() # used to record the time when we processed last frame
        self.new_frame_time = time.time() # used to record the time at which we processed current frame
        

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for selecting points and labels."""
        
        if self.points_events:
            if event == cv2.EVENT_LBUTTONDOWN:
                # Append selected point and label
                self.selected_points.append((x, y))
                print(f"Selected point: {(x, y)}")
                label = "1"

                # while label not in ("0", "1"):
                #     label = input("Enter label (0: does not belong, 1: belongs): ")
                

                self.point_labels.append(int(label))
                # Print the current state of points and labels
                print(f"Selected points ({len(self.selected_points)}): {self.selected_points}")
                print(f"Point Labels ({len(self.point_labels)}): {self.point_labels}")

            if event == cv2.EVENT_RBUTTONDOWN or event == cv2.EVENT_MBUTTONDOWN:
                # Append selected point and label
                self.selected_points.append((x, y))
                print(f"Selected point: {(x, y)}")
                label = "0"

                # while label not in ("0", "1"):
                #     label = input("Enter label (0: does not belong, 1: belongs): ")
                
                self.point_labels.append(int(label))
                # Print the current state of points and labels
                print(f"Selected points ({len(self.selected_points)}): {self.selected_points}")
                print(f"Point Labels ({len(self.point_labels)}): {self.point_labels}")

        if self.bounding_box_events:
                    
            if event == cv2.EVENT_LBUTTONDOWN:  # Mouse button pressed
                self.drawing = True
                self.ix, self.iy = x, y

            elif event == cv2.EVENT_MOUSEMOVE:  # Mouse movement
                if self.drawing:
                    self.x = x
                    self.y = y
                #     img_copy = frame.copy()  # Make a copy of the image to avoid redrawing
                #     cv2.rectangle(img_copy, (self.ix, self.iy), (x, y), (0, 255, 0), 2)
                #     cv2.imshow('Select Points', img_copy)

            elif event == cv2.EVENT_LBUTTONUP:  # Mouse button released
                self.drawing = False
                self.finished_drawing = True
                self.x = x
                self.y = y

                self.rect = ((self.ix, self.iy), (self.x, self.y))  # Store rectangle coordinates
                # cv2.rectangle(frame, (self.ix, self.iy), (self.x, self.y), (0, 255, 0), 2)  # Draw final rectangle
                # cv2.imshow('Select Points', frame)



    def process_user_input(self, frame):
        """Dsiplays the first frame and allows point selection."""
        cv2.namedWindow("Select Points")
        cv2.setMouseCallback("Select Points", self.mouse_callback)

        print("Select points on the frame, Left click to add points.")
        print("Press 's' to finalize selection and start tracking.")

        while not self.done_selecting:

            temp_frame = frame.copy()

            if self.points_events:
                for i, point in enumerate(self.selected_points):
                    color = (0, 255, 0)  if self.point_labels[i] == 1 else (0, 0, 255)
                    cv2.circle(temp_frame, point, 5, color, -1)


            if self.bounding_box_events:
                if self.drawing:
                    pass
                    # cv2.rectangle(frame, (self.ix, self.iy), (self.x, self.y), (0, 255, 0), 2)
                
                elif not self.drawing and self.finished_drawing:
                    cv2.rectangle(frame, (self.ix, self.iy), (self.x, self.y), (0, 255, 0), 2)  # Draw final rectangle
                    self.finished_drawing = False


            cv2.imshow("Select Points", temp_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'): # Finalize the selection
                print("Finalizing print selection")
                self.done_selecting = True
                cv2.destroyWindow("Select Points")


    def draw_bb(self):
        pass


    def main(self):

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):

            while True:

                ret, frame = self.cap.read()
                height, width = frame.shape[:2]  # Get frame dimensions
                
                if not ret:
                    break

                if self.tracking_flag:


                    # print(height, width)

                    if not self.if_init:

                        self.process_user_input(frame)
                        
                        # Normalize points relative to the frame dimensions
                        # multiple_points = [(x , y) for x, y in self.initial_points]
                        multiple_points = [(x , y) for x, y in self.selected_points]

                        # # Define central region for initialization
                        # central_bbox = [int(width * 0.3), int(height * 0.2), int(width * 0.7), int(height * 0.8)]  # x1, y1, x2, y2
                        
                        # Define a point for initialization (center of the frame)
                        # point_x, point_y = width // 2, height // 2  # Center of the frame
                        # points = [(point_x, point_y)]  # Single point
                        # labels = [1]  # Label for foreground

                        self.predictor.load_first_frame(frame)
                        self.if_init = True
                        
                        # # Use the point and object ID as prompts
                        # _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                        #     frame_idx=0,  # First frame
                        #     obj_id=self.initial_obj_id,
                        #     points=points,  # Use points instead of bbox
                        #     labels=labels
                        # )
                        
                        # Use points and object ID as prompts to multiple points:
                        _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                            frame_idx=0,  # First frame
                            obj_id=self.initial_obj_id,
                            points=multiple_points,
                            labels=self.point_labels
                        )
                        
                        # Use bounding box and object ID as prompts
                        # _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                        #     frame_idx=0,  # First frame
                        #     obj_id=self.initial_obj_id,
                        #     bbox=central_bbox
                        # )

                        self.done_selecting = False
                        self.selected_points.clear()
                        self.point_labels.clear()

                
                    else:
                        # Track object in subsequent frames
                        out_obj_ids, out_mask_logits = self.predictor.track(frame)

                    # Convert logits to binary mask
                    mask = (out_mask_logits[0] > 0).cpu().numpy().astype("uint8") * 255  # Binary mask, 2D

                    
                    # Ensure the mask is 2D before applying colormap
                    if mask.ndim == 3:
                        mask = mask.squeeze()  # Remove extra dimensions if present

                    # Create a white mask where the object is segmented
                    white_mask = (mask > 0).astype("uint8") * 255  # Binary mask, 2D with white pixels

                    # Ensure the mask is 2D before applying colormap
                    # if white_mask.ndim == 3:
                    #     white_mask = white_mask.squeeze()  # Remove extra dimensions if present



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
                    # cv2.imshow("Segmented Object", overlay)


                    green_overlay = np.zeros_like(frame)
                    green_overlay[:, :] = [0, 255, 0]  # BGR for green
                    green_part = cv2.bitwise_and(green_overlay, green_overlay, mask=mask)
                    overlay_green = cv2.addWeighted(frame, 1.0, green_part, 0.3, 0)
                    
                    self.new_frame_time = time.time()
                    self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
                    self.prev_frame_time = self.new_frame_time
                    self.fps = str(self.fps)
                    print(self.fps)
                    cv2.putText(overlay_green, 'fps:' + self.fps, (0, height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    
                    cv2.imshow("Frame with Green Mask", overlay_green)
                    
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                    polygons = []

                    for obj in contours:
                        coords = []
                            
                        for point in obj:
                            coords.append(int(point[0][0]))
                            coords.append(int(point[0][1]))

                        polygons.append(coords)

                    teste = frame.copy()
                    for p in polygons:
                        cv2.polylines(teste, [np.array(p).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
                        cv2.fillPoly(teste, [np.array(p).reshape((-1, 1, 2))], (0, 100, 0))
                    cv2.imshow("Test Polygon", teste)
                    

                    
                    
                    """
                    # Apply colormap for visualization
                    mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)
                    # Blend the mask with the frame
                    overlay = cv2.addWeighted(frame, 0.7, mask_colored, 0.3, 0)
                    # Display the result
                    cv2.imshow("Segmented Object", frame_with_white_mask)

                    # Create a white image of the same size as the frame
                    white_image = np.full(frame.shape, 255, dtype=np.uint8)
                    cv2.imshow("Segmented 3 Object", frame)


                    # Combine the masked frame and white part
                    result = cv2.add(mask, white_image)

                    # Use the mask to combine the frame and the white image
                    result = cv2.bitwise_or(frame, white_image, mask=mask)
                
                    cv2.imshow("Segmented 2 Object", result)
                    """

                    # self.draw_bb()


                else:


                    # Display the result
                    # cv2.imshow("Segmented Object", overlay)
                    self.new_frame_time = time.time()
                    self.fps = round(1/(self.new_frame_time-self.prev_frame_time), 2)
                    self.prev_frame_time = self.new_frame_time
                    self.fps = str(self.fps)

                    print(self.fps)
                    cv2.putText(frame, 'fps:' + self.fps, (0, height-10), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.imshow("Frame with Green Mask", frame)
                    # cv2.imshow("Frame with Mask", cv2.bitwise_and(frame, frame, ))



                k = cv2.waitKey(1)
                if k == ord('q'):
                    break
                if k == ord('r'):
                    self.predictor.reset_state()
                    self.if_init = False
                if k == ord(' '):
                    self.tracking_flag = not self.tracking_flag

                    # if k == ord('q'):
                    #     self.floor_dist -= 10
                    # if k == ord('s'):
                    #     self.top_bag_dist += 10
                    # if k == ord('a'):
                    #     self.top_bag_dist -= 10

                    # if cv2.waitKey(1) & 0xFF == ord('q'):
                    #     break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
        # main loop
        while True:
            # do something
            pass
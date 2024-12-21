#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point

import threading
import cv2
import torch
from sam2.build_sam import build_sam2_camera_predictor
from pathlib import Path



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


    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for selecting points and labels."""
        
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

        if event == cv2.EVENT_RBUTTONDOWN:
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


    def process_user_input(self, frame):
        """Dsiplays the first frame and allows point selection."""
        cv2.namedWindow("Select Points")
        cv2.setMouseCallback("Select Points", self.mouse_callback)

        print("Select points on the frame, Left click to add points.")
        print("Press 's' to finalize selection and start tracking.")

        while not self.done_selecting:

            temp_frame = frame.copy()
            
            for i, point in enumerate(self.selected_points):
                color = (0, 255, 0)  if self.point_labels[i] == 1 else (0, 0, 255)
                cv2.circle(temp_frame, point, 5, color, -1)

            cv2.imshow("Select Points", temp_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'): # Finalize the selection
                print("Finalizing print selection")
                self.done_selecting = True
                cv2.destroyWindow("Select Points")

    def main(self):

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):

            while True:

                if self.tracking_flag:

                    ret, frame = self.cap.read()
                    if not ret:
                        break

                    height, width = frame.shape[:2]  # Get frame dimensions

                    # print(height, width)

                    if not self.if_init:

                        self.process_user_input(frame)

                        # Normalize points relative to the frame dimensions
                        # multiple_points = [(x , y) for x, y in self.initial_points]
                        multiple_points = [(x , y) for x, y in self.selected_points]

                        # # Define central region for initialization
                        central_bbox = [int(width * 0.3), int(height * 0.2), int(width * 0.7), int(height * 0.8)]  # x1, y1, x2, y2
                        
                        # Define a point for initialization (center of the frame)
                        point_x, point_y = width // 2, height // 2  # Center of the frame
                        points = [(point_x, point_y)]  # Single point
                        labels = [1]  # Label for foreground

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
                
                    else:
                        # Track object in subsequent frames
                        out_obj_ids, out_mask_logits = self.predictor.track(frame)

                    # Convert logits to binary mask
                    mask = (out_mask_logits[0] > 0).cpu().numpy().astype("uint8") * 255  # Binary mask, 2D

                    # Ensure the mask is 2D before applying colormap
                    if mask.ndim == 3:
                        mask = mask.squeeze()  # Remove extra dimensions if present

                    # Apply colormap for visualization
                    mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)

                    # Blend the mask with the frame
                    overlay = cv2.addWeighted(frame, 0.7, mask_colored, 0.3, 0)

                    # Display the result
                    cv2.imshow("Segmented Object", overlay)


                k = cv2.waitKey(1)
                if k == ord('q'):
                    break
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
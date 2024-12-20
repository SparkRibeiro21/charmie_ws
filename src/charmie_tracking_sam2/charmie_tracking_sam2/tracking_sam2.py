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

        # Define points for initialization (example: 3 points)
        self.initial_points = [
            (320, 240),  # Center point
            (300, 220),  # Slightly left and up
            (340, 260)   # Slightly right and down
            # (32, 180),  # Slightly left and up
            # (528, 197)   # Slightly right and down
        ]
        self.point_labels = [1, 1, 1]  # Positive labels for all points


    def main(self):

        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):

            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break

                height, width = frame.shape[:2]  # Get frame dimensions

                # print(height, width)

                if not self.if_init:
                    # Normalize points relative to the frame dimensions
                    multiple_points = [(x , y) for x, y in self.initial_points]

                    # # Define central region for initialization
                    central_bbox = [int(width * 0.3), int(height * 0.2), int(width * 0.7), int(height * 0.8)]  # x1, y1, x2, y2
                    
                    # Define a point for initialization (center of the frame)
                    point_x, point_y = width // 2, height // 2  # Center of the frame
                    points = [(point_x, point_y)]  # Single point
                    labels = [1]  # Label for foreground

                    self.predictor.load_first_frame(frame)
                    self.if_init = True
                    
                    # # Use the point and object ID as prompts
                    _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                        frame_idx=0,  # First frame
                        obj_id=self.initial_obj_id,
                        points=points,  # Use points instead of bbox
                        labels=labels
                    )
                    
                    # # Use points and object ID as prompts to multiple points:
                    # _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt(
                    #     frame_idx=0,  # First frame
                    #     obj_id=self.initial_obj_id,
                    #     points=multiple_points,
                    #     labels=self.point_labels
                    # )
                    
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
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
        # main loop
        while True:
            # do something
            pass
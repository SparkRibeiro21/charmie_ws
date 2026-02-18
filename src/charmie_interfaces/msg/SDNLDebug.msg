# SDNL Debug Message
# This message is meant ONLY for visualization/debug tools (e.g., sdnl_debug.py).
# It contains full 0..2pi curves used to produce the SDNL control output.

std_msgs/Header header                  # Timestamp + frame id (typically "base_link" or robot base frame for angular curves)

uint16 n_samples                        # Number of samples used to represent the curves over [0, 2*pi)
float32 dtheta                          # Angular step per sample [rad], typically (2*pi)/n_samples

float32[] y_attractor                   # Attractor influence curve y_att(theta) for theta in [0, 2*pi), size = n_samples
float32[] y_rep_sum                     # Sum of all repulsor curves y_rep_sum(theta), size = n_samples
float32[] y_final                       # Final curve y_final(theta) = y_att(theta) + y_rep_sum(theta) (optionally + noise), size = n_samples

geometry_msgs/Pose2D robot_map          # Robot pose in MAP frame (x,y,theta) used for computing target direction
geometry_msgs/Pose2D target_pose        # Target pose in MAP frame (x,y,theta) currently being pursued

float32 psi_target_base                 # Target bearing expressed in BASE frame [rad] (useful to draw reference lines)

float32 f_target                        # Instantaneous attractor scalar contribution at current robot heading
float32 f_obstacle                      # Instantaneous repulsor scalar contribution (sum) at current robot heading
float32 f_final                         # Instantaneous final scalar command before conversion/clamping to angular speed

geometry_msgs/Twist cmd_vel             # The final velocity command (cmd_vel) produced by the controller at this time

float32 dist_to_target                  # Current distance to target position [m] (MAP frame)
float32 min_obstacle_dist_edge          # Minimum obstacle distance to robot edge [m] used by speed modulation / safety logic

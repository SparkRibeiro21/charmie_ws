# SDNL Marker Debug message
# Published by the SDNL navigation node so RViz marker tools can visualize the active goal.

std_msgs/Header header                # Timestamp + frame_id (typically "map")

geometry_msgs/Pose2D target_pose      # Current target pose in MAP frame: (x, y) [m], theta [rad]

float32 reached_radius                # Distance threshold [m] used to consider the target position reached
bool active                           # True when a goal is currently active, false when idle / finished / canceled
uint8 mode                            # Navigation mode (e.g., MOVE_TO_POSE, MOVE_ONLY, ROTATE_ONLY, etc.)
bool ignore_obstacles                 # If true, obstacle repulsion is ignored (attractor-only behavior)
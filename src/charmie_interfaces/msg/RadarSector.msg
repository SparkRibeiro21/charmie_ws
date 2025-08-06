# Represents one angular sector of the virtual radar

float64 start_angle       # Sector start angle in radians (relative to robot_base_frame)
float64 end_angle         # Sector end angle in radians
float64 min_distance      # Closest obstacle distance in meters (uses the highest max_obstacle_range from all observation_sources)
geometry_msgs/Point point # Coordinates of the closest point in robot_base_frame
bool has_point            # True if a valid obstacle was found in this sector
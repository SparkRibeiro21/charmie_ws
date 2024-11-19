# Navigation information for Non-Linear Dynamic Systems

geometry_msgs/Pose2D target_coordinates # x,y target coordinates
string move_or_rotate # type of movement intended to do
float32 orientation_absolute # orientation using IMU
bool flag_not_obs # whether obstacles should be considered or not
float32 reached_radius # radius where inside it is assumed the robot has reached the target
bool avoid_people # for inspection, when we want the robot to stop if detectes a person in front od the robot
float32 adjust_distance # linear adjustment: using distance
float32 adjust_direction # the direction of the linear adjustment
float32 adjust_min_dist # adjusts until the distance detected from the closest obstacle is lower than this variable
float32 max_speed # max speed, max because robot uses speed ramps
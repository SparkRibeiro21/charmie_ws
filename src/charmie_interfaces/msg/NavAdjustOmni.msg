# Message to adjust omnidirectional position movement parameters

float32 dx                      # distance to move in x direction (forward/backward)
float32 dy                      # distance to move in y direction (left/right)
float32 ang_obstacle_check      # angle to check for obstacles when moving omnidirectionally
bool safety                     # whether to enable safety distance checks when moving
float32 max_speed               # maximum linear speed
float32 tolerance               # tolerance for reaching the target position
float32 kp                      # proportional gain for the controller
bool enter_house_special_case   # special case for entering the house after door has been opened
bool use_wheel_odometry         # whether to use wheel odometry for movement control

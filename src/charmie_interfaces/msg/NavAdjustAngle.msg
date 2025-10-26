# Message definition for adjusting robot's angle during navigation

float32 angle               # angle to adjust
float32 max_angular_speed   # maximum angular speed
float32 tolerance           # tolerance for reaching the target angle
float32 kp                  # proportional gain for the controller
bool use_wheel_odometry     # whether to use wheel odometry for movement control

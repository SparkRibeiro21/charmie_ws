# All information interpreted regarding a detected person

int32 index
float32 confidence

geometry_msgs/Point position_cam      # position relative to the camera frame
geometry_msgs/Point position_relative # position relative to the base_footprint frame
geometry_msgs/Point position_absolute # position relative to the map frame

geometry_msgs/Point position_cam_head      # position relative to the camera frame
geometry_msgs/Point position_relative_head # position relative to the base_footprint frame
geometry_msgs/Point position_absolute_head # position relative to the map frame

geometry_msgs/Point position_cam_torso      # position relative to the camera frame
geometry_msgs/Point position_relative_torso # position relative to the base_footprint frame
geometry_msgs/Point position_absolute_torso # position relative to the map frame

int32 box_top_left_x
int32 box_top_left_y
int32 box_width
int32 box_height

bool is_box_head
int32 box_head_top_left_x
int32 box_head_top_left_y
int32 box_head_width
int32 box_head_height

MaskDetection mask      # not implemented yet for detected people (only for objects)
MaskDetection mask_norm # not implemented yet for detected people (only for objects)

string room_location
string furniture_location

string arm_raised
string body_posture
string pointing_at
string pointing_with_arm
float32 height

string ethnicity
float32 ethnicity_probability
string age_estimate
float32 age_estimate_probability
string gender
float32 gender_probability

string shirt_color
RGB shirt_rgb
string pants_color
RGB pants_rgb

int32 kp_nose_x
int32 kp_nose_y
float32 kp_nose_conf

int32 kp_eye_left_x
int32 kp_eye_left_y
float32 kp_eye_left_conf

int32 kp_eye_right_x
int32 kp_eye_right_y
float32 kp_eye_right_conf

int32 kp_ear_left_x
int32 kp_ear_left_y
float32 kp_ear_left_conf

int32 kp_ear_right_x
int32 kp_ear_right_y
float32 kp_ear_right_conf

int32 kp_shoulder_left_x
int32 kp_shoulder_left_y
float32 kp_shoulder_left_conf

int32 kp_shoulder_right_x
int32 kp_shoulder_right_y
float32 kp_shoulder_right_conf

int32 kp_elbow_left_x
int32 kp_elbow_left_y
float32 kp_elbow_left_conf

int32 kp_elbow_right_x
int32 kp_elbow_right_y
float32 kp_elbow_right_conf

int32 kp_wrist_left_x
int32 kp_wrist_left_y
float32 kp_wrist_left_conf

int32 kp_wrist_right_x
int32 kp_wrist_right_y
float32 kp_wrist_right_conf

int32 kp_hip_left_x
int32 kp_hip_left_y
float32 kp_hip_left_conf

int32 kp_hip_right_x
int32 kp_hip_right_y
float32 kp_hip_right_conf

int32 kp_knee_left_x
int32 kp_knee_left_y
float32 kp_knee_left_conf

int32 kp_knee_right_x
int32 kp_knee_right_y
float32 kp_knee_right_conf

int32 kp_ankle_left_x
int32 kp_ankle_left_y
float32 kp_ankle_left_conf

int32 kp_ankle_right_x
int32 kp_ankle_right_y
float32 kp_ankle_right_conf

int32 body_center_x
int32 body_center_y

int32 head_center_x
int32 head_center_y

sensor_msgs/Image image_rgb_frame
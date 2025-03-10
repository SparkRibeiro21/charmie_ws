# All information interpreted regarding a detected object

string object_name
string object_class
int32 index
float32 confidence

geometry_msgs/Point position_relative # position relative to the robot frame
geometry_msgs/Point position_absolute # position relative to the map frame

int32 box_top_left_x
int32 box_top_left_y
int32 box_width
int32 box_height

MaskDetection mask
MaskDetection mask_norm

string room_location
string furniture_location

int32 box_center_x
int32 box_center_y

string camera

float32 orientation

sensor_msgs/Image image_rgb_frame
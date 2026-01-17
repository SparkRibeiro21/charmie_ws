# All information interpreted regarding a detected object

string object_name
string object_class
int32 index
float32 confidence

geometry_msgs/Point position_cam      # position relative to the camera frame
geometry_msgs/Point position_relative # position relative to the base_footprint frame
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

float32 orientation # not implemented yet (should be 6D pose)

sensor_msgs/Image image_rgb_frame

# Added data from configuration_files objects.json
float32 cf_width
float32 cf_length
float32 cf_height
string cf_shape
string cf_can_pick
string cf_std_pick
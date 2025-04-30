# SAM2 Tracking activated information

geometry_msgs/Point centroid   # pixel centroid coordinates
ListOfMaskDetections mask      # mask of the tracking selection

geometry_msgs/Point position_cam      # position relative to the camera frame
geometry_msgs/Point position_relative # position relative to the robot frame
geometry_msgs/Point position_absolute # position relative to the map frame

# ListOfMaskDetections mask_norm # normalized mask of the tracking selection
# sensor_msgs/Image binary_mask  # binary mask of the tracking selection
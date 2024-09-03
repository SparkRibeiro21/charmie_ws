# x,y,z coordinates of a received point cloud

geometry_msgs/Point center_coords # calculated center of all points inside bounding box - filtered TR
geometry_msgs/Point[] requested_point_coords # exact coordinates of specific requested pixels
geometry_msgs/Point[] bbox_point_coords # all x,y,z points of the bounding box
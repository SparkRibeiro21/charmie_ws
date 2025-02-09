import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import json
from pathlib import Path

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        # self.publisher = self.create_publisher(Marker, "visualization_marker", 10)
        self.publisher_marker_array_rooms =             self.create_publisher(MarkerArray, "visualization_marker_array_rooms", 10)
        self.publisher_marker_array_rooms_names =       self.create_publisher(MarkerArray, "visualization_marker_array_rooms_names", 10)
        self.publisher_marker_array_furniture =         self.create_publisher(MarkerArray, "visualization_marker_array_furniture", 10)
        self.publisher_marker_array_furniture_names =   self.create_publisher(MarkerArray, "visualization_marker_array_furniture_names", 10)
        self.publisher_marker_array_navigations =       self.create_publisher(MarkerArray, "visualization_marker_array_navigations", 10)
        self.publisher_marker_array_navigations_names = self.create_publisher(MarkerArray, "visualization_marker_array_navigations_names", 10)


        # info regarding the paths for the recorded files intended to be played
        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        self.home = str(Path.home())
        self.midpath = "charmie_ws/src/configuration_files"
        self.complete_path = self.home+'/'+self.midpath+'/'

        # Open all configuration files
        try:
            with open(self.complete_path + 'rooms.json', encoding='utf-8') as json_file:
                self.house_rooms = json.load(json_file)
            # print(self.house_rooms)

            with open(self.complete_path + 'furniture.json', encoding='utf-8') as json_file:
                self.house_furniture = json.load(json_file)
            # print(self.house_furniture)

        except:
            print("Could NOT import data from json configuration files. (objects_list, house_rooms and house_furniture)")
        

        self.COLOR_LIST = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
            (0.5, 0.5, 0.5)   # Gray
        ]

        self.names_text_size = 0.2


        # self.timer = self.create_timer(1.0, self.publish_marker)  # Publish every 1 second
        # self.timer = self.create_timer(1.0, self.publish_marker_array)  # Publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_all_marker_arrays)  # Publish every 1 second


    def publish_all_marker_arrays(self):
        self.publish_marker_array_rooms()
        self.publish_marker_array_furniture()
        self.publish_marker_array_navigation()


    def publish_marker_array_rooms(self):

        marker_array = MarkerArray()
        marker_array_names = MarkerArray()

        ### FALTA:
        # ORIENTATION

        height = 0.1
        for index, room in enumerate(self.house_rooms):
            # print(index, room)
            marker = Marker()

            # Header - Defines frame and timestamp
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            # Namespace and ID (useful when publishing multiple markers)
            marker.ns = room['name']
            marker.id = index  # Each marker must have a unique ID
            # Marker Type (Choose shape)
            marker.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
            # Marker Action
            marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

            marker.pose.position.x = (room['top_left_coords'][0] + room['bot_right_coords'][0]) / 2  # Set the X coordinate
            marker.pose.position.y = (room['top_left_coords'][1] + room['bot_right_coords'][1]) / 2  # Set the X coordinate
            marker.pose.position.z = height/2  # Set the Z coordinate

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0  # No rotation

            marker.scale.x = abs(room['top_left_coords'][0] - room['bot_right_coords'][0]) # + 2*margin_x  # Width
            marker.scale.y = abs(room['top_left_coords'][1] - room['bot_right_coords'][1]) # + 2*margin_y # Width
            marker.scale.z = height  # Height

            rgb_v = self.COLOR_LIST[index % len(self.COLOR_LIST)]
            # print(index % len(self.COLOR_LIST), rgb_v)
            # Color (RGBA format, values from 0 to 1)
            marker.color.r = rgb_v[0] # 0.0  # Red
            marker.color.g = rgb_v[1] # 1.0  # Green
            marker.color.b = rgb_v[2] # 1.0  # Blue
            marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
            
            # Lifetime (0 = forever, otherwise, disappears after X seconds)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # Frame behavior (Keeps marker always facing the camera if enabled)
            marker.frame_locked = False
            
            marker_array.markers.append(marker)

            marker_name = Marker()
            marker_name.header.frame_id = "map"
            marker_name.header.stamp = self.get_clock().now().to_msg()
            marker_name.ns = room['name']+" Name"
            marker_name.id = index+len(self.house_rooms)
            marker_name.type = Marker.TEXT_VIEW_FACING
            marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
            marker_name.pose.position.x = (room['top_left_coords'][0] + room['bot_right_coords'][0]) / 2  # Set the X coordinate
            marker_name.pose.position.y = (room['top_left_coords'][1] + room['bot_right_coords'][1]) / 2  # Set the X coordinate
            marker_name.pose.position.z = height/2  # Set the Z coordinate
            marker_name.pose.orientation.w = 1.0  # No rotation
            marker_name.scale.z = self.names_text_size  # Height
            marker_name.text = room['name'].replace(" ", "_")
            marker_name.color.r = 1.0  # Red
            marker_name.color.g = 1.0  # Green
            marker_name.color.b = 1.0  # Blue
            marker_name.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

            marker_array_names.markers.append(marker_name)

        self.publisher_marker_array_rooms.publish(marker_array)
        self.publisher_marker_array_rooms_names.publish(marker_array_names)


    def publish_marker_array_furniture(self):

        marker_array = MarkerArray()
        marker_array_names = MarkerArray()

        ### FALTA:
        # ORIENTATION

        for index, furniture in enumerate(self.house_furniture):
            marker = Marker()

            # Header - Defines frame and timestamp
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            # Namespace and ID (useful when publishing multiple markers)
            marker.ns = furniture['name']
            marker.id = index  # Each marker must have a unique ID
            # Marker Type (Choose shape)
            marker.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
            # Marker Action
            marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE


            marker.pose.position.x = (furniture['top_left_coords'][0] + furniture['bot_right_coords'][0]) / 2  # Set the X coordinate
            marker.pose.position.y = (furniture['top_left_coords'][1] + furniture['bot_right_coords'][1]) / 2  # Set the X coordinate
            marker.pose.position.z = furniture['height']/2  # Set the Z coordinate

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0  # No rotation

            marker.scale.x = abs(furniture['top_left_coords'][0] - furniture['bot_right_coords'][0]) # + 2*margin_x  # Width
            marker.scale.y = abs(furniture['top_left_coords'][1] - furniture['bot_right_coords'][1]) # + 2*margin_y # Width
            marker.scale.z = furniture['height']  # Height

            # rgb_v = self.COLOR_LIST[index % len(self.COLOR_LIST)]
            # print(ctr % len(self.COLOR_LIST), rgb_v)
            # Color (RGBA format, values from 0 to 1)
            # marker.color.r = rgb_v[0] # 0.0  # Red
            # marker.color.g = rgb_v[1] # 1.0  # Green
            # marker.color.b = rgb_v[2] # 1.0  # Blue
            # marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
            
            # Color (RGBA format, values from 0 to 1)
            marker.color.r = 0.5 # 0.0  # Red
            marker.color.g = 0.5 # 1.0  # Green
            marker.color.b = 0.5 # 1.0  # Blue
            marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
            
            # Lifetime (0 = forever, otherwise, disappears after X seconds)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # Frame behavior (Keeps marker always facing the camera if enabled)
            marker.frame_locked = False
            
            
            marker_array.markers.append(marker)

            marker_name = Marker()
            marker_name.header.frame_id = "map"
            marker_name.header.stamp = self.get_clock().now().to_msg()
            marker_name.ns = furniture['name']+" Name"
            marker_name.id = index+len(self.house_rooms)
            marker_name.type = Marker.TEXT_VIEW_FACING
            marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
            marker_name.pose.position.x = (furniture['top_left_coords'][0] + furniture['bot_right_coords'][0]) / 2  # Set the X coordinate
            marker_name.pose.position.y = (furniture['top_left_coords'][1] + furniture['bot_right_coords'][1]) / 2  # Set the X coordinate
            marker_name.pose.position.z = furniture['height']/2  # Set the Z coordinate
            marker_name.pose.orientation.w = 1.0  # No rotation
            marker_name.scale.z = self.names_text_size # Height
            marker_name.text = furniture['name'].replace(" ", "_")
            marker_name.color.r = 1.0  # Red
            marker_name.color.g = 1.0  # Green
            marker_name.color.b = 1.0  # Blue
            marker_name.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

            marker_array_names.markers.append(marker_name)

        self.publisher_marker_array_furniture.publish(marker_array)
        self.publisher_marker_array_furniture_names.publish(marker_array_names)


    def publish_marker_array_navigation(self):

        marker_array = MarkerArray()
        marker_array_names = MarkerArray()

        height = 0.185 # floor_to_base(0.05) + half_of_move_base_haight(0.27/2) 
        for index, room in enumerate(self.house_rooms):
            # print(index, room)
            marker = Marker()

            # Header - Defines frame and timestamp
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            # Namespace and ID (useful when publishing multiple markers)
            marker.ns = room['name']+"_nav"
            marker.id = 100+index  # Each marker must have a unique ID
            # Marker Type (Choose shape)
            marker.type = Marker.ARROW  # Other options: SPHERE, CYLINDER, ARROW, etc.
            # Marker Action
            marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

            # Position (Start of Arrow)
            marker.pose.position.x = room['nav_coords'][0]  # Set the X coordinate
            marker.pose.position.y = room['nav_coords'][1]  # Set the X coordinate
            marker.pose.position.z = height  # Set the Z coordinate
            
            # print(marker.ns, room['nav_coords'][2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(math.radians(room['nav_coords'][2])/2)  # Rotate 45 degrees around the Z axis
            marker.pose.orientation.w = math.cos(math.radians(room['nav_coords'][2])/2)  # Rotate 45 degrees around the Z axis

            # Scale (Defines Arrow Size)
            marker.scale.x = 1.0  # Shaft length
            marker.scale.y = 0.1  # Shaft thickness
            marker.scale.z = 0.1  # Arrowhead size

            # Color (RGBA format, values from 0 to 1)
            marker.color.r = 0.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 1.0  # Blue
            marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
            
            # Lifetime (0 = forever, otherwise, disappears after X seconds)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # Frame behavior (Keeps marker always facing the camera if enabled)
            marker.frame_locked = False
            
            marker_array.markers.append(marker)

            
            marker_name = Marker()
            marker_name.header.frame_id = "map"
            marker_name.header.stamp = self.get_clock().now().to_msg()
            marker_name.ns = room['name']+"_nav"+" Name"
            marker_name.id = index+len(self.house_rooms)
            marker_name.type = Marker.TEXT_VIEW_FACING
            marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
            marker_name.pose.position.x = room['nav_coords'][0]  # Set the X coordinate
            marker_name.pose.position.y = room['nav_coords'][1]  # Set the X coordinate
            marker_name.pose.position.z = height  # Set the Z coordinate
            marker_name.pose.orientation.w = 1.0  # No rotation
            marker_name.scale.z = self.names_text_size  # Height
            marker_name.text = room['name'].replace(" ", "_")+"_Nav"
            marker_name.color.r = 1.0  # Red
            marker_name.color.g = 1.0  # Green
            marker_name.color.b = 1.0  # Blue
            marker_name.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

            marker_array_names.markers.append(marker_name)

        for index, furniture in enumerate(self.house_furniture):
            marker = Marker()

            # Header - Defines frame and timestamp
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            # Namespace and ID (useful when publishing multiple markers)
            marker.ns = furniture['name']+"_nav"
            marker.id = 100+index  # Each marker must have a unique ID
            # Marker Type (Choose shape)
            marker.type = Marker.ARROW  # Other options: SPHERE, CYLINDER, ARROW, etc.
            # Marker Action
            marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

            # Position (Start of Arrow)
            marker.pose.position.x = furniture['nav_coords'][0]  # Set the X coordinate
            marker.pose.position.y = furniture['nav_coords'][1]  # Set the X coordinate
            marker.pose.position.z = height  # Set the Z coordinate
            
            # print(marker.ns, furniture['nav_coords'][2])
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(math.radians(furniture['nav_coords'][2])/2)  # Rotate 45 degrees around the Z axis
            marker.pose.orientation.w = math.cos(math.radians(furniture['nav_coords'][2])/2)  # Rotate 45 degrees around the Z axis

            # Scale (Defines Arrow Size)
            marker.scale.x = 1.0  # Shaft length
            marker.scale.y = 0.1  # Shaft thickness
            marker.scale.z = 0.1  # Arrowhead size

            # Color (RGBA format, values from 0 to 1)
            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 1.0  # Blue
            marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
            
            # Lifetime (0 = forever, otherwise, disappears after X seconds)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # Frame behavior (Keeps marker always facing the camera if enabled)
            marker.frame_locked = False
            
            marker_array.markers.append(marker)

            
            marker_name = Marker()
            marker_name.header.frame_id = "map"
            marker_name.header.stamp = self.get_clock().now().to_msg()
            marker_name.ns = furniture['name']+"_nav"+" Name"
            marker_name.id = index+len(self.house_furniture)
            marker_name.type = Marker.TEXT_VIEW_FACING
            marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
            marker_name.pose.position.x = furniture['nav_coords'][0]  # Set the X coordinate
            marker_name.pose.position.y = furniture['nav_coords'][1]  # Set the X coordinate
            marker_name.pose.position.z = height  # Set the Z coordinate
            marker_name.pose.orientation.w = 1.0  # No rotation
            marker_name.scale.z = self.names_text_size  # Height
            marker_name.text = furniture['name'].replace(" ", "_")+"_Nav"
            marker_name.color.r = 1.0  # Red
            marker_name.color.g = 1.0  # Green
            marker_name.color.b = 1.0  # Blue
            marker_name.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

            marker_array_names.markers.append(marker_name)

        self.publisher_marker_array_navigations.publish(marker_array)
        self.publisher_marker_array_navigations_names.publish(marker_array_names)

    """
    def publish_marker_array(self):
        marker_array = MarkerArray()
        
        sofa = Marker()

        # Header - Defines frame and timestamp
        sofa.header.frame_id = "map"  # Change to "odom" or "base_link" if needed
        sofa.header.stamp = self.get_clock().now().to_msg()
        # Namespace and ID (useful when publishing multiple markers)
        sofa.ns = "sofa"
        sofa.id = 1  # Each marker must have a unique ID
        # Marker Type (Choose shape)
        sofa.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
        # Marker Action
        sofa.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

        corner_top_left  = [3.610-0, 3.836-0]
        # corner_top_right = [3.702-0, 1.846-0]
        # corner_bot_left  = [2.731-0, 3.828-0]
        corner_bot_right = [2.807-0, 1.899-0]
        margin_x = 0.0
        margin_y = 0.0
        height = 0.4

        sofa.pose.position.x = (corner_top_left[0] + corner_bot_right[0]) / 2  # Set the X coordinate
        sofa.pose.position.y = (corner_top_left[1] + corner_bot_right[1]) / 2  # Set the X coordinate
        sofa.pose.position.z = height/2  # Set the Z coordinate
        
        sofa.pose.orientation.x = 0.0
        sofa.pose.orientation.y = 0.0
        sofa.pose.orientation.z = 0.0
        sofa.pose.orientation.w = 1.0  # No rotation

        sofa.scale.x = abs(corner_top_left[0] - corner_bot_right[0]) + 2*margin_x  # Width
        sofa.scale.y = abs(corner_top_left[1] - corner_bot_right[1]) + 2*margin_y # Width
        sofa.scale.z = height  # Height

        # Color (RGBA format, values from 0 to 1)
        sofa.color.r = 0.0  # Red
        sofa.color.g = 1.0  # Green
        sofa.color.b = 1.0  # Blue
        sofa.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
        
        # Lifetime (0 = forever, otherwise, disappears after X seconds)
        sofa.lifetime.sec = 0
        sofa.lifetime.nanosec = 0

        # Frame behavior (Keeps marker always facing the camera if enabled)
        sofa.frame_locked = False




        cabinet = Marker()

        # Header - Defines frame and timestamp
        cabinet.header.frame_id = "map"  # Change to "odom" or "base_link" if needed
        cabinet.header.stamp = self.get_clock().now().to_msg()
        # Namespace and ID (useful when publishing multiple markers)
        cabinet.ns = "cabinet"
        cabinet.id = 2  # Each marker must have a unique ID
        # Marker Type (Choose shape)
        cabinet.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
        # Marker Action
        cabinet.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

        corner_top_left  = [3.610-1, 3.836-2]
        # corner_top_right = [3.702-1, 1.846-2]
        # corner_bot_left  = [2.731-1, 3.828-3]
        corner_bot_right = [2.807-1, 1.899-3]
        margin_x = 0.0
        margin_y = 0.0
        height = 0.4

        cabinet.pose.position.x = (corner_top_left[0] + corner_bot_right[0]) / 4  # Set the X coordinate
        cabinet.pose.position.y = (corner_top_left[1] + corner_bot_right[1]) / 4  # Set the X coordinate
        cabinet.pose.position.z = height/2  # Set the Z coordinate
        
        # ROTATION:
        theta = math.radians(45)
        cabinet.pose.orientation.x = 0.0
        cabinet.pose.orientation.y = 0.0
        cabinet.pose.orientation.z = math.sin(theta/2)  # Rotate 45 degrees around the Z axis
        cabinet.pose.orientation.w = math.cos(theta/2)  # Rotate 45 degrees around the Z axis


        cabinet.scale.x = abs(corner_top_left[0] - corner_bot_right[0]) + 2*margin_x  # Width
        cabinet.scale.y = abs(corner_top_left[1] - corner_bot_right[1]) + 2*margin_y # Width
        cabinet.scale.z = height  # Height

        # Color (RGBA format, values from 0 to 1)
        cabinet.color.r = 1.0  # Red
        cabinet.color.g = 1.0  # Green
        cabinet.color.b = 1.0  # Blue
        cabinet.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
        
        # Lifetime (0 = forever, otherwise, disappears after X seconds)
        cabinet.lifetime.sec = 0
        cabinet.lifetime.nanosec = 0

        # Frame behavior (Keeps marker always facing the camera if enabled)
        cabinet.frame_locked = False


        marker_array.markers.append(sofa)
        marker_array.markers.append(cabinet)

        # Publish the marker
        self.publisher_array.publish(marker_array)
        self.get_logger().info("Marker Array published!")

    """
    

    """
    def publish_marker(self):
        marker = Marker()
        
        # Header - Defines frame and timestamp
        marker.header.frame_id = "map"  # Change to "odom" or "base_link" if needed
        marker.header.stamp = self.get_clock().now().to_msg()

        # Namespace and ID (useful when publishing multiple markers)
        marker.ns = "basic_shapes"
        marker.id = 1  # Each marker must have a unique ID

        # Marker Type (Choose shape)
        marker.type = Marker.CUBE  # Other options: SPHERE, CYLINDER, ARROW, etc.
        
        # Marker Action
        marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE


        corner_top_left  = [3.610-0, 3.836-0]
        corner_top_right = [3.702-0, 1.846-0]
        corner_bot_left  = [2.731-0, 3.828-0]
        corner_bot_right = [2.807-0, 1.899-0]
        margin_x = 0.0
        margin_y = 0.0
        height = 0.4

        # Position (x, y, z) and Orientation (quaternion w, x, y, z)
        # marker.pose.position.x = 1.0  # Set the X coordinate
        # marker.pose.position.y = 2.0  # Set the Y coordinate
        # marker.pose.position.z = 0.0  # Set the Z coordinate

        marker.pose.position.x = (corner_top_left[0] + corner_top_right[0] + corner_bot_left[0] + corner_bot_right[0]) / 4  # Set the X coordinate
        marker.pose.position.y = (corner_top_left[1] + corner_top_right[1] + corner_bot_left[1] + corner_bot_right[1]) / 4  # Set the X coordinate
        marker.pose.position.z = height/2  # Set the Z coordinate


        # ROTATION:
        theta = math.radians(45)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(theta/2)  # Rotate 45 degrees around the Z axis
        marker.pose.orientation.w = math.cos(theta/2)  # Rotate 45 degrees around the Z axis


        # NO ROTATION: 
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0  # No rotation

        # Scale (size of marker)
        # marker.scale.x = 0.5  # Width
        # marker.scale.y = 0.5  # Depth
        # marker.scale.z = 0.5  # Height
        
        marker.scale.x = abs(corner_top_left[0] - corner_bot_left[0]) + 2*margin_x  # Width
        marker.scale.y = abs(corner_top_left[1] - corner_top_right[1]) + 2*margin_y # Width
        marker.scale.z = height  # Height

        # Color (RGBA format, values from 0 to 1)
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 1.0  # Blue
        marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

        # Lifetime (0 = forever, otherwise, disappears after X seconds)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Frame behavior (Keeps marker always facing the camera if enabled)
        marker.frame_locked = False

        # Publish the marker
        self.publisher.publish(marker)
        self.get_logger().info("Marker published!")
    """


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from charmie_interfaces.msg import DetectedPerson, DetectedObject, ListOfDetectedPerson, ListOfDetectedObject, TrackingMask, RadarData
from geometry_msgs.msg import Point
import math
import json
from pathlib import Path
import time

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

        self.publisher_marker_array_detected_person =   self.create_publisher(MarkerArray, "visualization_marker_array_detected_person", 10)
        self.publisher_marker_array_detected_object =   self.create_publisher(MarkerArray, "visualization_marker_array_detected_object", 10)
        self.publisher_marker_array_tracking =          self.create_publisher(MarkerArray, "visualization_marker_array_tracking", 10)
        
        self.publisher_marker_array_radar =             self.create_publisher(MarkerArray, "visualization_marker_array_radar", 10)
        
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)
        # Yolo Objects
        self.objects_filtered_subscriber = self.create_subscription(ListOfDetectedObject, 'objects_all_detected_filtered', self.object_detected_filtered_callback, 10)
        # Tracking (SAM2)
        self.tracking_mask_subscriber = self.create_subscription(TrackingMask, 'tracking_mask', self.tracking_mask_callback, 10)
        # Radar
        self.radar_data_subscriber = self.create_subscription(RadarData, "radar/data", self.radar_data_callback, 10)

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
        

        self.detected_people = ListOfDetectedPerson()
        self.detected_object = ListOfDetectedObject()
        self.previous_marker_array_detected_people = ListOfDetectedPerson() 
        self.tracking = TrackingMask()
        self.radar = RadarData()
        self.aux_time = time.time()

        self.COLOR_LIST = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
            (0.5, 0.5, 0.5)   # Gray
        ]

        self.NAMES_TEXT_SIZE = 0.2


        # self.timer = self.create_timer(1.0, self.publish_marker)  # Publish every 1 second
        # self.timer = self.create_timer(1.0, self.publish_marker_array)  # Publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_all_marker_arrays)  # Publish every 1 second

    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people
        self.publish_marker_array_detected_person()

    def object_detected_filtered_callback(self, det_object: ListOfDetectedObject):
        self.detected_object = det_object
        self.publish_marker_array_detected_object()

    def tracking_mask_callback(self, track: TrackingMask):
        self.tracking = track
        self.publish_marker_array_tracking()
    
    def radar_data_callback(self, radar: RadarData):
        self.radar = radar
        self.publish_marker_array_radar()

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
            marker_name.scale.z = self.NAMES_TEXT_SIZE  # Height
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
            marker_name.scale.z = self.NAMES_TEXT_SIZE # Height
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
            marker_name.scale.z = self.NAMES_TEXT_SIZE  # Height
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
            marker_name.scale.z = self.NAMES_TEXT_SIZE  # Height
            marker_name.text = furniture['name'].replace(" ", "_")+"_Nav"
            marker_name.color.r = 1.0  # Red
            marker_name.color.g = 1.0  # Green
            marker_name.color.b = 1.0  # Blue
            marker_name.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)

            marker_array_names.markers.append(marker_name)

        self.publisher_marker_array_navigations.publish(marker_array)
        self.publisher_marker_array_navigations_names.publish(marker_array_names)

    def publish_marker_array_detected_person(self):

        marker_array = MarkerArray()
        # marker_array_names = MarkerArray()

        # x = -1.0
        # y = 0.0
        # height = 1.80

        person_size = 0.4
        head_size = 0.3

        # list_of_detected_person_indexes = []

        # for person in self.detected_people.persons:
        #     if person.index > 0:
        #         list_of_detected_person_indexes.append(person.index)
                
        # print("atuais:", list_of_detected_person_indexes)
        
        # I have to dynamically remove all previous persons that are no longer on the image otherwise rviz will always show them
        # for person in self.previous_marker_array_detected_people.persons:
        #     if person.index > 0:
        #         print("Comparing:", person.index)
        #         if person.index not in list_of_detected_person_indexes:

        #             print("REMOVED:", person.index)
                    
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Detected_people_B"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Detected_people_H"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Detected_people_N"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        # self.publisher_marker_array_detected_person.publish(marker_array)

        # marker_array.markers.clear()

        ### FALTA:
        # ORIENTATION

        for person in self.detected_people.persons:
            if person.index > 0:

                # print(person.index, round(person.position_absolute.x, 2), round(person.position_absolute.y, 2), round(person.position_absolute.z, 2), person.height, person.room_location, person.furniture_location)
                
                marker = Marker()

                # Header - Defines frame and timestamp
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                # Namespace and ID (useful when publishing multiple markers)
                marker.ns = "Detected_people_B"
                marker.id = person.index  # Each marker must have a unique ID
                # Marker Type (Choose shape)
                marker.type = Marker.CYLINDER  # Other options: SPHERE, CYLINDER, ARROW, etc.
                # Marker Action
                marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

                marker.pose.position.x = person.position_absolute.x  # Set the X coordinate
                marker.pose.position.y = person.position_absolute.y  # Set the X coordinate
                marker.pose.position.z = (person.height-head_size)/2  # Set the Z coordinate

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0  # No rotation

                marker.scale.x = person_size # Width
                marker.scale.y = person_size # Width
                marker.scale.z = person.height-head_size  # Height
                
                # Color (RGBA format, values from 0 to 1)
                marker.color.r = 1.0 # 0.0  # Red
                marker.color.g = 1.0 # 1.0  # Green
                marker.color.b = 0.0 # 1.0  # Blue
                marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
                
                # Lifetime (0 = forever, otherwise, disappears after X seconds)
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 0

                # Frame behavior (Keeps marker always facing the camera if enabled)
                marker.frame_locked = False
                
                marker_array.markers.append(marker)


                # for index, furniture in enumerate(self.house_furniture):
                marker = Marker()

                # Header - Defines frame and timestamp
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                # Namespace and ID (useful when publishing multiple markers)
                marker.ns = "Detected_people_H"
                marker.id = person.index  # Each marker must have a unique ID
                # Marker Type (Choose shape)
                marker.type = Marker.SPHERE  # Other options: SPHERE, CYLINDER, ARROW, etc.
                # Marker Action
                marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

                marker.pose.position.x = person.position_absolute.x  # Set the X coordinate
                marker.pose.position.y = person.position_absolute.y  # Set the X coordinate
                marker.pose.position.z = person.height-(head_size/2)  # Set the Z coordinate

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0  # No rotation

                marker.scale.x = head_size # Width
                marker.scale.y = head_size # Width
                marker.scale.z = head_size # Height
                
                # Color (RGBA format, values from 0 to 1)
                marker.color.r = 1.0 # 0.0  # Red
                marker.color.g = 1.0 # 1.0  # Green
                marker.color.b = 0.0 # 1.0  # Blue
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
                marker_name.ns = "Detected_people_N"
                marker_name.id = person.index
                marker_name.type = Marker.TEXT_VIEW_FACING
                marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
                marker_name.pose.position.x = person.position_absolute.x  # Set the X coordinate
                marker_name.pose.position.y = person.position_absolute.y  # Set the Y coordinate
                marker_name.pose.position.z =  person.height-(head_size/2)  # Set the Z coordinate
                marker_name.pose.orientation.w = 1.0  # No rotation
                marker_name.scale.z = self.NAMES_TEXT_SIZE  # Height
                marker_name.text = str(person.index)
                marker_name.color.r = 0.0  # Red
                marker_name.color.g = 0.0  # Green
                marker_name.color.b = 0.0  # Blue
                marker_name.color.a = 1.0  # Alpha (1.0 = fully visible, 0.0 = invisible)

                marker_array.markers.append(marker_name)
            
        self.publisher_marker_array_detected_person.publish(marker_array)
    
        # self.previous_marker_array_detected_people = self.detected_people

    def publish_marker_array_detected_object(self):

        marker_array = MarkerArray()

        object_size = 0.1

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Detected_object_B"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Detected_object_N"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        # marker_array.markers.clear()

        ### FALTA:
        # ORIENTATION

        for object_ in self.detected_object.objects:
            if object_.index > 0:

                conf = f"{object_.confidence * 100:.0f}%"
                x_ = f"{object_.position_absolute.x:4.2f}"
                y_ = f"{object_.position_absolute.y:5.2f}"
                z_ = f"{object_.position_absolute.z:5.2f}"
                print(f"{'ID:'+str(object_.index):<7} {object_.object_name:<17} {conf:<3} {object_.camera} ({x_}, {y_}, {z_}) {object_.room_location:<12} {object_.furniture_location}")
                
                marker = Marker()

                # Header - Defines frame and timestamp
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                # Namespace and ID (useful when publishing multiple markers)
                marker.ns = "Detected_object_B"
                marker.id = object_.index  # Each marker must have a unique ID
                # Marker Type (Choose shape)
                marker.type = Marker.CYLINDER  # Other options: SPHERE, CYLINDER, ARROW, etc.
                # Marker Action
                marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

                marker.pose.position.x = object_.position_absolute.x  # Set the X coordinate
                marker.pose.position.y = object_.position_absolute.y  # Set the X coordinate
                marker.pose.position.z = object_.position_absolute.z  # Set the Z coordinate

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0  # No rotation

                marker.scale.x = object_size # Width
                marker.scale.y = object_size # Width
                marker.scale.z = object_size  # Height
                
                # Color (RGBA format, values from 0 to 1)
                marker.color.r = 0.0 # 0.0  # Red
                marker.color.g = 0.0 # 1.0  # Green
                marker.color.b = 1.0 # 1.0  # Blue
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
                marker_name.ns = "Detected_object_N"
                marker_name.id = object_.index
                marker_name.type = Marker.TEXT_VIEW_FACING
                marker_name.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE
                marker_name.pose.position.x = object_.position_absolute.x  # Set the X coordinate
                marker_name.pose.position.y = object_.position_absolute.y  # Set the Y coordinate
                marker_name.pose.position.z = object_.position_absolute.z  # Set the Z coordinate
                marker_name.pose.orientation.w = 1.0  # No rotation
                marker_name.scale.z = self.NAMES_TEXT_SIZE/2  # Height
                marker_name.text = str(object_.object_name).replace(" ","_")
                marker_name.color.r = 0.0  # Red
                marker_name.color.g = 0.0  # Green
                marker_name.color.b = 0.0  # Blue
                marker_name.color.a = 1.0  # Alpha (1.0 = fully visible, 0.0 = invisible)

                marker_array.markers.append(marker_name)
            
        self.publisher_marker_array_detected_object.publish(marker_array)
    
    def publish_marker_array_tracking(self):

        marker_array = MarkerArray()

        object_size = 0.4 # same as person cylinder
        temp_height = 1.8

        ### DO I NEED TO DELETE, SINCE IT IS ONLY ONE OBJECT?
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Track"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)


        track_entity = self.tracking

        x_ = f"{track_entity.position_cam.x:4.2f}"
        y_ = f"{track_entity.position_cam.y:5.2f}"
        z_ = f"{track_entity.position_cam.z:5.2f}"
        # print(f"({x_}, {y_}, {z_}) {track_entity.room_location:<12} {track_entity.furniture_location}")
        print(f"({x_}, {y_}, {z_})")
        
        marker = Marker()

        # Header - Defines frame and timestamp
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        # Namespace and ID (useful when publishing multiple markers)
        marker.ns = "Track"
        marker.id = 1  # Each marker must have a unique ID
        # Marker Type (Choose shape)
        marker.type = Marker.CYLINDER  # Other options: SPHERE, CYLINDER, ARROW, etc.
        # Marker Action
        marker.action = Marker.ADD  # Can be ADD, MODIFY, or DELETE

        marker.pose.position.x = track_entity.position_absolute.x  # Set the X coordinate
        marker.pose.position.y = track_entity.position_absolute.y  # Set the X coordinate
        marker.pose.position.z = temp_height/2 # abs(track_entity.position_cam.z)  # Set the Z coordinate

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation

        marker.scale.x = object_size # Width
        marker.scale.y = object_size # Width
        marker.scale.z = temp_height # abs(2*track_entity.position_cam.z)  # Height
        
        # Color (RGBA format, values from 0 to 1)
        marker.color.r = 0.0 # 0.0  # Red
        marker.color.g = 1.0 # 1.0  # Green
        marker.color.b = 1.0 # 1.0  # Blue
        marker.color.a = 0.5  # Alpha (1.0 = fully visible, 0.0 = invisible)
        
        # Lifetime (0 = forever, otherwise, disappears after X seconds)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Frame behavior (Keeps marker always facing the camera if enabled)
        marker.frame_locked = False
        
        marker_array.markers.append(marker)

            
        self.publisher_marker_array_tracking.publish(marker_array)
                        
    def publish_marker_array_radar(self):
        curr_time = time.time()
        marker_array = MarkerArray()
        object_size = 0.1
        max_radar_range = 4.0 # meters

        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = "base_footprint"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Radar_sector_closest_point"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        delete_marker = Marker()
        delete_marker.header.frame_id = "base_footprint"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Radar_sector_arc"
        delete_marker.id = 0  # Use the same ID to delete it
        delete_marker.action = Marker.DELETEALL  # REMOVE from RViz
        marker_array.markers.append(delete_marker)

        delete_marker = Marker()
        delete_marker.header.frame_id = "base_footprint"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "Radar_sector_floor_line"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        print("[radar] --- Radar Data: ---")
        print("[radar] d_t:", round(curr_time - self.aux_time, 5))
        # print("[radar] header:", self.radar.header)
        print("[radar] No of Sectors:", self.radar.number_of_sectors)
        print("[radar] Sector Ang Range:", round(self.radar.sector_ang_range,3), "(rad), ", round(self.radar.sector_ang_range*180.0/math.pi, 1), "(deg)")
        
        for i, sector in enumerate(self.radar.sectors):
            print(f"\t[radar] --- Sector {i + 1} ---")
            print(f"\t[radar] Start Angle: {round(sector.start_angle, 3)} rad")
            print(f"\t[radar] End Angle: {round(sector.end_angle, 3)} rad")
            print(f"\t[radar] Has Point: {sector.has_point}")
            print(f"\t[radar] Min Distance: {sector.min_distance}")
            print(f"\t[radar] Point: ({round(sector.point.x, 3)}, {round(sector.point.y, 3)}, {round(sector.point.z, 3)})")

            if sector.has_point:

                # For Intensity-Based Color Coding
                dist_norm = min(1.0, sector.min_distance / max_radar_range)  # Normalize

                # Closest point sphere
                marker_point = Marker()
                marker_point.header.frame_id = "base_footprint"
                marker_point.header.stamp = self.get_clock().now().to_msg()
                marker_point.ns = "Radar_sector_closest_point"
                marker_point.id = i+1
                marker_point.type = Marker.SPHERE
                marker_point.action = Marker.ADD
                marker_point.pose.position.x = sector.point.x
                marker_point.pose.position.y = sector.point.y
                marker_point.pose.position.z = sector.point.z
                marker_point.pose.orientation.w = 1.0
                marker_point.scale.x = object_size
                marker_point.scale.y = object_size
                marker_point.scale.z = object_size
                marker_point.color.r = 0.0
                marker_point.color.g = 0.0
                marker_point.color.b = 1.0
                marker_point.color.a = 0.8
                marker_point.lifetime.sec = 0
                marker_point.lifetime.nanosec = 0
                marker_point.frame_locked = False
                marker_array.markers.append(marker_point)

                marker_floor_line = Marker()
                marker_floor_line.header.frame_id = "base_footprint"
                marker_floor_line.header.stamp = self.get_clock().now().to_msg()
                marker_floor_line.ns = "Radar_sector_floor_line"
                marker_floor_line.id = i + 1  # Use same ID as sector
                marker_floor_line.type = Marker.LINE_STRIP
                marker_floor_line.action = Marker.ADD
                marker_floor_line.pose.orientation.w = 1.0
                marker_floor_line.scale.x = 0.02  # Line thickness
                marker_floor_line.color.r = 1.0 - dist_norm
                marker_floor_line.color.g = dist_norm
                marker_floor_line.color.b = 0.0
                marker_floor_line.color.a = 1.0
                marker_floor_line.lifetime.sec = 0
                marker_floor_line.lifetime.nanosec = 0
                marker_floor_line.frame_locked = False

                # Build radar arc
                arc_segments = 24
                min_z = 0.0
                max_z = 0.5
                radius = sector.min_distance
                start_angle = sector.start_angle
                end_angle = sector.end_angle

                # Arc along floor (inner radius, z=0)
                for j in range(arc_segments + 1):
                    theta = start_angle + (end_angle - start_angle) * (j / arc_segments)
                    p = Point()
                    p.x = radius * math.cos(theta)
                    p.y = radius * math.sin(theta)
                    p.z = min_z
                    marker_floor_line.points.append(p)

                marker_array.markers.append(marker_floor_line)

                marker_arc = Marker()
                marker_arc.header.frame_id = "base_footprint"
                marker_arc.header.stamp = self.get_clock().now().to_msg()
                marker_arc.ns = "Radar_sector_arc"
                marker_arc.id = i+1
                marker_arc.type = Marker.TRIANGLE_LIST
                marker_arc.action = Marker.ADD
                marker_arc.pose.orientation.w = 1.0
                marker_arc.scale.x = 1.0  # Required for TRIANGLE_LIST
                marker_arc.scale.y = 1.0
                marker_arc.scale.z = 1.0
                marker_arc.color.r = 1.0 - dist_norm
                marker_arc.color.g = dist_norm
                marker_arc.color.b = 0.0
                marker_arc.color.a = 0.75
                marker_arc.lifetime.sec = 0
                marker_arc.lifetime.nanosec = 0
                marker_arc.frame_locked = False

                for j in range(arc_segments):
                    theta0 = start_angle + (end_angle - start_angle) * (j / arc_segments)
                    theta1 = start_angle + (end_angle - start_angle) * ((j + 1) / arc_segments)

                    # Bottom arc
                    b0 = Point(x=radius * math.cos(theta0), y=radius * math.sin(theta0), z=min_z)
                    b1 = Point(x=radius * math.cos(theta1), y=radius * math.sin(theta1), z=min_z)

                    # Top arc
                    t0 = Point(x=radius * math.cos(theta0), y=radius * math.sin(theta0), z=max_z)
                    t1 = Point(x=radius * math.cos(theta1), y=radius * math.sin(theta1), z=max_z)

                    # Two triangles per segment
                    marker_arc.points.extend([b0, b1, t0])
                    marker_arc.points.extend([t0, b1, t1])

                    # Duplicate in reverse (for back face)
                    marker_arc.points.extend([t0, b1, b0])
                    marker_arc.points.extend([t1, b1, t0])

                marker_array.markers.append(marker_arc)

        self.publisher_marker_array_radar.publish(marker_array)
        self.aux_time = curr_time


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

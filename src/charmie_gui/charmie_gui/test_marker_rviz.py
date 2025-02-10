import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from charmie_interfaces.msg import DetectedPerson, DetectedObject, ListOfDetectedPerson, ListOfDetectedObject
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

        self.publisher_marker_array_detected_person =   self.create_publisher(MarkerArray, "visualization_marker_array_detected_person", 10)

        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)

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
        self.previous_marker_array_detected_people = ListOfDetectedPerson() 

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

                print(person.index, person.position_absolute.x, person.position_absolute.y, person.position_absolute.z, person.height)
                
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


    """
    def publish_marker_array_detected_person(self):

        marker_array = MarkerArray()
        # marker_array_names = MarkerArray()

        # x = -1.0
        # y = 0.0
        # height = 1.80

        person_size = 0.4
        head_size = 0.3

        list_of_detected_person_indexes = []

        for person in self.detected_people.persons:
            if person.index > 0:
                list_of_detected_person_indexes.append(person.index)
                
        print("atuais:", list_of_detected_person_indexes)
        
        # I have to dynamically remove all previous persons that are no longer on the image otherwise rviz will always show them
        for person in self.previous_marker_array_detected_people.persons:
            if person.index > 0:
                print("Comparing:", person.index)
                if person.index not in list_of_detected_person_indexes:

                    print("REMOVED:", person.index)
                    
                    delete_marker = Marker()
                    delete_marker.header.frame_id = "map"
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "Detected_people_B"
                    delete_marker.id = person.index  # Use the same ID to delete it
                    delete_marker.action = Marker.DELETE  # REMOVE from RViz
                    marker_array.markers.append(delete_marker)

                    delete_marker = Marker()
                    delete_marker.header.frame_id = "map"
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "Detected_people_H"
                    delete_marker.id = person.index  # Use the same ID to delete it
                    delete_marker.action = Marker.DELETE  # REMOVE from RViz
                    marker_array.markers.append(delete_marker)

                    delete_marker = Marker()
                    delete_marker.header.frame_id = "map"
                    delete_marker.header.stamp = self.get_clock().now().to_msg()
                    delete_marker.ns = "Detected_people_N"
                    delete_marker.id = person.index  # Use the same ID to delete it
                    delete_marker.action = Marker.DELETE  # REMOVE from RViz
                    marker_array.markers.append(delete_marker)

        ### FALTA:
        # ORIENTATION

        for person in self.detected_people.persons:
            if person.index > 0:

                print(person.index, person.position_absolute.x, person.position_absolute.y, person.position_absolute.z, person.height)
                
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
    
        self.previous_marker_array_detected_people = self.detected_people

    """

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

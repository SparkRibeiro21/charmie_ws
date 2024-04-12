#!/usr/bin/env python3

# initial instructions:
# this is an example of the layout code for a task in our workspace
# It is used a threading system so the ROS2 functionalities are not blocked when executing the state machine
# So we create a thread for the main state machine of the task and another for the ROS2 funcitonalities
# It is being used GPSR as an example, so when changing the code for a specific task Ctrl+F gpsr
# and replace everything fot the name of your task 

import rclpy
from rclpy.node import Node

import threading

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from charmie_interfaces.msg import SpeechType, RobotSpeech
from charmie_interfaces.srv import SpeechCommand

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class GpsrNode(Node):

    def __init__(self):
        super().__init__("Gpsr")
        self.get_logger().info("Initialised CHARMIE GPSR Node")

        ### Topics (Publisher and Subscribers) ###   
        # Low Level 
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)    
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
    
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        while not self.speech_command_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.flag_navigation_done = False

        # Success and Message confirmations for all set_(something) CHARMIE functions
        self.speech_success = True
        self.speech_message = ""

    # Function to send commands to speaker module 
    def call_speech_command_server(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        request = SpeechCommand.Request()
        request.filename = filename
        request.command = command
        request.quick_voice = quick_voice
    
        future = self.speech_command_client.call_async(request)
        print("Sent Command")

        if wait_for_end_of:
            # future.add_done_callback(partial(self.callback_call_speech_command, a=filename, b=command))
            future.add_done_callback(self.callback_call_speech_command)
        else:
            self.speech_success = True
            self.speech_message = "Wait for answer not needed"

    # Function that wait for a response from the service (int his case: wait for speech being said to be over)
    def callback_call_speech_command(self, future): #, a, b):

        try:
            # in this function the order of the line of codes matter
            # it seems that when using future variables, it creates some type of threading system
            # if the flag raised is here is before the prints, it gets mixed with the main thread code prints
            response = future.result()
            self.get_logger().info(str(response.success) + " - " + str(response.message))
            self.speech_success = response.success
            self.speech_message = response.message
            self.waited_for_end_of_speaking = True
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    # Subscriber to get the start button status     
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

# main function that already creates the thread for the task state machine
def main(args=None):
    rclpy.init(args=args)
    node = GpsrNode()
    th_main = threading.Thread(target=thread_main_gpsr, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()

def thread_main_gpsr(node: GpsrNode):
    main = GpsrMain(node)
    main.main()

class GpsrMain():

    def __init__(self, node: GpsrNode):
        # create a node instance so all variables ros related can be acessed
        self.node = node
        
        # create the necessary variables for the state machine
        self.rgb_mode = Int16()

    # wait_for_end_of functions ... 
    # functions where the state machine must wait or not (depending on the wait_for_end_of) for a command 
    # whether this is a variable or just finish a sub-task (speaking, hearing, navigating ...)
    def set_speech(self, filename="", command="", quick_voice=False, wait_for_end_of=True):
        
        self.node.call_speech_command_server(filename=filename, command=command, wait_for_end_of=wait_for_end_of, quick_voice=quick_voice)
        
        if wait_for_end_of:
          while not self.node.waited_for_end_of_speaking:
            pass
        self.node.waited_for_end_of_speaking = False

        return self.node.speech_success, self.node.speech_message
    

    # just an example of a wait for end of navigation
    def wait_for_end_of_navigation(self):
        self.node.flag_navigation_done = False
        while not self.node.flag_navigation_done:
            pass
        self.node.flag_navigation_done = False
        print("Finished Navigation")

    # main state-machine function
    def main(self):
        # examples of names of states
        # use the names of states rather than just numbers to ease 3rd person code analysis
        Waiting_for_start_button = 0
        Receive_guest = 1
        Characteristics_first_guest = 2
        Navigation_to_sofa = 3
        Presentation_host_first_guest = 4
        Presentation_host_first_second_guest = 5
        Final_State = 6

        self.state = Waiting_for_start_button

        # debug print
        print("IN NEW MAIN")

        while True:

            if self.state == Waiting_for_start_button:
                print('State 0 = Initial')

                # your code here ...
                #NECK: LOOKS IN FRONT
                #SPEAK: Hello! I am ready to start the receptionist task! Waiting for start button to be pressed.
                #RGB: WAITING MODE
                #IF BUTTON PRESSED:
                # RGB: OK MODE
                # NECK: LOOKS TO THE FLOOR (NAVIGATION POSE)
                # MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                #NEXT STATE 
                    
                # next state
                self.state = Receive_guest

            elif self.state == Receive_guest:
                print('State 1 = Receive guest')

                # your code here ...
                #NECK: LOOKS IN FRONT 
                #RGB: WAITING MODE
                #SPEAK: I am ready to receive a new guest. Please stand in front of me.
                #if face = exist:
                # RGB: OK MODE (GREEN)
                # NECK: LOOKS TO GUEST 1
                # SPEAK: Hello! My name is Charmie. I will make you some questions. Please speak loud and clear. Answer me after the green light on my face.
                # SPEAK:What's your name and favourite drink?
                # AUDIO: RECEIVE NAME AND DRINK OF GUEST 
                # RGB: OK MODE
                # CAMARA: SAVE IMAGE FROM GUEST 1
                # if numero do guest = 1:
                #  self.state = Characteristics_first_guest
                # elif numero do guest = 2:
                #  self.state = Navigation_to_sofa  
                #else:  
                # continue searching

            elif self.state == Characteristics_first_guest:
                print('State 2 = Characteristics first guest')
                
                #SAVE IMAGE FROM GUEST WITH NAME: "GUEST1"
                #SPEAK CHARACTERISTICS: AGE, RACE, GENDER, HEIGHT, SHIRT COLOR, PANTS COLORS
                #CALL FUNCTION - PROCESS_CHARACTERISTICS - PROCESSING INFORMATION AS IN 2024 CODE
                #similar like this:
                #get_caract, characteristics, none_variables = analyze_variables(race, age, gender, height, shirt_color, pant_color)


                self.state = Navigation_to_sofa
                
            elif self.state == Navigation_to_sofa:
                print('State 3 = Navigation to sofa')
                #RGB: OK MODE
                #SPEAK:Thank you. Please follow me.
                #NECK: LOOKS TO THE FLOOR (NAVIGATION POSE)
                #MOVE TO SOFA LOCALISATION
                #SPEAK:Please stay on my left until I give you instructions on where to sit.
                #if numero do guest = 1:
                #self.state = Presentation_host_first_guest
                #elif numero do guest = 2:
                #self.state = Presentation_host_first_second_guest 

            elif self.state == Presentation_host_first_guest:
                print('State 4 = Presentation host and first guest')
                #NECK: LOOK TO THE SOFA
                #ACTION: FOUND PERSON (HOST)
                #NECK: LOOK TO HOST
                #SPEAK:Hello, I will present everyone in this room.
                #NECK: LOOK TO THE GUEST
                #SPEAK:The host is "NAME" and his favorite drink is "DRINK".
                #NECK: LOOK TO HOST 
                #SPEAK:The first guest name is "NAME" and the favorite drink is "DRINK"
                #ACTION: FOUND AN EMPTY SEAT
                #NECK: LOOK TO AN EMPTY SEAT
                #SPEAK:Please take a sit on the sofa that I'm looking at.
                #RGB: OK MODE
                #NECK: LOOKS TO THE FLOOR (NAVIGATION POSE)
                #MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                self.state = Receive_guest
                                
            elif self.state == Presentation_host_first_second_guest:
                print('State 4 = Presentation host, first and second guest')
                #NECK: LOOK TO THE GUEST
                #SPEAK:The host is "NAME" and his favorite drink is "DRINK".
                #SPEAK:Introduce the first guest - name and drink and characteristics
                #NECK: LOOK TO THE SOFA/CHAIRS
                #ACTION: FOUND PERSONS/LOCALISATIONS
                #ACTION: RECOGNIZE HOST AND GUEST 1 AND ASSOCIATE THEM COORDINATES
                #NECK: LOOK TO HOST
                #SPEAK:Hello, I will present everyone in this room.
                #SPEAK:Dear host
                #NECK: LOOK TO THE GUEST
                #SPEAK:Dear guest
                #SPEAK:The second guest name is "NAME" and the favorite drink is "DRINK"
                #ACTION: FOUND AN EMPTY SEAT
                #NECK: LOOK TO AN EMPTY SEAT
                #SPEAK:Please take a sit on the sofa that I'm looking at.
                #RGB: OK MODE
                #NECK: LOOKS TO THE FLOOR (NAVIGATION POSE)
                #MOVE TO DOOR LOCALISATION (PLACE TO RECEIVE THE GUEST)
                self.state = Final_State

            elif self.state == Final_State:
                
                print("Finished task!!!")
                #NECK: LOOK IN FRONT
                #SPEAK: Thank you. I finished my receptionist task
                #NECK: LOOK TO THE FLOOR
                # After finishing the task stays in this loop 
                while True:
                    pass

            else:
                pass

    '''def Get_characteristics(race, age, gender, height, shirt_color, pant_color):
        characteristics = []
        none_variables = []

        if race is not None:
            characteristics.append(race)
        else:
            none_variables.append("race")

        if age is not None:
            characteristics.append(age)
        else:
            none_variables.append("age")

        if gender is not None:
            characteristics.append(gender)
        else:
            none_variables.append("gender")

        if height is not None:
            characteristics.append(height)
        else:
            none_variables.append("height")

        if shirt_color is not None:
            characteristics.append(shirt_color)
        else:
            none_variables.append("shirt_color")

        if pant_color is not None:
            characteristics.append(pant_color)
        else:
            none_variables.append("pant_color")

        get_caract = len(characteristics)

        return get_caract, characteristics, none_variables


    def Process_Info(get_caract, characteristics, none_variables):
        if get_caract == 5 or get_caract == 6:
            print("Características:", characteristics)
        elif get_caract == 4 or get_caract == 3 or get_caract == 2 or get_caract == 1 or get_caract == 0:
            print(characteristics)
            if 'age' in none_variables:
                print('age 25 and 32')
            if 'gender' in none_variables:
                print('gender male')
            if 'race' in none_variables:
                print('race caucasian')
            if 'height' in none_variables:
                print('height taller than me')
            if 'shirt_color' in none_variables:
                print('white')
            if 'pant_color' in none_variables:
                pass
            if not none_variables:  # Se não houver variáveis ausentes
                print("nada")



    def main():
        # Variáveis de exemplo para teste
        race = "Caucasian"
        age = 18
        gender = None
        height = None
        shirt_color = "Blue"
        pant_color = None

        # Chamando a função Get_characteristics
        get_caract, characteristics, none_variables = Get_characteristics(race, age, gender, height, shirt_color, pant_color)

        # Mostrando os resultados
        print("Número de características:", get_caract)
        print("Características:", characteristics)
        print("Variáveis None:", none_variables)

        # Processando as informações
        Process_Info(get_caract, characteristics, none_variables)


    if __name__ == "__main__":
        main()'''
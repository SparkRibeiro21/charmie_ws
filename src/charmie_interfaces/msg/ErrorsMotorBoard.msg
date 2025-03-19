# List of Possible Errors from Motor Board (MD49)
# GET ERROR
# In the event of and error the red LED will be lit and a byte which contains
# the type of error that the MD49 is experiencing will have the appropriate bit set. 
# The bits are:

# bit 7     bit 6	    bit 5	        bit 4	        bit 3	        bit 2	        bit 1	bit 0
# UNDER 16V OVER 30V	MOTOR 2 SHORT	MOTOR 1 SHORT	MOTOR 2 TRIP	MOTOR 1 TRIP	-	    -

# when the emergency stop is pressed it initially says the robot is undervoltage and half a second later changes to overvoltage
# it is something that comes from the board... it is not a real error
bool undervoltage_error
bool overvoltage_error

bool undervoltage_b1_error
bool overvoltage_b1_error
bool undervoltage_b2_error
bool overvoltage_b2_error

bool motor2_b1_short_error
bool motor1_b1_short_error
bool motor2_b1_trip_error
bool motor1_b1_trip_error

bool motor2_b2_short_error
bool motor1_b2_short_error
bool motor2_b2_trip_error
bool motor1_b2_trip_error
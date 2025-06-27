#  Message to get the data from all the buttons and axis from a /joy gamepad controller to custom buttons and axes

float32[10] axes  # thumbsticks, triggers, ...
bool[18] buttons # a, b, x, y, bumpers, menu, ...
bool timeout     # true if the controller is not responding

# axes[14]:
# Index in List | Xbox Controller | PS4 Controller
# [0]             LTB_X             L3_X
# [1]             LTB_Y             L3_Y
# [2]             LTB_Ang           L3_Ang
# [3]             LTB_Dist          L3_Dist
# [4]             RTB_X             R3_X
# [5]             RTB_Y             R3_Y
# [6]             RTB_Ang           R3_Ang
# [7]             RTB_Dist          R3_Dist
# [8]             LT                L2
# [9]             RT                R2

# buttons[18]:
# Index in List | Xbox Controller | PS4 Controller
# [0]             A                 Cross
# [1]             B                 Circle
# [2]             X                 Square
# [3]             Y                 Triangle
# [4]             LB                L1
# [5]             RB                R1
# [6]             View              Share
# [7]             Menu              Options
# [8]             Logo              PS Button
# [9]             Share             -
# [10]            LTB               L3
# [11]            RTB               R3
# [12]            DPad Up           Arrow Up
# [13]            DPad Down         Arrow Down
# [14]            DPad Left         Arrow Left
# [15]            DPad Right        Arrow Right
# [16]            LT                L2
# [17]            RT                R2

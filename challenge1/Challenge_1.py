import math
from comm.Serial import SerialController, DataType_Int, DataType_Float, DataType_Boolean
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI
from user_parameters import ROBOT_MAC, SERIAL_PORT, PRINT_JOYSTICK

import time







BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address



if __name__ == "__main__":
    # Communication
    serial = SerialController(SERIAL_PORT, timeout=.5)  # .5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    time.sleep(.05)
    # // PID terms
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpyaw", 0.08) #0.05
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdyaw", -0.03)#.001
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiyaw", 0)

    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", 0.5) #0.01
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", 0.9) #0.006
    serial.send_preference(ROBOT_MAC, DataType_Float, "kiz", 0) #0.001

    # // Range terms for the integral
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_low", 0)#0.05
    serial.send_preference(ROBOT_MAC, DataType_Float, "z_int_high", 0) #0.15



    
    # Allows the robot to read the parameters from flash memory to be used.
    serial.send_control_params(ROBOT_MAC, (0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 1, 0))

    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()
    mygui = SimpleGUI()
    
    ready = 1
    old_b = 0
    old_x = 0
    dt = .1
    height = 0
    servos = 75
    tz = 0
    desired_height = 0.0
    yaw_rate_threshold = 1.0
    hight_threshold = 10
    direction_input = 0
    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()
            
            if buttons[3] == 1: # y stops the program
                break

            # b button is a toggle which changes the ready state
            if buttons[1] == 1 and old_b == 0: # b pauses the control
                if ready != 0:
                    desired_height = 0
                    ready = 0
                else:
                    ready = 1
            old_b = buttons[1]


            if PRINT_JOYSTICK:
                print("Joystick: ", ["{:.1f}".format(num) for num in axis], "Buttons: ", buttons)
            
            if(-axis[0] > 0.6 or -axis[0] < -0.6):
                desired_height += -axis[0]
                if(desired_height > hight_threshold):
                    desired_height = hight_threshold
                elif(desired_height <= 0):
                    desired_height = 0
            
            #### CONTROL INPUTS to the robot here #########
            fx = ((axis[5]+1) / 2 - (axis[2]+1) / 2) * 0.9 # Forward with the triggers
            fz = desired_height  # Vertical left joystick
            tx = 0  # No roll control
            
            tz = -axis[4] * math.pi # Horizontal right joystick
            led = -buttons[2]  # Button X

            print("fx={:.2f} fz={:.2f} tz={:.2f} LED={:.2f} ".format(fx, fz, tz, led), end=' ')
            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()
            #
            if sensors:
                print("Sensors:", ["{:.2f}".format(val) for val in sensors])
                mygui.update(
                    cur_yaw=sensors[1],
                    des_yaw=tz,
                    cur_height=sensors[0],
                    des_height=desired_height,
                    battery=sensors[2],
                    distance=sensors[3],
                    connection_status=True,
                )
            else:
                print("No sensors")
            
            tz = mygui.des_yaw


            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (ready, fx, fz, tx, tz, led, yaw_rate_threshold, 0, 0, 0, 0, 0, 0))
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("Stopping!")
        # Send zero input
serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))



from comm.Serial import SerialController, DataType_Int, DataType_Float
from joystick.JoystickManager import JoystickManager
from gui.simpleGUI import SimpleGUI
import time

##### Insert your robot's MAC ADDRESS here ####
## (you can get it by running your arduino and looking at the serial monitor for your flying drone) ##
ROBOT_MAC = "34:85:18:91:38:60" # "DC:54:75:D7:B3:E8"
### Insert your SERIAL PORT here ###
## may look like "COM5" in windows or "/dev/tty.usbmodem14301" in mac  #
## look in arduino for the port that your specific transeiver is connected to  ##
## Note: make sure that your serial monitor is OFF in arduino or else you will get "access is denied" error. ##
PORT = "COM6"


# For debug purposes
PRINT_JOYSTICK = False


BaseStationAddress = "" # you do not need this, just make sure your DroneMacAddress is not your base station mac address

def joystick_to_servo(input_list):
    # Assuming input_list[0] is the axis we're interested in
    joystick_value = input_list
    
    # Map from [-1.0, 1.0] to [0, 180]
    # First, normalize to [0, 1] then scale to [0, 180]
    servo_value = ((joystick_value + 1) / 2) * 180
    if servo_value > 90:
        servo_value = 75
    elif servo_value < 90:
        servo_value = 105
    else:
        servo_value = 90
    return servo_value

def map_joystick_to_thrust(joystick_input):
    # Ensure the joystick input is within the expected range
    assert -1 <= joystick_input <= 1, "Joystick input must be between -1 and 1"

    # Map the joystick input from [-1, 1] to [0, 1] for motor thrust
    motor_thrust = (joystick_input + 1) / 2
    if motor_thrust == 0:
        motor_thrust = 0.5

    return motor_thrust

def map_joystick_to_height(joystick_input):
    # Ensure the joystick input is within the expected range
    assert -1 <= joystick_input <= 1, "Joystick input must be between -1 and 1"

    # Map the joystick input from [-1, 1] to [0, 1] for motor thrust
    if(joystick_input > 0):
        return 10
    return 0

if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)
    serial.manage_peer("G", ROBOT_MAC)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kpz", .6)
    serial.send_preference(ROBOT_MAC, DataType_Float, "kdz", .8)
    serial.send_control_params(ROBOT_MAC, (0,0,90,90, 0, 0, 0, 0, 0, 0, 0, 1, 0)) #refresh parameters
    time.sleep(.2)

    # Joystick
    joystick = JoystickManager()
    mygui = SimpleGUI()
    is_motor_on = False

    try:
        while True:
            # Axis input: [left_vert, left_horz, right_vert, right_horz, left_trigger, right_trigger]
            # Button inputs: [A, B, X, Y]
            axis, buttons = joystick.getJoystickInputs()


            if PRINT_JOYSTICK:
                print(" ".join(["{:.1f}".format(num) for num in axis]), buttons)

            #### CONTROL INPUTS to the robot here #########
            m1 = 0.  # Motor 1: a value between 0-1
            m2 = 0.  # Motor 2: a value between 0-1
            s1 = 0.  # Servo 1: a value between 0-180
            s2 = 0.  # Servo 2: a value between 0-180
            led = axis[0]
            height = 0
            angle = 35
            ############# End CONTROL INPUTS ###############
            sensors = serial.getSensorData()
            #print(sensors)
            if (sensors):
                mygui.update(
                    cur_yaw=0,
                    des_yaw=-0,
                    cur_height=sensors[0],
                    des_height=0,
                    battery=0,
                    distance=0,
                    connection_status=True,
                )
            
            """if(buttons[0] == 1):
                is_motor_on = not is_motor_on
            if is_motor_on:
                #m1 = map_joystick_to_thrust(axis[5])
                #m2 = map_joystick_to_thrust(axis[5])
                height = map_joystick_to_height(axis[5])"""
            height = map_joystick_to_height(axis[5])
            if axis[4] > 0.8 or axis[4] < -0.8:
                s1 = joystick_to_servo(axis[3])
                s2 = s1
            else:
                if(axis[0] > 0.8):
                   s1 = angle
                   s2 = 180-s1
                elif(axis[0] < -0.8):
                    s2 = angle
                    s1 = 180-s2
                else:
                    s1 = 90
                    s2 = 90
            print(s1,s2)
            serial.send_control_params(ROBOT_MAC, (m1, m2, s1, s2, led, height, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(.3)
            
    except KeyboardInterrupt:
        print("Stopping!")
    # Send zero input
    serial.send_control_params(ROBOT_MAC, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    serial.close()

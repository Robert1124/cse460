

from comm.Serial import SerialController
from joystick.JoystickManager import JoystickManager
import time


def joystick_to_servo(input_list):
    # Assuming input_list[0] is the axis we're interested in
    joystick_value = input_list
    
    # Map from [-1.0, 1.0] to [0, 180]
    # First, normalize to [0, 1] then scale to [0, 180]
    servo_value = ((joystick_value + 1) / 2) * 180
    
    return servo_value

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



if __name__ == "__main__":
    # Communication
    serial = SerialController(PORT, timeout=.1)  # 5-second timeout
    serial.manage_peer("A", ROBOT_MAC)

    # Joystick
    joystick = JoystickManager()

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
            #print(buttons)
            


            # Map joystick input to servo values
            s1 = joystick_to_servo(axis[0])
            s2 = joystick_to_servo(axis[3])

            if(buttons[0] == 1):
                is_motor_on = not is_motor_on

            if is_motor_on:
                m1 = 0.6
                m2 = 0.6
            
            ############# End CONTROL INPUTS ###############

            # Send through serial port
            serial.send_control_params(ROBOT_MAC, (m1, m2, s1, s2, led, 0, 0, 0, 0, 0, 0, 0, 0))
            time.sleep(.05)
            
    except KeyboardInterrupt:
        print("Stopping!")
        # Send zero input
        serial.send_control_params(ROBOT_MAC, (0, 0, 180, 180, 0, 0, 0, 0, 0, 0, 0, 0, 0))



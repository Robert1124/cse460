# Untitled - By: WUzy2 - Wed May 8 2024

# Untitled - By: WUzy2 - Fri Apr 26 2024

from pyb import UART, LED

import sensor
import time

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.

r_LED = LED(1)  # The red LED
g_LED = LED(2)  # The green LED
b_LED = LED(3)  # The blue LED
g_LED.on()
r_LED.off()
b_LED.off()
clock = time.clock()  # Create a clock object to track the FPS.
uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

def checksum(arr, initial= 0):
    """ The last pair of byte is the checksum on iBus
    """
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB

def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]

    # Perform the checksume
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB

    uart.write(msg)

def refreshIbusConnection():
    if uart.any():
        uart_input = uart.read()


target =(0, 100, 21, 127, -23, 127)

while True:
    ############### Color detection here ###############

    #put color detection and such here

    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    #print(clock.fps())  # Note: OpenMV Cam runs about half as fast when connected
    # to the IDE. The FPS should increase once disconnected.
    blobs = img.find_blobs([target], area_threshold=1200, merge=True)

    if blobs:
        color_is_detected = True
        # Select the largest blob based on the area
        largest_blob = max(blobs, key=lambda b: b.pixels())
        pixels_x = largest_blob.cx()  # X center of the largest blob
        pixels_y = largest_blob.cy()  # Y center of the largest blob
        pixels_w = largest_blob.w()   # Width of the largest blob
        pixels_h = largest_blob.h()   # Height of the largest blob
        # Draw a rectangle where the blob was found
        img.draw_rectangle(largest_blob.rect(), color=(0,255,0))
        # Draw a cross in the middle of the blob
        img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=(0,255,0))
        g_LED.off()
        r_LED.off()
        b_LED.on()
        flag = 1  # Set the flag to 1 when a color is detected
    else:
        color_is_detected = False
        g_LED.on()
        r_LED.off()
        b_LED.off()
        pixels_x = 0
        pixels_y = 0
        pixels_w = 0
        pixels_h = 0
        flag = 0

    ###############################
    #print(f"Center: ({pixels_x}, {pixels_y}), Width: {pixels_w}, Height: {pixels_h}")


    messageToSend = [flag, pixels_x, pixels_y, pixels_w, pixels_h]

    IBus_message(messageToSend)
    refreshIbusConnection()

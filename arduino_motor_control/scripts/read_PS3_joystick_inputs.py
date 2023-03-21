#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame
import rospy
from arduino_motor_control.msg import Motors

"""
BUTTONS MAP FOR PS3 CONTROLLER:
button0 = X
button1 = circle
button2 = triangle
button3 = square
button4 = L1
button5 = R1
button6 = L2
button7 = R2
button8 = select
button9 = start
button10 = PS command
button11 = press left stick
button12 = press right stick
button13 = up arrow
button14 = down arrrow
button15 = left arrow
button16 = right arrow

axis0 = left stick left (-1) right (0.99)
axis1 = left stick up (-1) down (0.99)
axis2 -> stuck at -1(???)
axis3 = right stick left (-1) right (0.99)
"""

def remap_left_stick(analog_value):
    """
    The function takes an input ranging from -1 to 0.99
    Remaps it between -255 and 255
    Converted values between -20 and 20 are set to 0
    """
    result = int(510/2*analog_value + 255 - 510/2)
    if result < 20 and result > -20:
        result = 0
    return result

def block_pygame_events():
    """
    Stops the specified events from occupying pygame events queue
    """
    pygame.event.set_blocked(pygame.ACTIVEEVENT)
    pygame.event.set_blocked(pygame.KEYDOWN)
    pygame.event.set_blocked(pygame.KEYUP)
    pygame.event.set_blocked(pygame.MOUSEBUTTONDOWN)
    pygame.event.set_blocked(pygame.MOUSEBUTTONUP)
    pygame.event.set_blocked(pygame.MOUSEMOTION)
    pygame.event.set_blocked(pygame.JOYHATMOTION)
    pygame.event.set_blocked(pygame.JOYBALLMOTION)
    pygame.event.set_blocked(pygame.VIDEOEXPOSE)
    pygame.event.set_blocked(pygame.VIDEORESIZE)
    pygame.event.set_blocked(pygame.USEREVENT)
    pygame.event.set_blocked(pygame.AUDIODEVICEADDED)
    pygame.event.set_blocked(pygame.AUDIODEVICEREMOVED)
    pygame.event.set_blocked(pygame.FINGERDOWN)
    pygame.event.set_blocked(pygame.FINGERMOTION)
    pygame.event.set_blocked(pygame.FINGERUP)
    pygame.event.set_blocked(pygame.MOUSEWHEEL)
    pygame.event.set_blocked(pygame.MULTIGESTURE)
    pygame.event.set_blocked(pygame.TEXTEDITING)
    pygame.event.set_blocked(pygame.TEXTINPUT)
    pygame.event.set_blocked(pygame.WINDOWSHOWN)
    pygame.event.set_blocked(pygame.WINDOWCLOSE)
    pygame.event.set_blocked(pygame.WINDOWENTER)
    pygame.event.set_blocked(pygame.WINDOWEXPOSED)
    pygame.event.set_blocked(pygame.WINDOWFOCUSLOST)
    pygame.event.set_blocked(pygame.WINDOWFOCUSGAINED)
    pygame.event.set_blocked(pygame.WINDOWHIDDEN)
    pygame.event.set_blocked(pygame.WINDOWHITTEST)
    pygame.event.set_blocked(pygame.WINDOWLEAVE)
    pygame.event.set_blocked(pygame.WINDOWMAXIMIZED)
    pygame.event.set_blocked(pygame.WINDOWMINIMIZED)
    pygame.event.set_blocked(pygame.WINDOWMOVED)
    pygame.event.set_blocked(pygame.WINDOWRESIZED)
    pygame.event.set_blocked(pygame.WINDOWSIZECHANGED)
    pygame.event.set_blocked(pygame.WINDOWTAKEFOCUS)
    pygame.event.set_blocked(pygame.WINDOWRESTORED)
    pygame.event.set_blocked(pygame.JOYAXISMOTION)

def motors_command_server():
    # Initialize joysticks
    pygame.joystick.init()
    # Only one joystick connected, use it
    ps3 = pygame.joystick.Joystick(0)

    # Initialize pygame screen to read joystick events
    WINDOW = pygame.display.set_mode((600,600))

    # Initialize the node
    rospy.init_node('run_DC_servo_motors', anonymous=True)

    # Create a publisher on "motor_commands" topic
    motor_pub = rospy.Publisher('motor_commands', Motors, queue_size=10)

    # Initialize a "Motors" object 
    motors_command = Motors()

    # Set task rate
    rate = rospy.Rate(10) # Execute next while loop with 1Hz rate

    # Block useless events
    block_pygame_events()

    while True:
        # Tell servo motor it is going to receive an incremental type of command
        # Decrease servo angular position of 0°
        # (keep servo motor at its current position, going to be updated if any button is pressed)
        motors_command.servo_type = 1
        motors_command.servo_command = 0
        # Get event from pygame events queue 
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            pygame.quit()
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0: # X button
                # Tell servo motor it is going to receive an absolute position type of command
                # Send servo to 90° position
                motors_command.servo_type = 0
                motors_command.servo_command = 90
            elif event.button == 1: # Circle button
                # Tell servo motor it is going to receive an absolute position type of command
                # Send servo to 0° position
                motors_command.servo_type = 0
                motors_command.servo_command = 0
            elif event.button == 3: # Square button
                # Tell servo motor it is going to receive an absolute position type of command
                # Send servo to 180° position
                motors_command.servo_type = 0
                motors_command.servo_command = 180
            elif event.button == 4: # L1
                # Tell servo motor it is going to receive an incremental type of command
                # Decrease servo angular position of 10°
                motors_command.servo_type = 1
                motors_command.servo_command = -10
            elif event.button == 5: # R1
                # Tell servo motor it is going to receive an incremental type of command
                # Increase servo angular position of 10°
                motors_command.servo_type = 1
                motors_command.servo_command = 10

        # Read the position of left stick (range -1/0.99)
        left_stick_value = ps3.get_axis(1)
        # Map the left stick value in range -255/+255
        motors_command.DC_motor_command = remap_left_stick(left_stick_value)
        # Publish this value on the topic for the DC motor to read it:
        # The magnitude of the value determines the speed, the sign the direction
        motor_pub.publish(motors_command)
        # Log the message published on "DC_motor" topic also on terminal
        rospy.loginfo("DC motor value is: %s\nServo command type is: %s,\nServo Command is: %s" % (motors_command.DC_motor_command, motors_command.servo_type, motors_command.servo_command))

        rate.sleep() # Wait

if __name__ == "__main__":
    try:
        motors_command_server()
    except rospy.ROSInterruptException:
        pass        

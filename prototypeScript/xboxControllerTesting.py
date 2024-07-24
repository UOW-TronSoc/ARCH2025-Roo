import pygame
from pi5RC import pi5RC
from time import sleep
import numpy as np

# Initialize pygame for  controller input
pygame.init()

# Initialize the joystick
pygame.joystick.init()

# Check for connected joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick connected")
    pygame.quit()
    exit()

# Use the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Define the compensation function based on observed data
def speed_to_pwm(input_value):
    
    # convert [0, 1024] to [-1, 1] assuming 10-bit adc joystick as input value
    # normalisedVal = input_value/512 - 1
    normalisedVal = input_value

    transformedVal = (normalisedVal ** 2) *normalisedVal/abs(normalisedVal+0.0000001) / 1.5

    return int(transformedVal * 1000 + 1500)

def ang2ms(position):
    return 500 + 2000 * position/180

# Init PWM Pins
pwm0 = pi5RC(12)
pwm1 = pi5RC(13)
pwm2 = pi5RC(18)
pwm3 = pi5RC(19)

gymbalYaw = 1500
gymbalPitch = 1500
rooLeftDrive = 1500
rooRightDrive = 1500    

def get_dpad_status(hat):
    if hat == (0, 1):
        return "DPad Up"
    elif hat == (0, -1):
        return "DPad Down"
    elif hat == (-1, 0):
        return "DPad Left"
    elif hat == (1, 0):
        return "DPad Right"
    elif hat == (1, 1):
        return "DPad Up-Right"
    elif hat == (-1, 1):
        return "DPad Up-Left"
    elif hat == (1, -1):
        return "DPad Down-Right"
    elif hat == (-1, -1):
        return "DPad Down-Left"
    else:
        return "DPad Not Pressed"

def processHat(hat):
    global gymbalYaw, gymbalPitch
    if hat == (0, 1):
        gymbalPitch += 50
    elif hat == (0, -1):
        gymbalPitch -= 50
    elif hat == (-1, 0):
        gymbalYaw -= 50
    elif hat == (1, 0):
        gymbalYaw += 50
    elif hat == (1, 1):
        gymbalPitch += 50
        gymbalYaw += 50
    elif hat == (-1, 1):
        gymbalPitch += 50
        gymbalYaw -= 50
    elif hat == (1, -1):
        gymbalPitch -= 50
        gymbalYaw += 50
    elif hat == (-1, -1):
        gymbalPitch -= 50
        gymbalYaw -= 50

try:

    pwm0.set(gymbalYaw)
    pwm1.set(gymbalPitch)
    pwm2.set(rooLeftDrive)
    pwm3.set(rooRightDrive)

    while True:
        for event in pygame.event.get():
            pass  # We only need to pump the event queue

        # Read controller joystick vertical axis
        axis1 = joystick.get_axis(1)
        axis3 = joystick.get_axis(3)

        # Read controller dpad button
        hat = joystick.get_hat(0)

        # Set PWM Value based on joytick
        rooLeftDrive = speed_to_pwm(axis1)
        rooRightDrive = speed_to_pwm(-axis3)

        # Process gymbal based on the dpad/hat input
        processHat(hat)

        # Set PWM on pins
        pwm0.set(gymbalYaw)
        pwm1.set(gymbalPitch)
        pwm2.set(rooLeftDrive)
        pwm3.set(rooRightDrive)

        # dpad_status = get_dpad_status(hat)
        # print(f"Axis 1 value: {axis1:.2f} | Axis 3 value: {axis3:.2f} | {dpad_status}", end='\r')

        sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()

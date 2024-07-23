from pi5RC import pi5RC
from time import sleep
import numpy as np

# Define the compensation function based on observed data
def speed_to_pwm(input_value):
    
    # convert [0, 1024] to [-1, 1] assuming 10-bit adc joystick as input value
    normalisedVal = input_value/512 - 1

    transformedVal = (normalisedVal ** 2) *normalisedVal/abs(normalisedVal+0.0000001)

    return int(transformedVal * 1000 + 1500)

def ang2ms(position):
    return 500 + 2000 * position/180


pwm0 = pi5RC(12)
pwm1 = pi5RC(13)
pwm2 = pi5RC(18)
pwm3 = pi5RC(19)

# set all r/c to the middle
# the set command put timing in microsecond
#
#

# Function to sweep the servo back and forth
def sweep_servo():
    while True:
        for position in range(0, 1024, 1):  # Sweep from -1 to 1
            print(position)
            print(speed_to_pwm(position))
            pwm3.set(speed_to_pwm(position))
            sleep(0.01)

        for position in range(1024, 0, -1):  # Sweep back from 1 to -1
            print(position)
            print(speed_to_pwm(position))
            pwm3.set(speed_to_pwm(position))
            sleep(0.01)

# Example usage
try:
    pwm0.set(500)
    pwm1.set(1000)
    pwm2.set(1500)
    pwm3.set(1500)
    sweep_servo()

except KeyboardInterrupt:
    print("Program stopped")

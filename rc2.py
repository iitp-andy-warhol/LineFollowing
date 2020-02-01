# We need the time library so we can sleep
import time

# First we pick up the servokit library
from adafruit_servokit import ServoKit

import turtle

wn = turtle.Screen()
wn.title("AndyCar")
wn.bgcolor("black")
wn.setup(width=400, height=300)
wn.tracer(0)


# Pen
pen = turtle.Turtle()
pen.speed(0)
pen.color("white")
pen.penup()
pen.hideturtle()
pen.goto(0, 0)
pen.write("AndyCar_2", align="center", font=("Courier", 24, "normal"))


# Now we instantiate a library - note there are 16 channels on the PWM
# hat. We only use channels 0 and 1 for the bot's servos.
kit = ServoKit(channels=16)


# Function
# servo[0] -> left, 1 -> forward
# servo[1] -> right, -1 -> forward
def forward():
    kit.continuous_servo[0].throttle = 0.4
    kit.continuous_servo[1].throttle = -1
    time.sleep(0.2)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0
    


def backward():
    kit.continuous_servo[0].throttle = -0.4
    kit.continuous_servo[1].throttle = 1
    time.sleep(0.2)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0


def left():
    kit.continuous_servo[0].throttle = -1
    kit.continuous_servo[1].throttle = -1
    time.sleep(1.12)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0


def right():
    kit.continuous_servo[0].throttle = 1
    kit.continuous_servo[1].throttle = 1
    time.sleep(1.15)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0
    
    
def forwardleft():
    kit.continuous_servo[0].throttle = 0.13
    kit.continuous_servo[1].throttle = -1
    time.sleep(0.2)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0


def forwardright():
    kit.continuous_servo[0].throttle = 1
    kit.continuous_servo[1].throttle = -0.05
    time.sleep(0.2)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0

def stop():
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0
    time.sleep(0.2)
    kit.continuous_servo[0].throttle = 0
    kit.continuous_servo[1].throttle = 0


# Keyboard binding
wn.listen()
wn.onkeypress(forward, "w")
wn.onkeypress(backward, "s")
wn.onkeypress(left, "a")
wn.onkeypress(right, "d")
wn.onkeypress(forwardleft, "q")
wn.onkeypress(forwardright, "e")
wn.onkeypress(stop, "f")


# Main game loop
while True:
    wn.update()



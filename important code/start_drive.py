"""
Test Program for UART output
"""

import serial
import time
from direct_websocket_control import *

"""
Serial Port Initialization
	baud rate: 9600
	data: 8 bit
	parity: none
	stop bits: 1 stop bit
"""

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
time.sleep(1)

# Print parameters
# print("serial_port.baudrate: ", serial_port.baudrate)
# print("serial_port.bytesize: ", serial_port.bytesize)
# print("serial_port.parity: ", serial_port.parity)
# print("serial_port.stopbits: ", serial_port.stopbits)


ser = serial.Serial(port="/dev/ttyTHS1", baudrate=9600, timeout=1)


# Get the base accelerometer values so they can be canceled out
# For example, the car doesn't need to know the acceleration due to gravity
def getAccel():
    x_axis = 0
    y_axis = 0
    z_axis = 0

    message = ser.read(6)
    if message:
        msg_hex = message.hex()
        # print(msg_hex, end='\n\r', flush=True)
        x_axis_str = msg_hex[2] + msg_hex[3] + msg_hex[0] + msg_hex[1]
        x_axis = int(x_axis_str, 16) - (int(x_axis_str, 16) >> 15 << 16)
        # print("X_axis: ", x_axis * 8, end='\n\r', flush=True)

        y_axis_str = msg_hex[6] + msg_hex[7] + msg_hex[4] + msg_hex[5]
        y_axis = int(y_axis_str, 16) - (int(y_axis_str, 16) >> 15 << 16)
        # print("Y_axis: ", y_axis * 8, end='\n\r', flush=True)

        z_axis_str = msg_hex[10] + msg_hex[11] + msg_hex[8] + msg_hex[9]
        z_axis = int(z_axis_str, 16) - (int(z_axis_str, 16) >> 15 << 16)
        # print("Z_axis: ", z_axis * 8, end='\n\r', flush=True)

    return x_axis, y_axis, z_axis


def getCarToMove(base_x_axis, base_y_axis, base_z_axis, initSpeed, stopAtEnd):

    speed = initSpeed
    steer = 90

    setSpeed(speed)
    setRot(steer)

    x_axis = base_x_axis
    y_axis = base_y_axis
    z_axis = base_z_axis

    prevTime = time.time() * 1000  # Gets time in milliseconds

    accelSame = True

    # While the accelerometer data is the same as the set up accelerometer
    while accelSame:
        # Increase speed by 1
        speed += 1
        print(speed)
        setSpeed(speed)

        # Read in the new accelerometer data
        x_axis, y_axis, z_axis = getAccel()

        curTime = time.time() * 1000  # Gets time in milliseconds

        x_same = abs(x_axis - base_x_axis) < 500
        y_same = abs(y_axis - base_y_axis) < 500
        z_same = abs(z_axis - base_z_axis) < 500

        print(x_same, y_same, z_same)

        accelSame = x_same and y_same and z_same
        while (curTime - prevTime < 500) and accelSame:
            x_axis, y_axis, z_axis = getAccel()

            # print("prevTime: " , prevTime)
            # print("curTime: ", curTime)

            curTime = time.time() * 1000  # Gets time in milliseconds

        prevTime = curTime

    # Print out the data for all data and speed
    print("base_x_axis: ", base_x_axis)
    print("base_y_axis: ", base_y_axis)
    print("base_z_axis: ", base_z_axis)
    print("x_axis: ", x_axis)
    print("y_axis: ", y_axis)
    print("z_axis: ", z_axis)
    print("speed: ", speed)

    # Stop car before it can run away
    if stopAtEnd:
        stopSpeed = 0
        setSpeed(stopSpeed)

    return speed


def toSpeed(base_x_axis, base_y_axis, base_z_axis, startSpeed, goalSpeed, slowStop):
    speed = startSpeed
    steer = 90

    setSpeed(speed)
    setRot(steer)

    x_axis = base_x_axis
    y_axis = base_y_axis
    z_axis = base_z_axis

    prevTime = time.time() * 1000  # Gets time in milliseconds
    accelDiff = True  # Assume accelDiff is true initially

    while speed != goalSpeed:
        if speed > goalSpeed:
            # Decrease speed by 1
            speed -= 1
            print("Decrease speed to: ", speed)
            setSpeed(speed)
        else:
            # Increase speed by 1
            speed += 1
            print(speed)
            setSpeed(speed)

        curTime = time.time() * 1000  # Gets time in milliseconds

        while (curTime - prevTime < 250) and accelDiff:
            # print("prevTime: " , prevTime)
            # print("curTime: ", curTime)

            curTime = time.time() * 1000  # Gets time in milliseconds

        prevTime = curTime

    # While the speed of the wheels aren't the same speed as the car
    # while(speed != goalSpeed and accelDiff):
    # 	if(speed > goalSpeed):
    # 		# Decrease speed by 1
    # 		speed -= 1
    # 		print("Decrease speed to: ", speed)
    # 		setSpeed(speed)
    # 	else:
    # 		# Increase speed by 1
    # 		speed += 1
    # 		print(speed)
    # 		setSpeed(speed)
    #
    # 	# Read in the new accelerometer data
    # 	x_axis, y_axis, z_axis = getAccel()
    #
    #
    # 	curTime = time.time() * 1000 # Gets time in milliseconds
    #
    # 	accelDiff = (x_axis != base_x_axis) or (y_axis != base_y_axis) or (z_axis != base_z_axis)
    #
    # 	while((curTime - prevTime < 500) and accelDiff):
    # 		x_axis, y_axis, z_axis = getAccel()
    #
    # 		#print("prevTime: " , prevTime)
    # 		#print("curTime: ", curTime)
    #
    # 		curTime = time.time() * 1000 # Gets time in milliseconds
    #
    # 	prevTime = curTime

    # Currently not working. Just set parameter to false
    if slowStop:
        slowChange(speed, 0, 250)
    else:
        stopSpeed = 0
        setSpeed(stopSpeed)

    return speed == goalSpeed


# Slow down function
def slowChange(currSpeed, goalSpeed, incTime):
    prevTime = time.time() * 1000  # Gets time in milliseconds

    while currSpeed != goalSpeed:
        if currSpeed > goalSpeed:
            # Decrease Speed by 1
            currSpeed -= 1
            setSpeed(currSpeed)
            print("Speed Decreased to: ", currSpeed)
        else:
            # Increase Speed by 1
            currSpeed += 1
            setSpeed(currSpeed)
            print("Speed Increased to: ", currSpeed)

        curTime = time.time() * 1000  # Gets time in milliseconds

        while curTime - prevTime < incTime:
            # print("prevTime: " , prevTime)
            # print("curTime: ", curTime)

            curTime = time.time() * 1000  # Gets time in milliseconds

        prevTime = curTime


# Script
running = True
stressTest = True

if running:
    if not stressTest:
        # Get the initial Accel Values
        x_axis, y_axis, z_axis = getAccel()

        # Increase speed until wheels start moving
        # Returns the "Start Speed" (The Speed Setting required to get the car to move)
        initSpeed = 0
        startSpeed = getCarToMove(x_axis, y_axis, z_axis, initSpeed, False)

        # Start the car at the "Start Speed" and reduce it until the goal speed
        goalSpeed = 25
        toSpeedCheck = toSpeed(x_axis, y_axis, z_axis, startSpeed, goalSpeed, True)
        print("toSpeedCheck: ", toSpeedCheck)
    else:
        print("Stop Car")
        # setRot(90)
        # time.sleep(1)
        # setRot(45)
        # time.sleep(1)
        setRot(90)
        # time.sleep(1)
        # setSpeed(30)
        # time.sleep(5)
        setSpeed(0)

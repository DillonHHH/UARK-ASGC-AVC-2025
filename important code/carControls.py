import serial
import time


# define serial port
serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
time.sleep(1)


# Print parameters
print("serial_port.baudrate: ", serial_port.baudrate)
print("serial_port.bytesize: ", serial_port.bytesize)
print("serial_port.parity: ", serial_port.parity)
print("serial_port.stopbits: ", serial_port.stopbits)


# function to compute checksum
def checksum(speed, rot):
    result = speed ^ rot
    return result


rot = 90
speed = 0
acc = 0

# initalize packet
packet = bytearray(4)

# game loop
running = True


def write():
    global rot
    global speed

    if rot > 135:
        rot = 135

    elif rot < 45:
        rot = 45

    speed = speed + acc

    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100

    # create and send out packet
    starting_byte = 255
    # packet[0] = starting_byte
    # packet[1] = speed
    # packet[2] = rot
    # packet[3] = checksum(speed, rot)

    # write out bytes
    serial_port.write(starting_byte.to_bytes(1, byteorder="little"))
    serial_port.write(speed.to_bytes(1, byteorder="little"))
    serial_port.write(rot.to_bytes(1, byteorder="little"))

    _checksum = speed ^ rot

    serial_port.write(_checksum.to_bytes(1, byteorder="little"))


def setSpeed(newSpeed: int):
    global speed
    if newSpeed < 0:
        newSpeed = 0
    elif newSpeed > 50:
        newSpeed = 50

    speed = newSpeed
    write()


def setRot(newRot: int):
    global rot
    if newRot < 45:
        newRot = 45
    elif newRot > 135:
        newRot = 135

    rot = newRot
    write()

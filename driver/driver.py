import serial
import time

encoder_1 = 0
encoder_2 = 0

angleMultiplier1 = 90/326000
angleMultiplier2 = 60/307778

def setSpeed(port: serial.Serial, motor: int, speed: int):
    #split speed into three bytes with twos complement MSB first
    speed_bytes = speed.to_bytes(3, byteorder='big', signed=True)
    #write command motornumber, 00, speed_bytes[0], speed_bytes[1], speed_bytes[2] with spaces between the bytes, the bytes just as ascii text without any formatting
    command = f"{motor} 01 {speed_bytes[0]:02X} {speed_bytes[1]:02X} {speed_bytes[2]:02X}\n"
    port.write(command.encode('ascii'))

def stopMotor(port: serial.Serial, motor: int):
    #write command motornumber, 05with spaces between the bytes, the bytes just as ascii text without any formatting
    command = f"{motor} 05\n"
    port.write(command.encode('ascii'))

def readEncoder(port: serial.Serial, motor: int):
    port.flushInput()  # Clear the input buffer to avoid reading old data
    #write command motornumber, 03 with spaces between the bytes, the bytes just as ascii text without any formatting
    command = f"{motor} 08\n"
    print(f"Sending command: {command.strip()}")  # Debugging line to see the command being sent
    port.write(command.encode('ascii'))
    #read response from serial port
    response = port.readline()
    print(f"Raw response bytes: {response}")  # Debugging line to see the raw response bytes
    response = response.decode('ascii').strip()  # Decode bytes to string and strip whitespace
    #split response into bytes
    response = response.split('RX: ')[1]
    response_bytes = response.split(' ')
    response_bytes = response_bytes[0:2]
    #decode the bytes into an integer with twos complement MSB first
    encoder_value = (int.from_bytes(bytes(int(b, 16) for b in response_bytes), byteorder='big', signed=True))
    if motor == 1:
        encoder_value = encoder_value * angleMultiplier1
        global encoder_1
        encoder_1 += encoder_value
        return encoder_1
    elif motor == 2:
        encoder_value = encoder_value * angleMultiplier2
        global encoder_2
        encoder_2 += encoder_value
        return encoder_2

def calibrateMotor(port: serial.Serial, motor: int):
    #write command motornumber, 04 with spaces between the bytes, the bytes just as ascii text without any formatting
    command = f"{motor} 04\n"
    port.write(command.encode('ascii'))

def gotoPosition(port: serial.Serial, motor: int, position: int, speed: int):
    currentpos1 = readEncoder(port, motor)
    moving_up = position > currentpos1
    if moving_up:
        setSpeed(port, motor, speed)
    else:
        setSpeed(port, motor, -speed)
    while True:
        time.sleep(0.2)
        currentpos1 = readEncoder(port, motor)
        print(f"Motor {motor} position: {currentpos1}")
        if (moving_up and currentpos1 >= position) or (not moving_up and currentpos1 <= position):
            stopMotor(port, motor)
            print(f"Motor {motor} position: {currentpos1}")
            break

def controlledGOTO(motor_number, goal_position, port):
    if abs(readEncoder(port, motor_number) - goal_position) > 2:
        gotoPosition(port, motor_number, goal_position, 7000000)
        time.sleep(0.1)
        gotoPosition(port, motor_number, goal_position, 4000000)
        time.sleep(0.1)
        gotoPosition(port, motor_number, goal_position, 2000000)
        time.sleep(0.1)
        gotoPosition(port, motor_number, goal_position, 600000)
        time.sleep(0.1)
        gotoPosition(port, motor_number, goal_position, 200000)
    time.sleep(0.1)
    gotoPosition(port, motor_number, goal_position, 100000)

def get_position():
    return encoder_1, encoder_2

if __name__ == "__main__":
    #open serial port
    port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    calibrateMotor(port, 1)
    time.sleep(3)
    calibrateMotor(port, 2)
    time.sleep(3)
    while True:
        motor_number = 0
        while motor_number not in [1, 2]:
            motor_number = int(input("Enter motor number (1 or 2): "))
        goal_position = int(input("Enter goal position for motor " + str(motor_number) + ": "))
        controlledGOTO(motor_number, goal_position, port)



    
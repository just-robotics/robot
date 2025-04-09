import serial
import time

from logging import *


def openSerialPort(ser):
    data = ser.readline().decode('ascii').strip()
    # print(f"data={data}")
    if (data == ""):
        ser.close()
        ser.open()
        # ser.reset_input_buffer()
        # time.sleep(1)
        ser.write(b'\r')
        time.sleep(0.1)
        ser.write(b'\r')
        time.sleep(1)
        data = ser.readline()

        while (not data):
            ser.write(b'\r\r')
            print("connecting...")
        data = ser.readline()
        ser.write(b'les\r')
        # print("Port is open")


def readSerial(ser):
    if ser.in_waiting > 0:
        data = ser.readline().decode('ascii').strip()
        if (data.find("est[") == -1):
            print("", end="")
            return None
        
        start_index = data.find("est[") + len("est[")
        end_index = data.find("]", start_index)
        est_content = data[start_index:end_index]
        values = est_content.split(",")

        x = float(values[0])
        y = float(values[1])
        z = float(values[2])

        return [x, y, z]


ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.1)
openSerialPort(ser)


while True:
    data = readSerial(ser)
    # prev_time = time.time()
    if data is not None:
        print(data)
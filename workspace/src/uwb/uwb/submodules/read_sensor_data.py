from serial import Serial
import time



def open_serial_port(ser : Serial):
    data = ser.readline().decode('ascii').strip()
    if (data == ''):
        if ser.is_open:
            ser.close()
        ser.open()
        ser.write(b'\r')
        time.sleep(0.1)
        ser.write(b'\r')
        time.sleep(1.0)
        data = ser.readline()

        while (not data):
            ser.write(b'\r\r')
            print('connecting...')
        data = ser.readline()
        ser.write(b'les\r')


def read_sensor_data(ser):
    if ser.in_waiting > 0:
        data = ser.readline().decode('ascii').strip()
        if (data.find('est[') == -1):
            return None
        
        start_index = data.find('est[') + len('est[')
        end_index = data.find(']', start_index)
        est_content = data[start_index:end_index]
        values = est_content.split(",")

        x = float(values[0])
        y = float(values[1])
        z = float(values[2])
        
        return [x, y, z]
        
# -------------------------------------------------------------- #
# ENVIRONMENT RELATED FUNCTIONS FOR UR ROBOT DEGRADATION TESTING #                            
# BS. 11.04.25                                                   #
# -------------------------------------------------------------- #

import os
import serial
import serial.tools.list_ports
import socket
import numpy as np
import copy

# --------------------------------------------------------------------------------------------------
# >>> SAMPLE GRID FUNCTIONS

class Sample:
    def __init__(self, id, index, data, outdex, status):
        self.id = id
        self.index = index
        self.data = data
        self.outdex = outdex
        self.status = status


def generate_sample_grid(columns, rows):
    """Generate a ssmple grid for the experiment."""
    samples = {}
    i = 1
    x = 1
    y = 1
    while i <= columns * rows:
        sample = Sample(i, [x, y], [], [], "in")
        samples[i] = sample
        i += 1
        y += 1
        if y == rows + 1:
            x += 1
            y = 1
    return samples

# --------------------------------------------------------------------------------------------------
# >>> BALANCE FUNCTIONS

# 1. CALIBRATE BALANCE
def calibrate_balance(balance, remote = True):
    # REMOTE
    if remote == True:
        # REMOTE
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((balance, 65432))
            # Send a request for the reading
            client_socket.sendall(b'CALIBRATE')
            client_socket.close()
            return None
    # LOCAL
    else:
        # Calibrate the balance automatically
        try:
            balance.write(b'C\r\n')
        except:
            balance = serial.Serial(port=balance_port, baudrate=9600, bytesize=serial.SEVENBITS,
                           parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
            balance.write(b'C\r\n')

# 2. TARE BALANCE
def tare_balance(balance, remote = True):
    # REMOTE
    if remote == True:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((balance, 65432))
        # Send a request for the reading
        client_socket.sendall(b'TARE')
        client_socket.close()
        return None
    # LOCAL
    else:
        # Set the balance measurement to 0.0 g
        try:
            balance.write(b'T\r\n')
        except:
            balance = serial.Serial(port=balance_port, baudrate=9600, bytesize=serial.SEVENBITS,
                           parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
            balance.write(b'T\r\n')

# 3. RECORD BALANCE DATA
def measure_weight(balance, remote=True, balance_port=None, raspberry_pi_ip=None):
    """
    Measures the weight using a scale, either locally or remotely.
    
    :param balance: Object or IP address of the scale (depending on the type).
    :param remote: Boolean indicating whether to use a remote scale (True) or a local one (False).
    :param balance_port: Port of the local scale (if remote=False).
    :param raspberry_pi_ip: IP address of the Raspberry Pi (if remote=True).
    
    :return: Measured weight (float).
    """
    # REMOTE
    if remote:
        if raspberry_pi_ip is None:
            raise ValueError("For a remote scale, the Raspberry Pi's IP address must be provided.")
        
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((raspberry_pi_ip, 65432))  # Connection port
        client_socket.sendall(b'MEASURE')  # Send measurement request
        data = client_socket.recv(1024)  # Receive response
        client_socket.close()
        
        try:
            return float(data.decode('utf-8'))  # Convert response to float
        except ValueError:
            print(f"Error converting response: {data.decode('utf-8')}")
            return None
    # LOCAL
    else:
        if balance_port is None:
            raise ValueError("For a local scale, the scale's port must be provided.")
        
        try:
            # Configure the local scale (if reconfiguration is needed)
            if not isinstance(balance, serial.Serial):  # If balance is not of type Serial
                balance = serial.Serial(port=balance_port, baudrate=9600, bytesize=serial.SEVENBITS,
                                         parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
            
            balance.flushInput()  # Clear any previous data in the input buffer
            balance.write(b'B\r\n')  # Command to get the measurement
            reading = balance.readline()  # Read the value from the scale
            weight = float(reading[:10])  # Assume the weight is in the first 10 characters
            
            return weight
        
        except Exception as e:
            print(f"Error measuring with the local scale: {e}")
            return None

# 4. SETUP REMOTE SCALE
def setup_remote_scale(access_remote):
    # Print the remote access value for debugging
    print(f"Accessing remote system: {access_remote}")  

    if access_remote:
        # If remote access is enabled, return the IP address
        print("Remote access enabled. Returning IP.")
        return True, "192.168.8.151"
    else:
        # If not remote, search for the scale's serial port
        print("Local access. Searching for serial port...")
        ports = serial.tools.list_ports.comports()
        balance_serial = "5658030514"  
        balance_port = None  # Ensure balance_port is always defined

        for port in ports:
            # Print each port for debugging purposes
            print(f"Checking port: {port.device}, Serial: {port.serial_number}")  
            if port.serial_number == balance_serial:
                balance_port = port.device
                break

        # If no matching port is found, raise an error
        if balance_port is None:
            raise ValueError(f"Port with serial number {balance_serial} not found")

        # If the port is found, establish the serial connection to the scale
        balance = serial.Serial(port=balance_port, baudrate=9600, bytesize=serial.SEVENBITS,
                                parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)

        return False, balance

# --------------------------------------------------------------------------------------------------
# >>> PHOTO STAND FUNCTIONS

import os
import cv2
import socket

def create_photo_directory(name):
    '''
    Creates a directory for saving photos, named with the current date and a custom name.
    This ensures that each set of photos is stored in a unique folder.
    The folder is created if it does not already exist.
    '''
    from time import strftime, localtime  # Import strftime and localtime to get the current date and time
    date = strftime('%m_%d', localtime())  # Format the current date as month_day
    photo_directory = f"../data/Photos_{date}_{name}"  # Create the directory path string
    os.makedirs(photo_directory, exist_ok=True)  # Create the directory; no error if it already exists
    return photo_directory  # Return the path to the created directory


def take_photo(camera_number = 1):
    '''
    Captures a photo using a specified camera.
    The default camera is set to 1, which could be either an on-robot camera or a photo stand camera.
    The camera stream is accessed through an IP address, and the autofocus is enabled.
    '''
    # camera_number = 1 for on-robot camera, 2 for photo stand camera
    camera = cv2.VideoCapture("http:/192.168.8.151:808"+str(camera_number))  # Access the camera stream via IP
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # Enable autofocus on the camera
    return_value1, img1 = camera.read()  # Read a frame from the camera
    camera.release()  # Release the camera resource after capturing the image
    return img1  # Return the captured image

# --------------------------------------------------------------------------------------------------
# >>> ARDUINO FUNCTIONS

def arduino(task, ip="192.168.8.151", port=65432):
    '''
    Sends a task or command to an Arduino device over a network.
    Establishes a socket connection to communicate with the Arduino, sends the task, and receives a response.
    The default IP and port are provided, but can be customized if needed.
    '''
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a TCP/IP socket
    client_socket.connect((ip, port))  # Connect to the Arduino device at the specified IP and port
    client_socket.sendall(task)  # Send the task/command to the Arduino
    data = client_socket.recv(1024)  # Receive up to 1024 bytes of response data from the Arduino
    client_socket.close()  # Close the socket connection
    return str(data)  # Return the received data as a string
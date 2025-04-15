# -------------------------------------------------------------- #
# ENVIRONMENT RELATED FUNCTIONS FOR UR ROBOT DEGRADATION TESTING #                            
# BS. 31.12.24                                                   #
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

# ---------------------- # 
# LOCAL BALANCE FUNCIONS #
# ---------------------- # 

# 1. CALIBRATE BALANCE
def calibrate_balance(balance):
    """
    Calibrates the balance by sending the calibration command 'C'.
    
    Parameters:
    balance (serial.Serial): The serial connection to the balance.

    If the balance is not already connected, a new serial connection is established.
    """
    try:
        balance.write(b'C\r\n')  # Send calibration command
    except:
        # Re-establish serial connection if an error occurs
        balance = serial.Serial(
            port=balance_port,
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        balance.write(b'C\r\n')

# 2. TARE BALANCE
def tare_balance(balance):
    """
    Tares the balance by setting the current measurement to zero.
    
    Parameters:
    balance (serial.Serial): The serial connection to the balance.

    If the balance is not connected, a new connection is established.
    """
    try:
        balance.write(b'T\r\n')  # Send tare command
    except:
        # Reconnect if the serial connection is lost
        balance = serial.Serial(
            port=balance_port,
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        balance.write(b'T\r\n')

# 3. RECORD BALANCE DATA
def measure_weight(balance):
    """
    Measures and returns the weight from the balance.
    
    Parameters:
    balance (serial.Serial): The serial connection to the balance.

    Returns:
    float: The measured weight from the balance.

    If the connection is interrupted, the function re-establishes the connection and retries.
    """
    try:
        # Clear any previous data in the buffer
        balance.read_all()
        
        # Send command to initiate measurement
        balance.write(b'B\r\n')
        
        # Read the response containing the weight value
        reading = balance.readline()
        weight = float(reading[:10])  # Extract and convert weight to float
        
    except:
        # Reconnect if serial connection fails
        balance = serial.Serial(
            port=balance_port,
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        
        balance.read_all()
        balance.write(b'B\r\n')
        reading = balance.readline()
        weight = float(reading[:10])
        
    return weight

# ----------------------- # 
# REMOTE BALANCE FUNCIONS #
# ----------------------- # 

# 1. TARE BALANCE
def tare_balance_remote(raspberry_pi_ip):
    """
    Sends a tare command to the remote balance connected to a Raspberry Pi.
    
    Parameters:
    raspberry_pi_ip (str): The IP address of the Raspberry Pi controlling the balance.

    Returns:
    None
    """
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((raspberry_pi_ip, 65432))  # Connect to the Raspberry Pi on port 65432
    
    client_socket.sendall(b'TARE')  # Send tare command to the remote balance
    client_socket.close()

    return None


# 2. CALIBRATE BALANCE
def calibrate_balance_remote(raspberry_pi_ip):
    """
    Sends a calibration command to the remote balance connected to a Raspberry Pi.
    
    Parameters:
    raspberry_pi_ip (str): The IP address of the Raspberry Pi controlling the balance.

    Returns:
    None
    """
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((raspberry_pi_ip, 65432))
    
    client_socket.sendall(b'CALIBRATE')  # Send calibration command
    client_socket.close()
    
    return None

# 3. RECORD BALANCE DATA
def measure_weight_remote(raspberry_pi_ip):
    """
    Sends a measurement request to the remote balance and returns the measured weight.
    
    Parameters:
    raspberry_pi_ip (str): The IP address of the Raspberry Pi controlling the balance.

    Returns:
    float: The measured weight received from the remote balance.
    """
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((raspberry_pi_ip, 65432))
    
    client_socket.sendall(b'MEASURE')  # Request measurement
    
    # Receive the measured weight as a byte string, decode and convert to float
    weight_data = client_socket.recv(1024)
    client_socket.close()
    
    return float(weight_data.decode('utf-8'))

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
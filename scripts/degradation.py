# ------------------------------------------ #
# FUNCTIONS FOR UR ROBOT DEGRADATION TESTING #                            
# BS. 31.12.24                               #
# ------------------------------------------ #

import datetime
import numpy as np
import serial
import csv
import pandas as pd
import socket
import math
from random import seed
from random import choice
import cv2
import time

import robot
import gripper
import environment

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO

# ---------------------------------------------------------------------------------------------------------------------
# 1. MANIPULATE BASIN LID

def move_lid(STATUS, rtde_c, rtde_r, rtde_io):
    """
    This function controls the movement of a robotic lid based on the provided status. 
    It also manages the gripping of the lid and performs actions depending on whether the lid should be opened or closed.

    Parameters:
    -----------
    STATUS : str
        The status of the lid movement. It can be "on" or "off":
            - "on": The lid should be in the "open" state (gripper closes).
            - "off": The lid should be in the "closed" state (gripper opens).
    
    rtde_c : RTDEControlInterface
    rtde_r : RTDEReadInterface
    rtde_io : RTDEIOInterface

    Returns:
    --------
    lid_position_before_move : float
        If the status is "off", it returns the position of the lid before the movement was executed.
        This can be used later if you need to know where the lid was before being moved to the "on" position.
        If the status is "on", no position is returned.

    Description:
    -----------
    - If the STATUS is "off":
        - The robot first moves downward and detects contact to simulate the process of closing the lid.
        - Then it moves to an intermediate position, followed by a move to a final position.
        - The robot detects contact at the final position, where it may open the gripper or keep it closed.
        - After the operation, the robot returns to the initial TCP position.
    
    - If the STATUS is "on":
        - The robot follows similar steps to those described for "off", but instead of opening the lid, it closes it to secure the lid.

    - The function ensures that the lid is either opened or closed based on the given STATUS, with necessary steps for detecting contact, controlling the gripper, and moving the robot.

    Notes:
    ------
    - The function expects that the gripper and movement actions are defined in other parts of the code, specifically the `open_grip` and `close_grip` functions.
    - The robot's movements are controlled using the `moveL` method for linear motion and `moveUntilContact` to detect contact with the surface.
    """
    
    current_tcp_position = rtde_r.getActualTCPPose() # Get the current tool center point (TCP) position
    gripper.open_grip(40, rtde_c, rtde_r, rtde_io)

    # Interface A/R: If the status is "off", perform actions to close the lid
    if STATUS == "off":
        downward_speed = [0, 0, -0.1, 0, 0, 0] # Define a speed vector to move downwards
        rtde_c.zeroFtSensor() # Zero the force/torque sensor to ensure accurate measurement
        rtde_c.moveUntilContact(downward_speed, direction=downward_speed) # Move the robot until contact is detected with the downward speed
        gripper.close_grip(rtde_c, rtde_r, rtde_io, force=40)
        lid_position_before_move = rtde_r.getActualTCPPose()[2]
        rtde_c.moveL(current_tcp_position, 0.05)

    intermediate_tcp_position = rtde_r.getActualTCPPose()
    final_lid_position = [0.6030834794381555, 0.9021261902449913, 0.39639681430164314, 0.01272613523271384, 3.1354295627494064, 0.0013375201682988259] # Define the final position for the lid to be moved to
    final_lid_position[2] += 0.05
    rtde_c.moveL(final_lid_position, 3, 1)
    
    # Move down to detect contact with the lid or surface below
    downward_speed = [0,0, -.1, 0, 0, 0]
    rtde_c.zeroFtSensor()
    rtde_c.moveUntilContact(downward_speed, direction=downward_speed)
    
    # Interface A/R: If the status is "off", open the gripper slightly to release the lid
    if STATUS == "off":
        gripper.open_grip(30, rtde_c, rtde_r, rtde_io)
    else:
        # If the status is "on", close the gripper with a force of 40 to keep the lid secured
        gripper.close_grip(rtde_c, rtde_r, rtde_io, force=40)

    rtde_c.moveL(final_lid_position, 3, 1)
    rtde_c.moveL(intermediate_tcp_position, 3, 1)

    # Interface A/R: If the status is "on", perform additional actions
    if STATUS == "on":
        downward_speed = [0,0, -.1, 0, 0, 0]
        rtde_c.zeroFtSensor()
        rtde_c.moveUntilContact(downward_speed, direction=downward_speed)
        gripper.open_grip(40, rtde_c, rtde_r, rtde_io)
        
        # Move the robot back to the initial position (restore the TCP position)
        rtde_c.moveL(current_tcp_position, 0.05)
    
    # If the status is "off", return the position of the lid before the move (PAS represents "Position Before Action")
    else:
        return lid_position_before_move

# ---------------------------------------------------------------------------------------------------------------------
# 2. MOVE UNTIL TOUCH TO CENTER THE GRID

import math

def center(temporal_position, rtde_c, rtde_r, rtde_io, OFFSETT):
    """
    This function is designed to center the robot's tool at a specific point in space based on measurements 
    taken along the X and Y axes. It uses the gripper's position and force sensors to determine the center 
    of an object (such as a bin) in the robot's workspace, adjusting its position accordingly.

    Parameters:
    -----------
    temporal_position : list
        The initial position of the robot. The robot will move from this position while performing 
        the centering operation. It is adjusted during the execution of the function.

    rtde_c : RTDEControlInterface
    rtde_r : RTDEReadInterface
    rtde_io : RTDEIOInterface
        
    OFFSETT : float
        An offset value used to adjust the final position of the robot, especially for Z-coordinate calibration. 
        It is applied to fine-tune the robot's final position after centering the object.

    Returns:
    --------
    final_pose : list
        The final position of the robot after centering the tool at the calculated position, including both 
        the position (X, Y, Z) and the orientation (rotation angle).

    rotation_angle : float
        The calculated rotation angle required for centering the robot based on the slope of the measurements 
        along the X and Y axes.

    Description:
    -----------
    - The function starts by closing the gripper with a specified force to prepare for the centering operation.
    - The robot first moves to a temporary position and adjusts for measurements along both the Y and X axes.
    - It performs a series of movements to measure contact points along the Y-axis and X-axis, calculating slopes 
      and intersections to find the center of the object.
    - The robot calculates the intersection point of the two lines (one for the Y-axis and one for the X-axis) 
      and uses this to determine the object's center in the workspace.
    - The robot then adjusts its position based on the calculated center, applies an offset for precision, 
      and moves to the final position.
    - The final position of the robot is returned, along with the rotation angle required for centering the object.

    Notes:
    ------
    - The function assumes that the gripper and movement actions (e.g., `moveL`, `moveUntilContact`) are defined 
      and configured properly in other parts of the code.
    - The function uses force feedback sensors to detect contact and determine the position of the object.
    - The robot’s alignment and rotation are carefully adjusted by calculating the slopes of the lines created 
      during the measurements.
    """

    gripper.close_grip(rtde_c, rtde_r, rtde_io, force=40)

    # Initial adjustment to temporary position for centering
    temporal_position[0] += 0.05

    while True:
        rtde_c.moveL(temporal_position, 1)

        # Gripper offset dimensions
        gripper_offset_y = 0.005
        gripper_offset_x = 0.009

        # First line (Y-axis measurement)
        # Move down along the Y-axis until contact
        speed = [0, -0.1, 0, 0, 0, 0]
        rtde_c.zeroFtSensor()
        rtde_c.moveUntilContact(speed, direction=speed)
        rtde_c.stopScript()
        pose_1 = rtde_r.getActualTCPPose()

        y1 = pose_1[1] - gripper_offset_y
        x1 = pose_1[0]

        # Move to the next position for further calculations
        pose_1[1] += 0.04
        rtde_c.moveL(pose_1, 0.5, 1)
        pose_1[0] -= 0.15
        rtde_c.moveL(pose_1, 0.5, 1)

        # Second measurement along the Y-axis
        rtde_c.zeroFtSensor()
        rtde_c.moveUntilContact(speed, direction=speed)
        rtde_c.stopScript()
        pose_2 = rtde_r.getActualTCPPose()

        y2 = pose_2[1] - gripper_offset_y
        x2 = pose_2[0]

        # Calculate slope for the first line
        try:
            slope_y = (y2 - y1) / (x2 - x1)
        except ZeroDivisionError:
            slope_y = y2 - y1
        intercept_y = y1 - slope_y * x1

        # Move back to a safer position
        pose_2[1] += 0.06
        rtde_c.moveL(pose_2, 0.5, 1)

        # Second line (X-axis measurement)
        speed = [-0.1, 0, 0, 0, 0, 0]
        rtde_c.zeroFtSensor()
        rtde_c.moveUntilContact(speed, direction=speed)
        rtde_c.stopScript()
        pose_3 = rtde_r.getActualTCPPose()

        y3 = pose_3[1]
        x3 = pose_3[0] - gripper_offset_x

        # Move to next position
        pose_3[0] += 0.04
        pose_3[1] += 0.15
        rtde_c.moveL(pose_3, 0.5, 1)

        # Second measurement along the X-axis
        rtde_c.zeroFtSensor()
        rtde_c.moveUntilContact(speed, direction=speed)
        rtde_c.stopScript()
        pose_4 = rtde_r.getActualTCPPose()

        y4 = pose_4[1]
        x4 = pose_4[0] - gripper_offset_x

        # Calculate slope for the second line
        try:
            slope_x = (y4 - y3) / (x4 - x3)
        except ZeroDivisionError:
            slope_x = y4 - y3
        intercept_x = y3 - slope_x * x3

        # Calculate intersection point
        try:
            X_intersection = (intercept_x - intercept_y) / (slope_y - slope_x)
        except ZeroDivisionError:
            X_intersection = (intercept_x - intercept_y)
        Y_intersection = slope_y * X_intersection + intercept_y

        # Calculate rotation angle
        rotation_angle = math.atan(slope_y)

        try:
            inverse_slope_y = 1 / slope_y
        except ZeroDivisionError:
            inverse_slope_y = 1

        inverse_slope_x = -1 * slope_x

        try:
            difference = abs(abs(inverse_slope_x) / abs(inverse_slope_y))
        except ZeroDivisionError:
            difference = abs(inverse_slope_x)

        # Check if alignment is within tolerance
        if 0.5 <= difference <= 1.5:
            break

    # Calculate the center of the bin
    x_center = X_intersection + (0.25 * math.cos(rotation_angle) - 0.15 * math.sin(rotation_angle))
    y_center = Y_intersection + (0.15 * math.cos(rotation_angle) + 0.25 * math.sin(rotation_angle))

    pose_2[0] = x_center
    pose_2[1] = y_center

    # Adjust to center on rack corner
    pose_2[0] -= (0.222 * math.cos(rotation_angle) - 0.102 * math.sin(rotation_angle))
    pose_2[1] -= (0.102 * math.cos(rotation_angle) + 0.222 * math.sin(rotation_angle))
    pose_2[2] = 0.4 + OFFSETT

    rtde_c.moveL(pose_2, 3, 1)
    final_pose = rtde_r.getActualTCPPose()

    return final_pose, rotation_angle

# ---------------------------------------------------------------------------------------------------------------------
#  3. USE SPONGE

def use_sponge(rtde_c, rtde_r, rtde_io):
    PI = rtde_r.getActualTCPPose()
    sequence = [-3,-2,-1,0,1,2,3]
    # Random X coordinate along the sponge
    RAND = choice(sequence)

    P0 = rtde_r.getActualTCPPose() 
    P0[0] += RAND * .01
    # Down
    P0[2] -= .05
    rtde_c.moveL(P0,2)
    # Up
    P0[2] += .05
    rtde_c.moveL(P0,2)
    # Over
    P0[1] -= .023
    rtde_c.moveL(P0,3,1)
    # Down & dab
    P0[2] -= .04
    rtde_c.moveL(P0,2,1)
    P0[2] -= .08
    rtde_c.moveL(P0,2,1)
    P0[1] += .01
    rtde_c.moveL(P0,2,1)
    P0[2] += .12
    rtde_c.moveL(P0,2,1)
    P0[1] -= .01
    rtde_c.moveL(P0,2,1)

    # Turn
    J0 = rtde_r.getActualQ()
    J0[-1] += np.pi
    rtde_c.moveJ(J0,3,3)

    P0 = rtde_r.getActualTCPPose()
    # Down & dab
    P0[2] -= .04
    rtde_c.moveL(P0,2,1)
    P0[2] -= .08
    rtde_c.moveL(P0,2,1)
    P0[1] += .01
    rtde_c.moveL(P0,2,1)
    P0[2] += .12
    rtde_c.moveL(P0,2,1)
    P0[1] -= .01
    rtde_c.moveL(P0,2,1)
    # Turn back
    J0[-1] -= np.pi
    rtde_c.moveJ(J0,3,3)

# ---------------------------------------------------------------------------------------------------------------------
# 4. USE OF SCALE & DATA COLLECTION

def use_scale(n, cycle_number, SAMPLE, rtde_c, rtde_r, rtde_io, balance, remote, photo_directory):
    """
    Performs three measurements of a sample using a robotic arm and scale, calculates the average, and stores the results.
    
    Parameters:
    n (int): Index of the sample being processed.
    cycle_number (int): Current cycle number of the measurement process.
    SAMPLE (list): List of sample objects where measurements are stored.
    rtde_c (object): RTDE control interface for robot movement.
    rtde_r (object): RTDE reader interface for reading robot state.
    rtde_io (object): RTDE IO interface for gripper control.
    balance (object): Serial connection to the balance.
    remote (bool): If True, performs remote measurement through a Raspberry Pi.
    photo_directory (str): Directory to save photos of each measurement step.
    """
    measurement_count = 1
    average_weight = 0
    initial_position = rtde_r.getActualTCPPose()  # Get current robot TCP position

    # Perform three measurements per sample
    while measurement_count <= 3:
        # Tare the balance to reset measurement to zero
        environment.tare_balance(balance, remote)
        time.sleep(3.5)
        
        # Lower the gripper to insert the sample
        initial_position[2] -= 0.072
        rtde_c.moveL(initial_position, 0.1)
        gripper.open_grip(40, rtde_c, rtde_r, rtde_io)
        
        # Rotate gripper to dislodge sample if it sticks
        joint_positions = rtde_r.getActualQ()
        joint_positions[-1] -= np.pi / 6
        rtde_c.moveJ(joint_positions, 3, 3)
        joint_positions[-1] += np.pi / 6
        rtde_c.moveJ(joint_positions, 3, 3)
        
        time.sleep(3.5)
        
        # Measure the weight and add it to the total
        measured_weight = float(environment.measure_weight(balance=balance, remote=remote, raspberry_pi_ip="192.168.8.151"))
        average_weight += measured_weight
        SAMPLE[n].data.append(measured_weight)
        
        # Capture photo of the measurement process
        photo = environment.take_photo(1)
        cv2.imwrite(photo_directory + 
                    "/Sample_" + str(n) + 
                    "_cycle_" + str(cycle_number) + 
                    "_" + str(measurement_count) + "_scale.png", photo)
        
        # Lower further for precise centering
        initial_position[2] -= 0.0085
        rtde_c.moveL(initial_position, 1, 0.5)
        
        # Slightly close gripper to help center the sample
        gripper.open_grip(15, rtde_c, rtde_r, rtde_io)
        
        # Shake to ensure proper placement
        shake(rtde_c, rtde_r, rtde_io)
        gripper.open_grip(10, rtde_c, rtde_r, rtde_io)
        shake(rtde_c, rtde_r, rtde_io)
        
        # Close gripper to secure sample
        gripper.close_grip(rtde_c, rtde_r, rtde_io, force=25)
        
        # Raise the sample after measurement
        initial_position[2] += 0.08
        rtde_c.moveL(initial_position, 3, 1)
        
        measurement_count += 1
        
    # Calculate average weight and add to results
    average_weight /= 3
    average_weight = round(average_weight, 3)
    SAMPLE[n].data.append(average_weight)

    # Raise the robot arm to the safe height
    initial_position[2] += 0.1
    rtde_c.moveL(initial_position, 3, 1)

# ---------------------------------------------------------------------------------------------------------------------
# 5. MOVE TO PHOTO STAND

def photo_stand(n, cycle_number, rtde_c, rtde_r, rtde_io, photo_directory):
    photo_position = rtde_r.getActualTCPPose()
    photo_position[0] += 0.1
    photo_position[1] += 0.07
    photo_position[2] -= 0.07
    rtde_c.moveL(photo_position,3,1)

    photo = environment.take_photo(2)
    cv2.imwrite(photo_directory + "/Sample_" + str(n) + "_cycle_" + str(cycle_number) +"_front.png", photo)

    joints = rtde_r.getActualQ()
    joints[-1] += np.pi/2
    rtde_c.moveJ(joints,3,3)

    photo = environment.take_photo(2)
    cv2.imwrite(photo_directory + "/Sample_" + str(n) + "_cycle_" + str(cycle_number) +"_side.png", photo)
                    
# ---------------------------------------------------------------------------------------------------------------------
# 6. SHAKE FOR IMPROVED COMPLIANCE

def shake(rtde_c, rtde_r, rtde_io):
    P0 = rtde_r.getActualTCPPose()
    P0[0] += .002
    rtde_c.moveL(P0,2,.75)
    P0[0] -= .004
    rtde_c.moveL(P0,2,.75)
    P0[0] += .002
    rtde_c.moveL(P0,2,.75)
    P0[1] += .004
    rtde_c.moveL(P0,2,.75)
    P0[1] -= .008
    rtde_c.moveL(P0,2,.75)
    P0[1] += .004
    rtde_c.moveL(P0,2,.75)

# ---------------------------------------------------------------------------------------------------------------------
# 7. CALCULATE DATA AND ADD TO CSV FOR RESULTS

def add_csv_stats(filename, POPNUM):
    # Read the CSV file into a Pandas DataFrame
    df = pd.read_csv(filename)
    # Identify the columns whose headers do not include the words "average" or "sample"
    columns_to_include = [col for col in df.columns if 'average' not in col.lower() and 'sample' not in col.lower()]
    # Split the DataFrame into segments of n rows each for different material populations
    num_rows = df.shape[0]
    segments = [df[i:i+POPNUM] for i in range(0, num_rows, POPNUM)]
    # Extract values for each segment and store them in a list for standard deviation
    value_lists = []
    for segment in segments:
        values = segment[columns_to_include].values.flatten().tolist()
        value_lists.append(values)
        TEMP = [x for x in value_lists if not (isinstance(x, float) and np.isnan(x)) and x != 'nan' and x != '']
    #print(TEMP)
    ANS = {}
    for i, sublist in enumerate(TEMP):
        sublist_title = f"Material {i+1} Std. Deviation:"
        std = round(np.std(sublist),6)
        ANS[sublist_title] = std
    print(ANS)
    # Identify the columns whose headers include the word "average"
    average_columns = [col for col in df.columns if 'average' in col.lower()]
    # Add blank column for formatting
    df.insert(0,'', '')
    dfsd = pd.DataFrame()
    for key in ANS:
        #df.insert(1,key,ANS[key])
        dfsd.at[1,key] = ANS[key]
    df = pd.concat([dfsd,df],ignore_index=True)
    # Compute the average for each row for these columns
    df.insert(0,'Average of Averages', round(df[average_columns].mean(axis=1),4))
    # Transpose the DataFrame
    tdf = df.set_index('Sample').transpose()
    # Reset the index to ensure proper formatting
    tdf.reset_index(inplace=True)
    # Save the updated DataFrame back to a CSV file
    newname = 'DATA_' + filename 
    
    tdf.to_csv(newname, index=False)

# ---------------------------------------------------------------------------------------------------------------------
# CYCLE OPTIONS

# 1. INSERT SAMPLE IN EXTERNAL TRAY
def replace_sample_out(rtde_c, rtde_r, rtde_io, PD, initial_position):
    """ 
    Insert sample in external tray.
    """
    # Return sample to its original position
    rtde_c.moveL(PD, 3, 1)
    
    # Move down to insert sample back into its position
    PD[2] -= 0.188
    rtde_c.moveL(PD, 0.05)
    
    # Open gripper slightly and insert sample back
    gripper.open_grip(7, rtde_c, rtde_r, rtde_io)  # Slightly open gripper
    PD[2] -= 0.012
    rtde_c.moveL(PD, 0.05)
    
    # Shake to ensure proper placement
    shake(rtde_c, rtde_r, rtde_io)
    
    # Fully open gripper to release the sample
    gripper.open_grip(15, rtde_c, rtde_r, rtde_io)
    
    # Move up after insertion
    PD[2] += 0.04  
    PD[2] += 0.26  
    rtde_c.moveL(PD, 3, 1)
    
    # Move gripper back to initial position
    initial_position[2] += 0.15
    rtde_c.moveL(initial_position, 3, 1)
    initial_position[2] -= 0.15
    rtde_c.moveL(initial_position, 3, 1)

# 2. INSERT SAMPLE BACK INTO THE TRAY
def replace_sample_in(rtde_c, rtde_r, rtde_io, P0):
    """ 
    Insert sample back into the tray.
    """
    # Return sample to its original position
    rtde_c.moveL(P0, 3, 1)
    
    # Move down to insert sample back into its position
    P0[2] -= 0.188
    rtde_c.moveL(P0, 0.05)
    
    # Open gripper slightly and insert sample back
    gripper.open_grip(7, rtde_c, rtde_r, rtde_io)
    P0[2] -= 0.012
    rtde_c.moveL(P0, 0.05)
    
    # Shake to ensure proper placement
    shake(rtde_c, rtde_r, rtde_io)
    
    # Fully open gripper to release the sample
    gripper.open_grip(15, rtde_c, rtde_r, rtde_io)
    
    # Move up after insertion
    P0[2] += 0.04  
    rtde_c.moveL(P0, 3, 1)

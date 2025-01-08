# ---------------------------- #
# BASIC FUNCTIONS FOR UR ROBOT #                            
# BS. 31.12.24                 #
# ---------------------------- #

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO

import time

# 1. ROBOT CONNECTION 
def connect_robot(robot_ip):
    """
    Connects to the robot using RTDE interfaces for control, receive, and IO.

    Args:
    - robot_ip
    
    Returns:
    - tuple: rtde_c (RTDE control interface), rtde_r (RTDE receive interface), rtde_io (RTDE IO interface).
    
    Raises:
    - Exception: If any of the interfaces fail to connect.
    """
    try:
        # Connect to RTDE Control interface
        rtde_c = RTDEControl(robot_ip, 500.0, RTDEControl.FLAG_USE_EXT_UR_CAP)
        print("Connection to control interface established.")

        # Connect to RTDE Receive interface
        rtde_r = RTDEReceive(robot_ip)
        print("Connection to receive interface established.")

        # Connect to RTDE IO interface
        rtde_io = RTDEIO(robot_ip)
        print("Connection to IO interface established.")

        # Optionally, retrieve and print the current joint positions
        current_pos = rtde_r.getActualQ()
        print("Current joint positions:", current_pos)

        return rtde_c, rtde_r, rtde_io

    except Exception as e:
        print("Error while connecting to the robot:", e)
        raise

# 2. SET POSITION
def set_initial_position(rtde_c):
    """
    Moves the robot to the specified initial position before performing any actions.

    Args:
    - rtde_c: RTDE control interface for robot movements.
    """
    # Define the initial joint positions
    initial_pos = [1.7239642143249512, -2.054093977014059, -0.8874862194061279, -1.7807942829527796, 1.5661470890045166, 1.714949131011963]

    # Move the robot to the initial position using a linear movement
    rtde_c.moveJ(initial_pos)  # Use moveJ for joint space movement

    # Optional: Wait a short time for the robot to reach the position
    time.sleep(1)
    print("Robot moved to initial position.")

# 3. ROBOT ONLINE
def robot_online(rtde_r):
    return not (not rtde_r.isConnected() or rtde_r.isProtectiveStopped() or rtde_r.isEmergencyStopped())


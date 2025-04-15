# ------------------------------ #
# GRIPPER FUNCTIONS FOR UR ROBOT #                            
# BS. 31.12.24                   #
# ------------------------------ #

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface as RTDEIO

import time

def open_grip(open_distance, rtde_c, rtde_r, rtde_io):
    """
    Opens the robotic gripper to a specified distance.
    
    Parameters:
    - open_distance (float): Distance in millimeters to open the gripper.
    - rtde_c: RTDE control interface (for robot control).
    - rtde_r: RTDE read interface (for reading robot state).
    - rtde_io: RTDE IO interface (for input/output communication).
    """
    force = 25  # Apply 25 N of force to ensure controlled grip opening
    
    # Set the applied force in the input register 19
    rtde_io.setInputDoubleRegister(19, force)
    
    # Set the desired opening distance in input register 18
    rtde_io.setInputDoubleRegister(18, open_distance)
    
    # Write 1 to input register 18 to initiate the gripper opening process
    rtde_io.setInputIntRegister(18, 1)

    time.sleep(0.5)
    
    # Check if the gripper has finished moving by reading output register 18
    if rtde_r.getOutputIntRegister(18) == 1:
        time.sleep(0.1)
    
    # Reset the input register by writing 0 to prepare for the next command
    rtde_io.setInputIntRegister(18, 0)
    

def close_grip(rtde_c, rtde_r, rtde_io, force=25):
    """
    Closes the robotic gripper to a fully closed position.
    
    Parameters:
    - rtde_c: RTDE control interface (for robot control).
    - rtde_r: RTDE read interface (for reading robot state).
    - rtde_io: RTDE IO interface (for input/output communication).
    - force (float, optional): Force to apply when closing the gripper. Default is 25 N.
    """
    force = 25  # Apply 25 N of force to ensure a firm grip
    closed_distance = 0  # Target position is 0 mm, meaning fully closed
    
    # Set the applied force in input register 19
    rtde_io.setInputDoubleRegister(19, force)
    
    # Set the desired closing distance in input register 18
    rtde_io.setInputDoubleRegister(18, closed_distance)
    
    # Write 1 to input register 18 to trigger the closing action
    rtde_io.setInputIntRegister(18, 1)

    time.sleep(0.5)
    
    # Check if the gripper has fully closed by reading output register 18
    if rtde_r.getOutputIntRegister(18) == 1:
        time.sleep(0.1)
    
    # Reset the input register by writing 0 to prepare for the next operation
    rtde_io.setInputIntRegister(18, 0)

# ------------ #
# LISTENER.PY  #                            
# BS. 11.04.25 #
# ------------ #

import socket
import threading
import queue, time
import json

import time
import os
import numpy as np
import math
import copy
import csv
import json
import pandas as pd
import matplotlib.pyplot as plt

import robot
import gripper
import environment
import data_processing
import degradation

# Queue to hold incoming commands
command_queue = queue.Queue()

# Flag to indicate if the robot is busy
is_busy = False

PORT = 5000

# CODE FOR EXPERIMENT IN THIS FUNCTION

def execute_command(command_data):
    """
    Execute a robot command using the ur-rtde API.
    """
    global is_busy
    is_busy = True
    
    print("Experiment running") 
    
    setup = command_data.get("setup")
    print("Performing experiment using setup ", setup)

    with open("config.json", "r") as file:
        config = json.load(file)
    
    robot_ip = config["robot"]["robot_ip"]
    setup = command_data.get("setup")
    material = command_data.get("material")
    temperature = command_data.get("temperature")
    date = command_data.get("date")
    time_delay = command_data.get("time_delay")
    rows = command_data.get("rows")
    columns = command_data.get("columns")
    choice = command_data.get("choice")
    name = f"{material}_{temperature}_{date}"

    print("\n--- Robot Connection ---")
    print("Connecting to the robot...")
    rtde_c, rtde_r, rtde_io = robot.connect_robot(robot_ip)
    print("Robot connected successfully.")
    robot.set_initial_position(rtde_c, setup)

    photo_dir = f"../data/Photos_{name}"
    if not os.path.exists(photo_dir): # Check if directory exists
        os.makedirs(photo_dir)
        print(f"Directory '{photo_dir}' created.")
    else:
        print(f"Directory '{photo_dir}' already exists.")
    
    temperature_registry = f"../data/TemperatureRegistry_{name}.txt"
    prev_time = int(time.strftime('%H%M%S', time.localtime()))

    # Robot Environment settings
    SAMPLE = environment.generate_sample_grid(columns,rows)

    OFFSET = -.068  # Z offset
    gripper_length = .05  # Gripper length adjustment
    OFFSET += gripper_length
    
    scale_access_remote = input("Allow remote scale access? (y/n): ").lower() == "y"
    remote, balance = environment.setup_remote_scale(scale_access_remote)

    lid_position = rtde_r.getActualTCPPose()  # Devuelve [X, Y, Z, RX, RY, RZ]
    print(f"Posición actual: {lid_position}")

    if setup == 1:
        lid_deposition = [-0.24133534388522648, 0.2131944897595923, 0.35, 2.224008047304794, -2.2121364315342986, -0.010991599941659188]
    elif setup == 2:
        lid_deposition = [0.6556738335891733, -0.32250568064465923, 0.4362477404307668, 2.267314033738123, -2.13353507951682, 0.026926286486254704]

    cycle_number = config["experiment"]["starting_cycle"]
    fields = ['Sample', 'Measure 1 (g)', 'Measure 2 (g)', 'Measure 3 (g)', 'Average (g)', 'Time of Test', 'Temperature (C)']  # Fields for the CSV

    print("_Removing lid...")
    rtde_c.moveL(lid_position, 3, 1)
    temporal_position = rtde_r.getActualTCPPose()
    degradation.move_lid('off', rtde_c, rtde_r, rtde_io)
    temporal_position[2] = .3 + OFFSET
    print("_Callibrating...")
    intersection, angle_deviation = degradation.center(temporal_position, rtde_c, rtde_r, rtde_io, OFFSET)
    X_intersection = intersection[0]
    Y_intersection = intersection[1]
        
    # Preparation
    gripper.open_grip(15, rtde_c, rtde_r, rtde_io)
    rtde_c.moveL(intersection, 3, 1)

    # Calculate positions for sponge and scale
    sponge_position = rtde_r.getActualTCPPose()
    X_sponge = .225
    Y_sponge = -.125 + .02
    sponge_position[0] = X_intersection + (X_sponge * math.cos(angle_deviation) - Y_sponge * math.sin(angle_deviation))
    sponge_position[1] = Y_intersection + (Y_sponge * math.cos(angle_deviation) + X_sponge * math.sin(angle_deviation))

    if setup == 2:
        scale_position = [0.10645207840498347, 0.6936982017571437, 0.28658622705355286, -2.222037213084921, 2.2142596251486433, -0.007182138314705186]

    elif setup == 1:
        scale_position = rtde_r.getActualTCPPose()

        X_scale = 0.1365 + 0.165
        Y_scale = -0.1875

        scale_position[0] = X_intersection + (
            X_scale * math.cos(angle_deviation) - Y_scale * math.sin(angle_deviation)
        )
        scale_position[1] = Y_intersection + (
            Y_scale * math.cos(angle_deviation) + X_scale * math.sin(angle_deviation)
        )
        scale_position[2] -= 0.095
        
    # Move and rotate the gripper
    J0 = rtde_r.getActualQ()
    J0[-1] -= angle_deviation
    rtde_c.moveJ(J0, 3, 1)
    P0 = rtde_r.getActualTCPPose()
    P0[2] -= .16
    rtde_c.moveL(P0, 3, 1)

    filename = f"../data/WT_{time.strftime('%d.%m.%y', time.localtime())}_{name}"
    csv_file = filename + '.csv'
    png_file = filename + '.png'
    time.sleep(1)
    
   
    for sample in command_data.get("samples"):
        print("__Measuring sample " + str(sample))
        n =int(sample)
        time.sleep(1)

        # CALCULATE GRID POSITION 
        P0 = rtde_r.getActualTCPPose()
        # Pull coords from class
        xtemp = .02 * (SAMPLE[n].index[0] - 1)
        ytemp = .02 * (SAMPLE[n].index[1] - 1)
        P0[0] = X_intersection + (xtemp * math.cos(angle_deviation) - ytemp * math.sin(angle_deviation))  # Rot transformation X
        if n > 121:
            P0[0] += 0.003
        P0[1] = Y_intersection + (ytemp * math.cos(angle_deviation) + xtemp * math.sin(angle_deviation))  # Rot transformation Y
        rtde_c.moveL(P0, 0.3, 1)

        # CALCULATE DEPOSIT POSITION
        PD = copy.copy(lid_deposition)

        PD[0] += xtemp
        PD[1] += ytemp

        # Sample collection (Picking the sample)
        P0 = rtde_r.getActualTCPPose()
        initial_position = copy.copy(P0)

        # Move down to collect sample
        P0[2] -= .04  # .05 or .21 depending on the position
        rtde_c.moveL(P0, .05)
            
        # Open grip slightly to help center the sample
        gripper.open_grip(15, rtde_c, rtde_r, rtde_io)  # Opening the gripper slightly
            
        # Shake sample to ensure it's secured
        degradation.shake(rtde_c, rtde_r, rtde_io)
        gripper.open_grip(10, rtde_c, rtde_r, rtde_io)  # Slightly opening to release tension
            
        # Shake it again to make sure it's secure
        degradation.shake(rtde_c, rtde_r, rtde_io)
            
        # Close the grip to secure the sample
        gripper.close_grip(rtde_c, rtde_r, rtde_io, force=25)
            
        # Move the gripper up to the collection position
        P0[2] += .3  # .21 or appropriate distance for sample collection
        rtde_c.moveL(P0, 3, 1)

        # TASKS: Move over the basin and apply delay for the use of compressed air and sponge
        if setup == 1:
            Pi = rtde_r.getActualTCPPose()
            P1 = copy.copy(Pi)
            P1[0] = -0.1961714502764558
            P1[1] = 0.6999493118395084
            rtde_c.moveL(P1, 3, 1)
            P1[2] -= 0.25
            rtde_c.moveL(P1, 3, 1)
            # Use compressed air here!
            environment.arduino(b'OPEN_VALVE')
            time.sleep(1)
            environment.arduino(b'CLOSE_VALVE')
            time.sleep(1)
            environment.arduino(b'OPEN_VALVE')
            time.sleep(0.5)
            environment.arduino(b'CLOSE_VALVE')
                
            # Move back up after using air
            P1[2] += .25
            rtde_c.moveL(P1, 3, 1)
                

        elif setup == 2:
            Pi = rtde_r.getActualTCPPose()
            P1 = copy.copy(Pi)
            P1[0] = 0.5416628641896849
            P1[1] = 0.09024120194746174 - 0.20
            rtde_c.moveL(P1, 3, 1)
            P1[2] -= 0.25
            rtde_c.moveL(P1, 3, 1)
            # Use compressed air here!
            environment.arduino(b'OPEN_VALVE')
            time.sleep(1)
            environment.arduino(b'CLOSE_VALVE')
            time.sleep(1)
            environment.arduino(b'OPEN_VALVE')
            time.sleep(0.5)
            environment.arduino(b'CLOSE_VALVE')
                
            # Move back up after using air
            P1[2] += .25
            rtde_c.moveL(P1, 3, 1)
        
            
        # Move to sponge position and use sponge to clean the sample
        if robot.robot_online(rtde_r) == 'False':
            continuation = input("Robot Offline! If you want to continue, reconnect and press ENTER")
        rtde_c.moveL(sponge_position, 3, 1)
        degradation.use_sponge(rtde_c, rtde_r, rtde_io)
        P3 = rtde_r.getActualTCPPose()
        P4 = copy.copy(P3)
        P4[2] += 0.50
            
        # Move to scale position to measure the sample
        rtde_c.moveL(scale_position, 3, 1)
        degradation.photo_stand(n, cycle_number, rtde_c, rtde_r, rtde_io, photo_dir)
            
        # Measure the weight of the sample on the scale
        rtde_c.moveL(scale_position, 3, 1)
        degradation.use_scale(n, cycle_number, SAMPLE, rtde_c, rtde_r, rtde_io, balance, remote, photo_dir)

        # Add timestamp and temperature measurement for the sample
        SAMPLE[n].data.append(time.strftime('%H:%M:%S | %Y-%m-%d', time.localtime()))
        temperature = environment.arduino(b'TEMPERATURE')
        while len(temperature.split()) < 5:  # Ensure temperature is valid
            temperature = environment.arduino(b'TEMPERATURE')
            time.sleep(1)
        SAMPLE[n].data.append(temperature.split()[4])  # Append temperature data

        # Execution loop
        if choice == 1:
            print("Running 'Remove sample to external tray'...")
            degradation.replace_sample_out(rtde_c, rtde_r, rtde_io, PD, initial_position)
                
        elif choice == 2:
            print("Running 'Insert sample into water bath'...")
            degradation.replace_sample_in(rtde_c, rtde_r, rtde_io, P0)
                
        # >>> RETURN AND REPEAT
        # After completing the cycle for all samples, proceed to save and repeat the cycle

        # Index and save data every cycle
    CSV = []
    for n in SAMPLE.keys():
        if SAMPLE[n].data == []:
            pass
        else:
            row = [SAMPLE[n].id]
            for value in SAMPLE[n].data:
                row.append(value)
            CSV.append(row)

    # Save CSV file with the collected data
    with open(csv_file, 'w') as csvfile:
        # Create a csv writer object
        csv_writer = csv.writer(csvfile)
        # Write header (fields)
        csv_writer.writerow(fields)
        # Write data rows
        for row in CSV:
            csv_writer.writerow(row)

        # Replace the lid and move to the next cycle
    rtde_c.moveL(lid_position, 3, 1)
    degradation.move_lid("on", rtde_c, rtde_r, rtde_io)

        # Log temperature during the cycle
    with open(temperature_registry, 'a') as f:
        while True:
            if robot.robot_online(rtde_r) == 'False':
                continuation = input("Robot Offline! If you want to continue, reconnect and press Enter")
                
            # Log temperature data every 10 minutes
            current_time = int(time.strftime('%H%M', time.localtime()))
            if current_time % 10 == 0 and current_time != prev_time:
                temperature = environment.arduino(b'TEMPERATURE')
                while len(temperature.split()) < 5:  # Ensure temperature is valid
                    temperature = environment.arduino(b'TEMPERATURE')
                    time.sleep(1)
                f.write(str(time.strftime('%m/%d, %H:%M:%S', time.localtime())+', '+temperature.split()[4]) + '\n')
                prev_time = current_time
                time.sleep(1)
                
                # Check if it is time to break the cycle
            if time.strftime('%H%M', time.localtime()) == time_delay:
                break
            time.sleep(1)
                
    print("_Closing lid...")
    is_busy = False
        
def process_queue():
    """
    Process commands from the queue.
    """
    while True:
        if not command_queue.empty() and not is_busy:
            command = command_queue.get()
            print(f"Executing command: {command}")

            execute_command(command)
            command_queue.task_done()

def handle_client(conn, addr):
    print(f"Connected by {addr}")
    with conn.makefile('r') as f:
        for line in f:
            try:
                command_data = json.loads(line.strip())
                print(f"Received command: {command_data}")

                if is_busy:
                    response = {"status": "busy", "message": "Robot is busy. Command queued."}
                else:
                    response = {"status": "accepted", "message": "Command accepted. Executing."}
                
                command_queue.put(command_data)
                conn.sendall(json.dumps(response).encode())

            except json.JSONDecodeError:
                response = {"status": "error", "message": "Invalid JSON format."}
                conn.sendall(json.dumps(response).encode())
    conn.close()

def start_server():
    """
    Start a socket server to listen for incoming commands.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('0.0.0.0', PORT))
        s.listen()
        print(f"Listening for commands on port {PORT}...")

        while True:
            conn, addr = s.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr))
            client_thread.start()

if __name__ == "__main__":
    # Start the queue processing thread
    queue_thread = threading.Thread(target=process_queue, daemon=True)
    queue_thread.start()

    # Start the server to listen for commands
    start_server()

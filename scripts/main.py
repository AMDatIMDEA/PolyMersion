# -------------------------------------------- #
# MAIN SCRIPT FOR UR ROBOT DEGRADATION TESTING #                            
# BS. 31.12.24                                 #
# -------------------------------------------- #

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

def main():

    with open("config.json", "r") as file:
        config = json.load(file)

    robot_ip = config["robot"]["robot_ip"]

    # STEP 1: ROBOT CONNECTION
    print("\n--- 1. Robot Connection ---")
    print("Connecting to the robot...")
    rtde_c, rtde_r, rtde_io = robot.connect_robot(robot_ip)
    print("Robot connected successfully.")
    robot.set_initial_position(rtde_c)

    # STEP 2: EXPERIMENT SETUP
    print("\n--- 2. Experiment setup ---")

    material = config["experiment"]["material"]
    temperature = config["experiment"]["temperature"]
    cycles = config["experiment"]["cycles"]
    samples_per_group = config["experiment"]["samples_per_group"]
    columns = config["grid"]["columns"]
    rows = config["grid"]["rows"]
    date = time.strftime('%m_%d', time.localtime())

    name = f"{material}_{temperature}_{date}"

    print(f"Name: {name}")
    print(f"Cycles: {cycles}")
    print(f"Samples per group: {samples_per_group}")

    # Store user choice
    choice = None
    # Selection loop 
    while choice not in ["1", "2"]:
        print("\nChoose an action:")
        print("1. Remove sample to external tray")
        print("2. Insert sample into water bath")
        choice = input("Enter option (1 or 2): ")

    photo_dir = f"../data/Photos_{name}"
    if not os.path.exists(photo_dir): # Check if directory exists
        os.makedirs(photo_dir)
        print(f"Directory '{photo_dir}' created.")
    else:
        print(f"Directory '{photo_dir}' already exists.")
    
    temperature_registry = f"../data/TemperatureRegistry_{name}.txt"
    prev_time = int(time.strftime('%H%M%S', time.localtime()))
    
    # Groups to be sampled during each cycle
    Groups = {
        i: np.arange((i - 1) * 11 + 1, (i - 1) * 11 + 11)[:samples_per_group]
        for i in range(1, cycles + 1)
    }

    # Print the result
    for group, samples in Groups.items():
        print(f"Group {group}: {samples}")

    hours_delay = config["timing"]["hours_delay"] # Delay between cycles in hours
    minutes_delay = config["timing"]["minutes_delay"] # Delay between cycles in minutes

    # STEP 3: ENVIRONMENT SETUP
    print("\n--- 3. Environment setup ---")

    # Robot Environment settings
    SAMPLE = environment.generate_sample_grid(columns,rows)

    OFFSET = -.068  # Z offset
    gripper_length = .05  # Gripper length adjustment
    OFFSET += gripper_length
    
    scale_access_remote = input("Allow remote scale access? (y/n): ").lower() == "y"
    remote, balance = environment.setup_remote_scale(scale_access_remote)

    # Initial conditions
    lid_position = [0.027553934039555448, 0.977422284211569, 0.45, -2.221727243766157, 2.2141086299174013, -0.005764703144384496]
    lid_deposition = [-0.24133534388522648, 0.2131944897595923, 0.35, 2.224008047304794, -2.2121364315342986, -0.010991599941659188]
    cycle_number = config["experiment"]["starting_cycle"]
    fields = ['Sample', 'Measure 1 (g)', 'Measure 2 (g)', 'Measure 3 (g)', 'Average (g)', 'Time of Test', 'Temperature (C)']  # Fields for the CSV

    # STEP 4: PRINCIPAL LOOP
    # >>> LOOP THROUGH CYCLES
    while cycle_number <= cycles:
        # Time delay management
        M = int(time.strftime('%M', time.localtime())) + minutes_delay
        H = int(time.strftime('%H', time.localtime())) + hours_delay
        if M >= 60:
            M -= 60
            H += 1
        if len(str(M)) == 1:
            M = '0' + str(M)
        if H >= 24:
            H = int(H) - 24 
        if len(str(H)) == 1:
            H = '0' + str(H)
        time_delay = str(H) + str(M)
        if len(str(time_delay)) == 3:
            time_delay = '0' + str(time_delay)
        
        print('Current Time: ', time.strftime('%H%M', time.localtime()))
        print('Next Cycle: ', time_delay)

        print(f"\n--- Starting cycle {cycle_number} of {cycles} cycle/s. ---")

        # Calibration
        rtde_c.moveL(lid_position, 3, 1)
        temporal_position = rtde_r.getActualTCPPose()
        degradation.move_lid('off', rtde_c, rtde_r, rtde_io)
        temporal_position[2] = .3 + OFFSET
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

        scale_position = rtde_r.getActualTCPPose()
        X_scale = (.1365 + .165)
        Y_scale = -.1875
        scale_position[0] = X_intersection + (X_scale * math.cos(angle_deviation) - Y_scale * math.sin(angle_deviation))
        scale_position[1] = Y_intersection + (Y_scale * math.cos(angle_deviation) + X_scale * math.sin(angle_deviation))
        scale_position[2] -= .095

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

        # >>> LOOP FOR SAMPLES
        SEQUENCE = Groups[cycle_number]
        for n in SEQUENCE:
            if robot.robot_online(rtde_r) == 'False':
                continuation = input("Robot Offline! If you want to continue, reconnect and press ENTER")

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
            P0[2] += .2  # .21 or appropriate distance for sample collection
            rtde_c.moveL(P0, 3, 1)

            # TASKS: Move over the basin and apply delay for the use of compressed air and sponge
            P1 = rtde_r.getActualTCPPose()
            P1[0] = -0.1961714502764558
            P1[1] = 0.6999493118395084
            rtde_c.moveL(P1, 3, 1)
            P1[2] -= .25
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
            rtde_c.moveL(sponge_position, 3, 1)
            degradation.use_sponge(rtde_c, rtde_r, rtde_io)
            
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
            if choice == "1":
                print("Running 'Remove sample to external tray'...")
                degradation.replace_sample_out(rtde_c, rtde_r, rtde_io, PD, initial_position)
                
            elif choice == "2":
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

        # Now allow the cycle to repeat
        cycle_number += 1

        if cycle_number > cycles:
            break
    
    print("Experiment complete.")

if __name__ == '__main__':
    main()

# ------------ #
# CLIENT.PY    #                            
# BS. 11.04.25 #
# ------------ #

import socket
import json
import time
import numpy as np

HOST = "127.0.0.1"  
PORT = 5000

with open("config.json", "r") as file:
    config = json.load(file)

material = config["experiment"]["material"]
temperature = config["experiment"]["temperature"]
cycles = config["experiment"]["cycles"]
subcycles = config["experiment"]["subcycles"]
samples_per_subcycle = config["experiment"]["samples_per_subcycle"]
columns = config["grid"]["columns"]
rows = config["grid"]["rows"]
setup = config["robot"]["setup"]
date = time.strftime('%m_%d', time.localtime())
choice = config["robot"]["choice"]

name = f"{material}_{temperature}_{date}"

print(f"Name: {name}")
print(f"Cycles: {cycles}")
print(f"Subycles: {subcycles}")
print(f"Samples per group: {samples_per_subcycle}")
    
# Groups to be sampled during each cycle
Groups = {}
for i in range(cycles):
    Groups[i] = []  # Inicializamos la lista para cada ciclo
    for j in range(subcycles):
        # Añadimos las muestras de cada subciclo directamente a la lista del ciclo
        Groups[i].extend(range((i * subcycles + j) * 11 + 1, (i * subcycles + j) * 11 + samples_per_subcycle + 1))

# Ahora imprimimos los resultados
print(Groups)

hours_delay = config["timing"]["hours_delay"] # Delay between cycles in hours
minutes_delay = config["timing"]["minutes_delay"] # Delay between cycles in minutes
cycle_number = config["experiment"]["starting_cycle"]

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
    SEQUENCE = Groups[cycle_number]
    

    # Define the command and parameters
    command_data = {
        "setup": setup,
        "date": date,
        "material": material,
        "rows": rows,
        "columns": columns,
        "temperature": temperature,
        "samples": [str(i) for i in SEQUENCE],
        "choice": choice,
        "time_delay": time_delay
    }

    # Convert the command to JSON
    json_data = json.dumps(command_data) + '\n'

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(json_data.encode())
        data = s.recv(1024)

    # Parse the JSON response
    response = json.loads(data.decode())
    print(f"Received response: {response}")
    
    cycle_number += 1
    # Check if it is time to break the cycle
    while True:
        if time.strftime('%H%M', time.localtime()) == time_delay:
            break
        time.sleep(1)
        
print("EXPERIMENT COMPLETED!!!")

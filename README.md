# PolyMersion

Automated Procedure for Polymer Degradation Testing through Water Immersion using UR10-e Robot Arm

---
### Project Structure

This repository contains two versions of the experimental workflow:

- `scripts_v1`: Single-bath experiment version.
- `scripts_v2`: Dual-bath experiment version.

Each version is self-contained and configurable via a `config.json` file and a separate README.md file for a more thorough explanation.

---

## Overview

This repository contains the final version of the CAD models, 3D printable STL files, and Python scripts used to automate and execute the experimental workflow for Polymer Degradation Testing using UR10-e Robot Arm. Also includes the code for the Arduino.

## Repository Structure

```
PolyMersion/ 
├── CAD/ 
├── STL/
├── data/
├── Arduino/
├── scripts_v1/
│ ├── README.md
│ ├── main.py 
│ ├── config.json
│ ├── robot.py
│ ├── gripper.py
│ ├── degradation.py
│ ├── environment.py 
│ └── data_processing.py
├── scripts_v2/
│ ├── README.md
│ ├── client.py
│ ├── listener.py
│ ├── config.json
│ ├── robot.py
│ ├── gripper.py
│ ├── degradation.py
│ ├── environment.py 
│ └── data_processing.py
├── requirements.txt
└── README.md
```

## How to use
### 1. 3D Printing Components

- The STL files located in the `STL` folder are ready to be used for 3D printing.
- Print the files according to the material and printer specifications suitable for the experiment.
- Included are files for designing trays for impact and tensile test samples, and a README.md file specifying the tray assembly.

### 2. Wiring the circuit
- Follow the circuit depicted in `Arduino\Schematic.png` using the components listed in the publication describing the experiment or adapting them to your use case.
  
![Circuit schematic](Arduino/Schematic.png)

### 2. Running the Workflow

- 
- The experimental workflow can be executed using the Python script `main.py`.
- Parameters such as the number of specimens, cycle length, and other relevant variables can be adjusted by modifying the `config.json` file.
- No changes to the Python code are necessary—simply update the JSON file to reflect the desired settings.

### 3. Configuration

- The `config.json` file allows users to easily configure the experiment without altering the codebase.
- Example of a `config.json` structure:

```
{
    "robot": {
        "robot_ip": "192.168.9.29",
        "setup": 2,
        "choice": 1 # 1: Extract samples to external tray, 2: Leave the samples in the internal tray	
    },
    "experiment": {
        "material": "PLA",
        "temperature": 40,
        "cycles": 23,
        "subcycles": 1,
	"samples_per_subcycle": 10,
	"starting_cycle": 15
    },
    "grid": {
        "columns": 23,
        "rows": 11
    },
    "timing": {
        "hours_delay": 5,
        "minutes_delay": 0
    }
}
```
> 💡 Each version (`scripts_v1`, `scripts_v2`) may have slightly different config parameters depending on bath logic. Check the `config.json` in each folder for version-specific options.

## Installation
1. Clone this repository:
```
git clone https://github.com/AMDatIMDEA/PolyMersion.git
```

2. Choose the version you want to use:
   - For single bath → `cd PolyMersion/scripts_v1`
   - For dual bath → `cd PolyMersion/scripts_v2`
     
3. Upload the Arduino code:
   ```bash
   cd PolyMersion/Arduino/PoliMersion
   open PoliMersion.ino  # Or use the Arduino IDE
   ```

3. Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

4. Edit the `config.json` file to configure your experiment (see below).

5. Run the workflow following the instructions in the README.md file that you have chosen.

## Requirements

- Python 3.x
- Required libraries listed in `requirements.txt`

You can install them via:
```bash
pip install -r requirements.txt
```

## Compatibility  

This project has been tested on:

- ✅ macOS
- ✅ Linux
  
> ⚠️ If `python` or `pip` refers to Python 2 on your system, use `python3` and `pip3` instead.

## Notes
- Make sure the 3D printed components match the CAD models used in the experiment.
- If adjustments to the CAD models are required, source files can be found in the CAD folder.

# PolyMersion
Automated Procedure for Polymer Degradation Testing through Water Immersion using UR10-e Robot Arm

## Overview

This repository contains the final version of the CAD models, 3D printable STL files, and Python scripts used to automate and execute the experimental workflow for Polymer Degradation Testing using UR10-e Robot Arm. 

## Repository Structure

```
PolyMersion/ 
├── CAD/ 
├── STL/
├── data/
├── arduino/
├── scripts/ 
│ ├── main.py 
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

### 2. Running the Workflow

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
    },
    "experiment": {
        "material": "PLA4043D",
        "temperature": "60C",
        "cycles": 15,
	    "samples_per_group": 10
    },
    "grid": {
        "columns": 23,
        "rows": 11
    },
    "timing": {
        "hours_delay": 12,
        "minutes_delay": 0
    }
}
```
- 

## Requirements

- Python 3.x
- Required libraries listed in `requirements.txt`

## Installation
1. Clone this repository:
```
git clone https://github.com/AMDatIMDEA/PolyMersion.git
```
2. Navigate to the project directory:
```
cd PolyMersion/scripts
```
3. Install dependencies:
```
pip install -r requirements.txt
```
```
pip3 install -r requirements.txt
```

4. Run the main script:
```
python main.py
```
```
python3 main.py
```

## Compatibility  
- This project has been tested on **MacOS** and **Linux** environments.  
- If `python` or `pip` defaults to Python 2, use `python3` and `pip3` to ensure compatibility with Python 3.x.  

## Notes
- Make sure the 3D printed components match the CAD models used in the experiment.
- If adjustments to the CAD models are required, source files can be found in the CAD folder.

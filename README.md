# INFI HAND

## Overview
**INFI HAND** is a robotic hand control system based on the [**ORCA Hand**](https://www.orcahand.com/dashboard) design, with custom modifications, and developed using **Feetech servo motors**.  
It provides control software, calibration tools, and motor control modules to interface with the hardware.  
The project also includes adapted CAD files to fit Feetech servos.

---

## Project Structure
```
INFI_HAND/
├── hls/
│ ├── contrl/
│ │ ├── client3.py # Handles communication with servos
│ │ ├── core3.py # Provides the link between scripts and servos; includes control functions
│ ├── orcahand_v1_right/
│ │ ├── calibration_origin.yaml
│ │ ├── calibration.yaml
│ │ ├── config.yaml
│ ├── scripts/
│ │ ├── calibrate.py # Calibration script
│ │ └── other scripts ...
├── scservo_sdk/ # Feetech Servo SDK
├── modified CAD/ # Adapted CAD files for Feetech servos
├── README.md
├── LICENSE
```

- **`contrl/`**  
  - `client3.py`: Handles communication with servos.  
  - `core3.py`: Establishes the link between scripts and servos; includes control functions.  

- **`scripts/`**  
  Adapted from the official [ORCA Hand repository](https://github.com/orcahand).  
  Only minimal modifications were made to function calls. See the official repository for the original versions.  

- **`modified CAD/`**  
  Contains our adapted CAD files for Feetech servos.  
  Other CAD resources can be found on the [ORCA Hand official site](https://www.orcahand.com/dashboard).

---

## Getting Started

Follow the steps below to set up and use **INFI HAND**:

### Step 1. Create a virtual environment (recommended)
```bash
python -m venv venv
source venv/bin/activate
```
### Step 2. Install dependencies
```bash
pip install -e .
```
### Step 3. Configure your setup
Review the configuration file (e.g., `INFI_hand/hls/orcahand_v1_right/config.yaml`)  
and ensure it matches your hardware setup.
### Step 4. Run tension and calibration scripts
```bash
python -m hls.HAND.scripts.tension hls/HAND/orcahand_v1_right
python -m hls.HAND.scripts.calibrate hls/HAND/orcahand_v1_right
```
### Step 5. Record and replay operations
After debugging and calibration, you can use the `record` and `replay` scripts  
to capture and reproduce a sequence of hand operations.

## LICENSE
This project is licensed under the [MIT License](https://github.com/EMAIL-lab-2025/INFI_hand/blob/main/LICENSE).


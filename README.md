# INFI HAND

## Overview
**INFI HAND** is a robotic hand control system based on the **ORCA Hand** design, with custom modifications, and developed using **Feetech servo motors**.  
It includes control software, calibration tools, and motor control modules that provide an interface between the hardware and software.  
The project also provides adapted CAD files for Feetech servos.

---

## Project Structure

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

markdown
复制
编辑

- **`contrl/`**  
  - `client3.py`: Handles communication with servos.  
  - `core3.py`: Establishes the link between scripts and servos, and contains control functions.  

- **`scripts/`**  
  Scripts are adapted from the official [ORCA Hand repository](https://github.com/orcahand).  
  We only made minimal modifications to function calls. Please refer to the official repository for the original version.  

- **`modified CAD/`**  
  Contains our adapted CAD files to fit Feetech servos.  
  Other CAD resources can be found on the [ORCA Hand official site](https://www.orcahand.com/dashboard).

---

## Getting Started

Follow the steps below to set up and use INFI HAND:

### 1. Create a virtual environment (recommended)
```bash
python -m venv venv
source venv/bin/activate
2. Install dependencies
bash
复制
编辑
pip install -e .
3. Configure your setup
Review the configuration file (e.g., INFI_hand/hls/orcahand_v1_right/config.yaml)
and make sure it matches your hardware setup.

4. Run tension and calibration scripts
bash
复制
编辑
python -m hls.HAND.scripts.tension hls/HAND/orcahand_v1_right
python -m hls.HAND.scripts.calibrate hls/HAND/orcahand_v1_right
Replace the path with your specific hand model folder if needed.

5. Record and replay operations
After debugging and calibration, you can use record and replay scripts
to capture and reproduce a sequence of hand operations.

License
This project is licensed under the MIT License.

Acknowledgments
Feetech Servo SDK

ORCA Hand Project

ORCA Hand Official Website
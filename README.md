# INFI HAND

## Overview
**INFI HAND** is a robotic hand control system based on the [**ORCA Hand**](https://www.orcahand.com/dashboard) design, with custom modifications, and developed using **Feetech servo motors**.  
It provides control software, calibration tools, and motor control modules to interface with the hardware.  
The project also includes adapted CAD files to fit Feetech servos.

---
## Servo Models

- **Servo for Wrist  (×1):** [HL-3930-C001 (12V, 35KG)](https://item.taobao.com/item.htm?id=820985064908&pisk=gt9gTQ9Rh7qjFVw_smXs1PX4ENcKfO6X3EeAktQqTw7Ic5FtfsS2mEvv6n7AnEYDoNCq5OpD-3txfKn1Qxj2yeXvXNQv-mY9c-3sGOe4oFtr5hQxfKbV-F8cGc_AuZYv0CnKeYK6ft6qo4H-e8PE-0Lg3G7Z0JSlmDIZACk2at6qyqVL31MlhF5rflE4TM7CmizNbEWU8iSP3i5V7DbFmgN47EWqxDjcDZSNuGSeYg_UuiW4_kyFcge4btyqxH7C4t72u1oHYwsV3Zk4Xc723CJEooWa8MBWfpjGsa-nN-y0qGaRrhobH-9Ht6bzew243pjMmACM3JkVzQTeTCRubJCAdwvRjEM7CO5DNnIDSA2NpQL6a_OIo7byw36N2hMgKT-w9KjpHmFc7F9HYpf3V81MggKPfUuoB6L9T_dykmPllHdlgQCaJWLyv3p1OK0u8ZOOV9SJfA2hUhR9COdnRWbeKnJ1wTSzCWPrzl2fYmpUGS1NAMbJMR-Uy_RvWTmnxPff_MscyDm3Gj1NAMxExD4MG1SC2x5..)

- **Other Servos (×16):** [HL-3915-C001 (12V, 14KG)](https://item.taobao.com/item.htm?id=944646559414&pisk=gFTUTYZszoU1klwsRdQz3C9l39_d5akjEU65ZQAlO9Xn26guzQRcNXH-9h5kFdh-9wOlULJ1Bz1SvYUyQpODAM9WATflIdAQRTgd4LRXHLaSJH1uzdORqLTywz5kZLhdFXn69BQRrxMjzqOp9j1fmoYFtP2M6sw3rcs3YVRI1xMjl4o3sakEhLOmv3xGgsbltg43Sf55My4kEgjgjOCRZJfu-fRGBOflZajhsPfAZzbuxTfgjsCbxkqhENfGB_blELbo_NXOZaXkEaDqtaba7TSns07ymEHK6MXDtOzuuSCFxROQVzaWX6jerCXw1CTNTMWcq4HzSUR2Ze_w08mHgNLfVFYtzPQBghbGGB3U7ZAHAeQBbAENIFRkJi_r5kbD_3vV6e0bL1KBsI6MgSz2Ow8w7ndZyy6hceOA_Geie1-MMLCcYRZpew5keiLSHyBHZILvcZu_xZAwqK5pPba5FedHqg8SCqIzZ8Ciz8YJY8qPx1CNhflarCuRd_8Ldyr82GNO_tGrauERxCCNhfyY2uIs61WjGgf..)

- **Development Board (×1):**[URT-1](https://item.taobao.com/item.htm?id=575365901461&pisk=g4E8Tu9VrFDWj4OyeWbDKahV53B0pZ2z37y6xXckRSFY9Rpur2cnRDFa62thz00KvSG0qb2oVvgQt5aoNzcHvkF4w3xnVWjKORzVLbDuqpHQl7K3r8cukpnzmzxnZ_oLd5mds1jGj8yrTD1GsxMQWFisIUMWtXMjhDDLo8TkC8yrYX9DON4bUpCY_I7IdX6xcvHwPDgWVjMjgvYSOWGSlIMtQDGQOWgjlvkpAUGIAx6xIA8WVvM5c-MnCeM7OD6YhjkIAYNIAtejgvMShUhJFAKLy1gODA51d11_DY3-9Hcvvuqyj4c1ejxpvbHxyL2SMHtQDyBaXVhdcChUcWnsAVpGw8oYU5HgZit86Rr4MxN1vHc347Erqo66kX4ZB7c01Gtxc-2btVzVTh04Mk3LV4vC1fgg25kLVIWjo-GrVAzA63ouBjnuDDJBy04gIku_VgdZiVkLZjV1fnnbER4iVkXH8YUQIu4d4trgXqlJsfHHPtBv8euS3eXjsvR4moujHfXzUe8E5xkxstpv8eu76xhGUY8e827R.&spm=tbpc.boughtlist.suborder_itemtitle.1.433f2e8dT4YlWJ)

## Note

For the HL-3915 servo, when installing the spools, do not add a nut under the spools' bottom as instructed by ORCA. Instead, use an HM2.5*14 screw to go through the entire spool structure.

---
## Project Structure
```
INFI_HAND/
├── environment/
│ ├── environment.yml
├── hls/
│ ├── HAND/
│ │ ├── contrl/
│ │ │ ├── client3.py # Handles communication with servos
│ │ │ ├── core3.py # Provides the link between scripts and servos; includes control functions
│ │ ├── orcahand_v1_right/
│ │ │ ├── calibration_origin.yaml
│ │ │ ├── calibration.yaml
│ │ │ ├── config.yaml
│ │ ├── scripts/
│ │ │ ├── calibrate.py # Calibration script
│ │ │ └── other scripts ...
│ ├── Some scripts # servo test codes provided by Feetech official sources
├── scservo_sdk/ # Feetech Servo SDK
├── modified_CAD_and_PCB/ 
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

### Step 1. Clone the repository and navigate into it
```bash
git clone https://github.com/EMAIL-lab-2025/INFI_hand.git
cd INFI_hand
```
### Step 2. Create a virtual environmentand install dependencies (recommended) 
```bash
cd environment
conda env create -f environment.yml
conda activate INFI_HAND
cd ..
```
### Step 3. Configure your setup
Review the configuration file (e.g., `INFI_hand/hls/HAND/orcahand_v1_right/config.yaml`)  
and ensure it matches your hardware setup.
### Step 4. Run tension and calibration scripts
```bash
python -m hls.HAND.scripts.tension hls/HAND/orcahand_v1_right
python -m hls.HAND.scripts.calibrate hls/HAND/orcahand_v1_right
```
### Step 5. Try to record and replay
After debugging and calibration, you can run record and replay scripts  
to capture and reproduce a sequence of hand operations.

## LICENSE
This project is licensed under the [MIT License](https://github.com/EMAIL-lab-2025/INFI_hand/blob/main/LICENSE).

## Contact us
- **EMAIL-lab**: email.lab.2025@gmail.com  
- **FJ-Chen**: chenfj20@gmail.com
- **ZL-Guo**: guozeliang20@hotmail.com

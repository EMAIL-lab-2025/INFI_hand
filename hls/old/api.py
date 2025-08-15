from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
from typing import List, Dict, Optional, Union
import time
import numpy as np
from yaml_utils import read_yaml, update_yaml
from core import OrcaHand

app = FastAPI(title="InfiHand BMotor API", version="1.0.0")

# --- Load Configuration ---
current_config_path = None
bhand: Optional[OrcaHand] = None


class MotorList(BaseModel):
    motor_ids: Optional[List[int]] = None


class MaxCurrent(BaseModel):
    current: Union[float, List[float]]


class JointPositions(BaseModel):
    positions: Dict[str, float] = Field(..., example={"index_flex": 0.5, "thumb_flex": 0.2})


def handle_exception(e: Exception):
    if isinstance(e, RuntimeError):
        raise HTTPException(status_code=409, detail=str(e))
    elif isinstance(e, ValueError):
        raise HTTPException(status_code=422, detail=str(e))
    else:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/config")
def set_config(config_path: str = Body(...)):
    global bhand, current_config_path
    try:
        current_config_path = config_path
        bhand = OrcaHand(config_path)
        return {"message": f"Config loaded: {config_path}"}
    except Exception as e:
        handle_exception(e)


@app.get("/config/settings")
def get_config_settings():
    try:
        if not bhand:
            raise HTTPException(status_code=400, detail="No configuration loaded.")
        config_data = read_yaml(bhand.model_path)
        return {"config": config_data}
    except Exception as e:
        handle_exception(e)


@app.post("/connect")
def connect():
    try:
        if bhand and bhand.is_connected():
            return {"message": "Already connected."}
        bhand.connect()
        return {"message": "Connection successful."}
    except Exception as e:
        handle_exception(e)


@app.post("/disconnect")
def disconnect():
    try:
        if bhand:
            bhand.disable_torque()
            time.sleep(0.1)
            bhand.disconnect()
        return {"message": "Disconnected."}
    except Exception as e:
        handle_exception(e)


@app.get("/status")
def get_status():
    try:
        return {
            "connected": bhand.is_connected() if bhand else False,
            "calibrated": bhand.is_calibrated() if bhand else False,
        }
    except Exception as e:
        handle_exception(e)


@app.post("/torque/enable")
def enable_torque(motor_list: MotorList = Body(None)):
    try:
        ids = motor_list.motor_ids if motor_list else None
        bhand.enable_torque(motor_ids=ids)
        return {"message": f"Torque enabled for motors: {ids or 'all'}"}
    except Exception as e:
        handle_exception(e)


@app.post("/torque/disable")
def disable_torque(motor_list: MotorList = Body(None)):
    try:
        ids = motor_list.motor_ids if motor_list else None
        bhand.disable_torque(motor_ids=ids)
        return {"message": f"Torque disabled for motors: {ids or 'all'}"}
    except Exception as e:
        handle_exception(e)


@app.post("/current/max")
def set_max_current(max_current: MaxCurrent):
    try:
        bhand.set_max_current(max_current.current)
        return {"message": "Max current set."}
    except Exception as e:
        handle_exception(e)


@app.get("/motors/position")
def get_motor_position():
    try:
        pos = bhand.get_motor_pos()
        return {"positions": pos.tolist() if pos is not None else None}
    except Exception as e:
        handle_exception(e)


@app.get("/motors/current")
def get_motor_current():
    try:
        cur = bhand.get_motor_current()
        return {"currents": cur.tolist() if cur is not None else None}
    except Exception as e:
        handle_exception(e)


@app.get("/motors/temperature")
def get_motor_temperature():
    try:
        temp = bhand.get_motor_temp()
        return {"temperatures": temp.tolist() if temp is not None else None}
    except Exception as e:
        handle_exception(e)


@app.get("/joints/position")
def get_joint_position():
    try:
        return {"positions": bhand.get_joint_pos()}
    except Exception as e:
        handle_exception(e)


@app.post("/joints/position")
def set_joint_position(joint_positions: JointPositions):
    try:
        bhand.set_joint_pos(joint_positions.positions)
        return {"message": "Joint positions command sent."}
    except Exception as e:
        handle_exception(e)


@app.get("/calibrate/status")
def get_calibration_status():
    try:
        return {"calibrated": bhand.is_calibrated()}
    except Exception as e:
        handle_exception(e)


@app.post("/calibrate")
def calibrate():
    try:
        if not bhand.is_connected():
            raise HTTPException(status_code=409, detail="Must connect first.")
        bhand.calibrate()
        return {"message": "Calibration complete.", "calibrated": bhand.is_calibrated()}
    except Exception as e:
        handle_exception(e)


@app.post("/calibrate/manual")
def manual_calibrate():
    try:
        if not bhand.is_connected():
            raise HTTPException(status_code=409, detail="Must connect first.")
        bhand.manual_calibrate()
        return {"message": "Manual calibration complete.", "calibrated": bhand.is_calibrated()}
    except Exception as e:
        handle_exception(e)


@app.post("/config/save")
def save_config():
    try:
        update_yaml(bhand.model_path, "joint_offsets", bhand.joint_offsets)
        return {"message": f"Configuration saved to {bhand.model_path}"}
    except Exception as e:
        handle_exception(e)

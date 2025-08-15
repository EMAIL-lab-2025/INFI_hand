import atexit
import logging
import time
from typing import Optional, Sequence, Union, Tuple
import numpy as np
import sys

sys.path.append("..")
from scservo_sdk import *  
from scservo_sdk.hls import hls

DEFAULT_POS_SCALE = 2.0 * np.pi / 4095
DEFAULT_VEL_SCALE = 1.0
DEFAULT_CUR_SCALE = 1.0

# Register addresses
ADDR_TORQUE_ENABLE = HLS_TORQUE_ENABLE
ADDR_PRESENT_POSITION = HLS_PRESENT_POSITION_L
ADDR_PRESENT_SPEED = HLS_PRESENT_SPEED_L
ADDR_PRESENT_CURRENT = HLS_PRESENT_CURRENT_L
ADDR_PRESENT_TEMPERATURE = HLS_PRESENT_TEMPERATURE

ADDR_GOAL_ACCELERATION = 0x29
ADDR_GOAL_SPEED = 0x2E
ADDR_GOAL_TORQUE = 0x2C
ADDR_PROTECT_CURRENT = 0x1C  # optional

class BMotorClient:
    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = "/dev/ttyUSB0",
                 baudrate: int = 1000000,
                 pos_scale: Optional[float] = DEFAULT_POS_SCALE,
                 vel_scale: Optional[float] = DEFAULT_VEL_SCALE,
                 cur_scale: Optional[float] = DEFAULT_CUR_SCALE):
        import serial
        self.motor_ids = list(motor_ids)
        self.port = port
        self.baudrate = baudrate
        self.pos_scale = pos_scale
        self.vel_scale = vel_scale
        self.cur_scale = cur_scale

        self.ph = PortHandler(port)
        self.ph.setBaudRate(baudrate)
        self.packetHandler = hls(self.ph)

    def connect(self):
        if self.ph.openPort():
            logging.info(f"Opened port {self.port} with baudrate {self.baudrate}")
            return True
        else:
            raise RuntimeError(f"Failed to open port {self.port}")

    def disconnect(self):
        self.ph.closePort()

    def is_connected(self) -> bool:
        return self.ph.is_open

    def set_torque_enabled(self, motor_ids: Sequence[int], enabled: bool):
        for mid in motor_ids:
            result, error = self.packetHandler.write1ByteTxRx(mid, ADDR_TORQUE_ENABLE, int(enabled))
            self._check_error(mid, result, error, "Torque Enable")

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        logging.warning("SCServo does not support dynamic operating mode switching.")

    def write_position_ex(self, motor_ids: Sequence[int], positions: np.ndarray,
                          speeds: Optional[np.ndarray] = None,
                          accels: Optional[np.ndarray] = None,
                          currents: Optional[np.ndarray] = None):
        speeds = speeds if speeds is not None else np.full(len(motor_ids), 60)
        accels = accels if accels is not None else np.full(len(motor_ids), 50)
        currents = currents if currents is not None else np.full(len(motor_ids), 500)

        for i, mid in enumerate(motor_ids):
            pos = int(positions[i] / self.pos_scale)
            spd = int(speeds[i] / self.vel_scale)
            acc = int(accels[i])
            cur = int(currents[i] / self.cur_scale)
            result, error = self.packetHandler.RegWritePosEx(mid, pos, spd, acc, cur)
            self._check_error(mid, result, error, "RegWritePosEx")

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos, vel, cur = [], [], []
        for mid in self.motor_ids:
            p, _, _ = self.packetHandler.ReadPos(mid)
            v, _, _ = self.packetHandler.ReadSpeed(mid)
            c, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_CURRENT)
            pos.append(p * self.pos_scale)
            vel.append(v * self.vel_scale)
            cur.append(self.packetHandler.scs_tohost(c, 15) * self.cur_scale)
        return np.array(pos), np.array(vel), np.array(cur)

    def read_temperature(self) -> np.ndarray:
        temps = []
        for mid in self.motor_ids:
            t, _, _ = self.packetHandler.read1ByteTxRx(mid, ADDR_PRESENT_TEMPERATURE)
            temps.append(t)
        return np.array(temps, dtype=np.float32)

    def _check_error(self, mid, result, error, context):
        if result != COMM_SUCCESS or error != 0:
            msg = f"[Motor {mid}] {context} failed. Result: {self.packetHandler.getTxRxResult(result)}, Error: {self.packetHandler.getRxPacketError(error)}"
            logging.warning(msg)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def enable_torque(self, motor_ids=None):
        self.set_torque_enabled(motor_ids or self.motor_ids, True)

    def disable_torque(self, motor_ids=None):
        self.set_torque_enabled(motor_ids or self.motor_ids, False)

    def read_positions(self) -> dict:
        positions = {}
        for mid in self.motor_ids:
            pos, _, _ = self.packetHandler.ReadPos(mid)
            positions[mid] = pos * self.pos_scale
        return positions

    def write_position_with_speed_acc_torque(self, motor_id, position, speed=20, acc=20, torque=500):
        self.write_positions_with_speed_acc_torque(
            {motor_id: position},
            speed=speed,
            acc=acc,
            torque=torque
        )

    def write_positions_with_speed_acc_torque(self, position_dict, speed=20, acc=20, torque=500):
        for motor_id, pos in position_dict.items():
            self.write_position(motor_id, pos)
            self.set_speed(motor_id, speed)
            self.set_acceleration(motor_id, acc)
            self.set_torque_limit(motor_id, torque)

    def write_position(self, motor_id: int, position: float):
        pos_int = int(position / self.pos_scale)
        result, error = self.packetHandler.WritePos(motor_id, pos_int, 0)
        self._check_error(motor_id, result, error, "WritePos")

    def set_speed(self, motor_id: int, speed: float):
        spd_int = int(speed / self.vel_scale)
        result, error = self.packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_SPEED, spd_int)
        self._check_error(motor_id, result, error, "SetSpeed")

    def set_acceleration(self, motor_id: int, acc: float):
        acc_int = int(acc)
        result, error = self.packetHandler.write1ByteTxRx(motor_id, ADDR_GOAL_ACCELERATION, acc_int)
        self._check_error(motor_id, result, error, "SetAccel")

    def set_torque_limit(self, motor_id: int, torque: float):
        torque_int = int(torque)
        result, error = self.packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_TORQUE, torque_int)
        self._check_error(motor_id, result, error, "SetTorque")

    def set_protect_current(self, motor_id: int, current: float = 100):
        cur_int = int(current / self.cur_scale)
        result, error = self.packetHandler.write2ByteTxRx(motor_id, ADDR_PROTECT_CURRENT, cur_int)
        self._check_error(motor_id, result, error, "SetProtectCurrent")

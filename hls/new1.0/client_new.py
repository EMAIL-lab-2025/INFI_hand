import atexit
import logging
import time
from typing import Optional, Sequence, Union, Tuple
import numpy as np
import sys

sys.path.append("..")
from scservo_sdk import *  
from scservo_sdk.protocol_packet_handler import protocol_packet_handler
from scservo_sdk.group_sync_write import GroupSyncWrite

DEFAULT_POS_SCALE = 2.0 * np.pi / 4095
DEFAULT_VEL_SCALE = 1.0
DEFAULT_CUR_SCALE = 1.0

# Register addresses
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION_L = 42
ADDR_GOAL_SPEED_L = 46
ADDR_PRESENT_POSITION_L = 56
ADDR_PRESENT_SPEED_L = 58
ADDR_PRESENT_CURRENT_L = 69
ADDR_PRESENT_TEMPERATURE = 63
ADDR_GOAL_ACCELERATION = 41
ADDR_GOAL_TORQUE_L = 44

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
        self.packetHandler = protocol_packet_handler(self.ph, 0)
        self.groupSyncWrite = GroupSyncWrite(self.packetHandler, ADDR_GOAL_ACCELERATION, 7)

    def connect(self):
        try:
            if self.ph.openPort():
                logging.info(f"Opened port {self.port} with baudrate {self.baudrate}")
                return True, "Connection successful"
            else:
                return False, f"Connection failed: Failed to open port {self.port}"
        except Exception as e:
            return False, f"Connection failed: {str(e)}"


    def disconnect(self):
        self.ph.closePort()

    def is_connected(self) -> bool:
        return self.ph.is_open

    def set_torque_enabled(self, motor_ids: Sequence[int], enabled: bool):
        for mid in motor_ids:
            result, error = self.packetHandler.write1ByteTxRx(mid, ADDR_TORQUE_ENABLE, int(enabled))
            self._check_error(mid, result, error, "Torque Enable")

    def write_position(self, motor_id: int, position: float):
        pos_int = int(position / self.pos_scale)
        result, error = self.packetHandler.WritePos(motor_id, pos_int, 0)
        self._check_error(motor_id, result, error, "WritePos")

    def set_speed(self, motor_id: int, speed: float):
        spd_int = int(speed / self.vel_scale)
        result, error = self.packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_SPEED_L, spd_int)
        self._check_error(motor_id, result, error, "SetSpeed")

    def set_acceleration(self, motor_id: int, acc: float):
        acc_int = int(acc)
        result, error = self.packetHandler.write1ByteTxRx(motor_id, ADDR_GOAL_ACCELERATION, acc_int)
        self._check_error(motor_id, result, error, "SetAccel")

    def set_torque_limit(self, motor_id: int, torque: float):
        torque_int = int(torque)
        result, error = self.packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_TORQUE_L, torque_int)
        self._check_error(motor_id, result, error, "SetTorque")

    def write_position_with_speed_acc_torque(self, motor_id, position, speed=20, acc=20, torque=150):
        self.write_positions_with_speed_acc_torque(
            {motor_id: position},
            speed=speed,
            acc=acc,
            torque=torque
        )

    def write_positions_with_speed_acc_torque(self, position_dict, speed=20, acc=20, torque=150):
        for motor_id, pos in position_dict.items():
            pos_int = int(pos / self.pos_scale)
            spd_int = int(speed / self.vel_scale)
            acc_int = int(acc)
            torque_int = int(torque / self.cur_scale)
            txpacket = [acc_int, pos_int & 0xFF, (pos_int >> 8) & 0xFF,
                        torque_int & 0xFF, (torque_int >> 8) & 0xFF,
                        spd_int & 0xFF, (spd_int >> 8) & 0xFF]
            self.packetHandler.writeTxRx(motor_id, ADDR_GOAL_ACCELERATION, len(txpacket), txpacket)

    def read_positions(self) -> dict:
        positions = {}
        for mid in self.motor_ids:
            pos, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_POSITION_L)
            positions[mid] = self.packetHandler.scs_tohost(pos, 15) * self.pos_scale
        return positions

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos = []
        vel = []
        cur = []
        for mid in self.motor_ids:
            p, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_POSITION_L)
            v, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_SPEED_L)
            c, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_CURRENT_L)

            pos.append(self.packetHandler.scs_tohost(p, 15) * self.pos_scale)
            vel.append(self.packetHandler.scs_tohost(v, 15) * self.vel_scale)
            cur.append(self.packetHandler.scs_tohost(c, 15) * self.cur_scale)

        return np.array(pos), np.array(vel), np.array(cur)

    def read_currents(self) -> dict:
        currents = {}
        for mid in self.motor_ids:
            c, _, _ = self.packetHandler.read2ByteTxRx(mid, ADDR_PRESENT_CURRENT_L)
            currents[mid] = self.packetHandler.scs_tohost(c, 15) * self.cur_scale
        return currents

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
